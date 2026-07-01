#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

class GlobalLocalizationNode: public rclcpp::Node {
public:
    GlobalLocalizationNode(): rclcpp::Node("fast_lio_localization") {
        // Parameters
        this->declare_parameter<bool>("map2odom_completed", false);
        this->declare_parameter<int>("region", 0);
        this->declare_parameter<double>("freq_localization", 0.5);
        this->declare_parameter<double>("localization_th", 0.25); // MSE threshold (lower better)
        this->declare_parameter<double>("map_voxel_size", 0.2);
        this->declare_parameter<double>("scan_voxel_size", 0.1);
        this->declare_parameter<double>("fov", 6.28);
        this->declare_parameter<double>("fov_far", 30.0);
        this->declare_parameter<std::string>("map_frame", "map3d");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_link_frame", "base_link");
        // Keep param for compatibility; not used
        this->declare_parameter<bool>("use_gicp", false);

        // Initial pose parameters for global initialization
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_z", 0.0);
        this->declare_parameter<double>("initial_yaw", 0.0); // in radians
        this->declare_parameter<bool>("use_initial_pose", false); // enable initial pose

        // Multi-hypothesis and global search initialization
        this->declare_parameter<bool>(
            "enable_multi_hypothesis",
            true
        ); // try multiple initial poses
        this->declare_parameter<int>("max_init_attempts", 10); // max attempts before giving up
        this->declare_parameter<bool>("enable_global_search", true); // enable global grid search
        this->declare_parameter<double>("global_search_step", 2.0); // grid step in meters
        this->declare_parameter<int>("global_search_yaw_steps", 8); // number of yaw angles to try

        // QoS
        rclcpp::QoS qos_reliable(10);
        qos_reliable.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        rclcpp::QoS qos_transient(1);
        qos_transient.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_transient.transient_local();

        // Publishers
        pub_pc_in_map_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan_in_map", 1);
        pub_submap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 1);
        pub_map_to_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 1);
        pub_initialpose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            qos_transient
        );

        // Subscriptions
        sub_cloud_registered_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered",
            qos_reliable,
            std::bind(&GlobalLocalizationNode::cbSaveCurScan, this, std::placeholders::_1)
        );
        sub_aft_mapped_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            qos_reliable,
            std::bind(&GlobalLocalizationNode::cbSaveCurOdom, this, std::placeholders::_1)
        );
        sub_map3d_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/map3d",
            qos_reliable,
            std::bind(&GlobalLocalizationNode::cbInitGlobalMap, this, std::placeholders::_1)
        );
        // Subscribe to /initialpose from RViz "2D Pose Estimate"
        sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose_rviz",
            10,
            std::bind(&GlobalLocalizationNode::cbInitialPose, this, std::placeholders::_1)
        );

        // TF buffer/listener (used to convert RViz base_link pose -> map3d->odom)
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Thread
        worker_ = std::thread(&GlobalLocalizationNode::initializeAndRun, this);
        RCLCPP_INFO(this->get_logger(), "Localization Node Inited (C++) ...");
    }

    ~GlobalLocalizationNode() override {
        alive_.store(false);
        if (worker_.joinable())
            worker_.join();
    }

private:
    // Utils
    static Eigen::Matrix4d poseToMat(const nav_msgs::msg::Odometry& odom) {
        const auto& p = odom.pose.pose.position;
        const auto& q = odom.pose.pose.orientation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
        T(0, 3) = p.x;
        T(1, 3) = p.y;
        T(2, 3) = p.z;
        return T;
    }

    static Eigen::Matrix4d inverseSE3(const Eigen::Matrix4d& T) {
        Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
        inv.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
        inv.block<3, 1>(0, 3) = -inv.block<3, 3>(0, 0) * T.block<3, 1>(0, 3);
        return inv;
    }

    static Eigen::Matrix4d transformToMat(const geometry_msgs::msg::TransformStamped& tf) {
        const auto& t = tf.transform.translation;
        const auto& q = tf.transform.rotation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
        T(0, 3) = t.x;
        T(1, 3) = t.y;
        T(2, 3) = t.z;
        return T;
    }

    // Atomically pull a pending manual pose (set by the RViz callback) into the
    // localization estimate. Returns true if a manual pose was applied.
    bool consumePendingManualPose() {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        if (manual_pose_pending_) {
            T_map_to_odom_ = pending_pose_;
            manual_pose_pending_ = false;
            return true;
        }
        return false;
    }

    static pcl::PointCloud<pcl::PointXYZ>::Ptr
    voxelDownSample(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, double voxel) {
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(voxel, voxel, voxel);
        vg.setInputCloud(cloud);
        auto filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        vg.filter(*filtered);
        return filtered;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr cropGlobalMapInFOV(
        const pcl::PointCloud<pcl::PointNormal>::ConstPtr& global_map,
        const Eigen::Matrix4d& pose_estimation,
        const nav_msgs::msg::Odometry& cur_odom
    ) {
        Eigen::Matrix4d T_odom_to_base = poseToMat(cur_odom);
        Eigen::Matrix4d T_map_to_base = pose_estimation * T_odom_to_base;
        Eigen::Matrix4d T_base_to_map = inverseSE3(T_map_to_base);

        double FOV = this->get_parameter("fov").as_double();
        double FOV_FAR = this->get_parameter("fov_far").as_double();

        auto submap =
            pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        submap->points.reserve(global_map->points.size() / 10);
        for (const auto& pt_map: global_map->points) {
            Eigen::Vector4d pm(pt_map.x, pt_map.y, pt_map.z, 1.0);
            Eigen::Vector4d pb = T_base_to_map * pm;
            double x = pb.x();
            double y = pb.y();
            double z = pb.z();
            double ang = std::atan2(y, x);
            bool in = false;
            if (FOV > 3.14) {
                in = (x < FOV_FAR) && (std::fabs(ang) < FOV / 2.0);
            } else {
                in = (x > 0.0) && (x < FOV_FAR) && (std::fabs(ang) < FOV / 2.0);
            }
            if (in) {
                pcl::PointNormal p;
                p.x = static_cast<float>(pt_map.x);
                p.y = static_cast<float>(pt_map.y);
                p.z = static_cast<float>(pt_map.z);
                p.normal_x = pt_map.normal_x;
                p.normal_y = pt_map.normal_y;
                p.normal_z = pt_map.normal_z;
                submap->points.push_back(p);
            }
        }
        submap->width = static_cast<uint32_t>(submap->points.size());
        submap->height = 1;
        submap->is_dense = false;

        // Publish for debug (downsampled for bandwidth)
        if (!submap->empty()) {
            auto ds = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            ds->points.reserve(submap->points.size());
            for (const auto& pn: submap->points) {
                pcl::PointXYZ p;
                p.x = pn.x;
                p.y = pn.y;
                p.z = pn.z;
                ds->points.push_back(p);
            }
            ds->width = static_cast<uint32_t>(ds->points.size());
            ds->height = 1;
            ds->is_dense = false;
            if (ds->size() > 200000)
                ds->points.resize(200000);
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*ds, msg);
            msg.header.stamp = this->now();
            msg.header.frame_id = this->get_parameter("map_frame").as_string();
            pub_submap_->publish(msg);
        }
        return submap;
    }

    // Estimate normals and build PointNormal for scan
    static pcl::PointCloud<pcl::PointNormal>::Ptr
    buildScanWithNormals(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& scan_xyz, double radius) {
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(scan_xyz);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        if (radius > 0.0)
            ne.setRadiusSearch(radius);
        else
            ne.setKSearch(20);
        auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
        ne.compute(*normals);
        auto out = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        out->points.resize(scan_xyz->points.size());
        for (size_t i = 0; i < scan_xyz->points.size(); ++i) {
            pcl::PointNormal pn;
            pn.x = scan_xyz->points[i].x;
            pn.y = scan_xyz->points[i].y;
            pn.z = scan_xyz->points[i].z;
            pn.normal_x = normals->points[i].normal_x;
            pn.normal_y = normals->points[i].normal_y;
            pn.normal_z = normals->points[i].normal_z;
            out->points[i] = pn;
        }
        out->width = static_cast<uint32_t>(out->points.size());
        out->height = 1;
        out->is_dense = false;
        return out;
    }

    static pcl::PointCloud<pcl::PointNormal>::Ptr
    voxelDownSampleNormals(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud, double voxel) {
        pcl::VoxelGrid<pcl::PointNormal> vg;
        vg.setLeafSize(voxel, voxel, voxel);
        vg.setInputCloud(cloud);
        auto filtered =
            pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        vg.filter(*filtered);
        return filtered;
    }

    std::pair<Eigen::Matrix4d, double> registrationAtScale(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& scan,
        const pcl::PointCloud<pcl::PointNormal>::ConstPtr& map,
        const Eigen::Matrix4d& initial,
        double scale
    ) {
        double scan_voxel = this->get_parameter("scan_voxel_size").as_double() * scale;
        double map_voxel = this->get_parameter("map_voxel_size").as_double() * scale;
        // Downsample scan first (XYZ), then estimate normals to reduce cost
        auto ds_scan_xyz = voxelDownSample(scan, std::max(0.01, scan_voxel));
        // Estimate normals for scan with a radius tied to voxel size
        double normal_radius = std::max(0.02, 2.5 * scan_voxel);
        auto ds_scan = buildScanWithNormals(ds_scan_xyz, normal_radius);

        // Register against the FOV submap (NOT the whole global map). The submap is
        // already cropped around the current estimate; downsample it coarse-to-fine
        // via the scale so each ICP target stays small and fast.
        auto ds_map = (map && !map->empty())
            ? voxelDownSampleNormals(map, std::max(0.01, map_voxel))
            : map;
        if (!ds_map || ds_map->empty()) {
            // Empty target (estimate far outside the map FOV): report failure.
            return { initial, std::numeric_limits<double>::max() };
        }

        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
        icp.setMaxCorrespondenceDistance(1.0 * scale);
        icp.setMaximumIterations(35);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-8);
        icp.setInputSource(ds_scan);
        icp.setInputTarget(ds_map);
        pcl::PointCloud<pcl::PointNormal> aligned;
        Eigen::Matrix4f initf = initial.cast<float>();
        icp.align(aligned, initf);
        Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();
        double mse = icp.getFitnessScore(1.0 * scale); // lower is better
        return { T, mse };
    }

    void publishInitialPose() {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = this->get_parameter("map_frame").as_string();
        msg.header.stamp = this->now();
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation.w = 1.0;
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = 0.0;
        pub_initialpose_->publish(msg);
    }

    void publishMapToOdom(
        const Eigen::Matrix4d& transform,
        const rclcpp::Time& stamp,
        const char* reason
    ) {
        nav_msgs::msg::Odometry map_to_odom;
        Eigen::Quaterniond q(transform.block<3, 3>(0, 0));
        q.normalize();
        map_to_odom.pose.pose.position.x = transform(0, 3);
        map_to_odom.pose.pose.position.y = transform(1, 3);
        map_to_odom.pose.pose.position.z = transform(2, 3);
        map_to_odom.pose.pose.orientation.x = q.x();
        map_to_odom.pose.pose.orientation.y = q.y();
        map_to_odom.pose.pose.orientation.z = q.z();
        map_to_odom.pose.pose.orientation.w = q.w();
        map_to_odom.header.stamp = stamp;
        map_to_odom.header.frame_id = this->get_parameter("map_frame").as_string();
        pub_map_to_odom_->publish(map_to_odom);

        RCLCPP_INFO(
            this->get_logger(),
            "Published map3d->odom (%s): x=%.2f, y=%.2f, z=%.2f",
            reason,
            transform(0, 3),
            transform(1, 3),
            transform(2, 3)
        );
    }

    bool globalLocalization(Eigen::Matrix4d& pose_estimation) {
        auto gm = global_map_;
        if (!gm || !refreshCurrentScan())
            return false;
        auto scan = std::atomic_load(&cur_scan_);
        auto odom = std::atomic_load(&cur_odom_);
        if (!scan || !odom)
            return false;

        // RCLCPP_INFO_THROTTLE(
        //     this->get_logger(),
        //     *this->get_clock(),
        //     1000,
        //     "Global localization by scan-to-map matching ..."
        // );
        auto submap = cropGlobalMapInFOV(gm, pose_estimation, *odom);

        auto [T1, mse1] = registrationAtScale(scan, submap, pose_estimation, 5.0);
        auto [T2, mse2] = registrationAtScale(scan, submap, T1, 1.0);
        double mse = mse2;

        bool map2odom_completed = this->get_parameter("map2odom_completed").as_bool();
        int region_id = this->get_parameter("region").as_int();
        if (map2odom_completed
            || (region_id == 2 || region_id == 4 || region_id == 5 || region_id == 6
                || region_id == 7))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "LOCALIZATION DISABLED: region=%d, map2odom_completed=%s",
                region_id,
                map2odom_completed ? "true" : "false"
            );
            return true;
        }

        // Direct MSE threshold: mse < localization_th
        double mse_th = std::max(1e-6, this->get_parameter("localization_th").as_double());
        if (mse < mse_th) {
            pose_estimation = T2;
            publishMapToOdom(T2, odom->header.stamp, "relocalization");
            RCLCPP_INFO(this->get_logger(), "Relocalization mse: %.6f (mse_th=%.6f)", mse, mse_th);
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Not match (mse=%.6f, mse_th=%.6f)", mse, mse_th);
            return false;
        }
    }

    // Generate initial pose from parameters
    Eigen::Matrix4d getInitialPoseFromParams() {
        double x = this->get_parameter("initial_x").as_double();
        double y = this->get_parameter("initial_y").as_double();
        double z = this->get_parameter("initial_z").as_double();
        double yaw = this->get_parameter("initial_yaw").as_double();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(0, 3) = x;
        T(1, 3) = y;
        T(2, 3) = z;
        T(0, 0) = std::cos(yaw);
        T(0, 1) = -std::sin(yaw);
        T(1, 0) = std::sin(yaw);
        T(1, 1) = std::cos(yaw);
        return T;
    }

    // Compute map bounding box from global map
    void
    computeMapBounds(double& min_x, double& max_x, double& min_y, double& max_y, double& avg_z) {
        min_x = std::numeric_limits<double>::max();
        max_x = std::numeric_limits<double>::lowest();
        min_y = std::numeric_limits<double>::max();
        max_y = std::numeric_limits<double>::lowest();
        double sum_z = 0.0;
        int count = 0;

        for (const auto& pt: global_map_->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;
            min_x = std::min(min_x, static_cast<double>(pt.x));
            max_x = std::max(max_x, static_cast<double>(pt.x));
            min_y = std::min(min_y, static_cast<double>(pt.y));
            max_y = std::max(max_y, static_cast<double>(pt.y));
            sum_z += pt.z;
            count++;
        }
        avg_z = (count > 0) ? (sum_z / count) : 0.0;
    }

    // Global grid search: search entire map for best matching position
    bool tryGlobalGridSearch() {
        if (!refreshCurrentScan())
            return false;
        auto scan = std::atomic_load(&cur_scan_);
        auto odom = std::atomic_load(&cur_odom_);
        auto gm = global_map_;
        if (!scan || !odom || !gm)
            return false;

        double grid_step = this->get_parameter("global_search_step").as_double();
        int num_yaw = this->get_parameter("global_search_yaw_steps").as_int();
        double mse_th = std::max(1e-6, this->get_parameter("localization_th").as_double());

        // Compute map bounds
        double min_x, max_x, min_y, max_y, avg_z;
        computeMapBounds(min_x, max_x, min_y, max_y, avg_z);

        // Add margin
        double margin = 1.0;
        min_x += margin;
        max_x -= margin;
        min_y += margin;
        max_y -= margin;

        int nx = static_cast<int>(std::ceil((max_x - min_x) / grid_step)) + 1;
        int ny = static_cast<int>(std::ceil((max_y - min_y) / grid_step)) + 1;
        int total = nx * ny * num_yaw;

        RCLCPP_INFO(this->get_logger(), "=== GLOBAL GRID SEARCH ===");
        RCLCPP_INFO(
            this->get_logger(),
            "Map bounds: x=[%.1f, %.1f], y=[%.1f, %.1f]",
            min_x,
            max_x,
            min_y,
            max_y
        );
        RCLCPP_INFO(
            this->get_logger(),
            "Grid: %dx%d positions, %d yaw angles, total %d candidates",
            nx,
            ny,
            num_yaw,
            total
        );

        double best_mse = std::numeric_limits<double>::max();
        Eigen::Matrix4d best_T = Eigen::Matrix4d::Identity();
        double best_x = 0, best_y = 0, best_yaw = 0;
        int tested = 0;

        // Coarse grid search
        for (int ix = 0; ix < nx && alive_.load(); ++ix) {
            double x = min_x + ix * grid_step;
            for (int iy = 0; iy < ny && alive_.load(); ++iy) {
                double y = min_y + iy * grid_step;

                for (int iyaw = 0; iyaw < num_yaw; ++iyaw) {
                    double yaw = iyaw * 2.0 * M_PI / num_yaw;

                    Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity();
                    T_init(0, 3) = x;
                    T_init(1, 3) = y;
                    T_init(2, 3) = avg_z;
                    T_init(0, 0) = std::cos(yaw);
                    T_init(0, 1) = -std::sin(yaw);
                    T_init(1, 0) = std::sin(yaw);
                    T_init(1, 1) = std::cos(yaw);

                    auto submap = cropGlobalMapInFOV(gm, T_init, *odom);
                    if (submap->size() < 100)
                        continue;

                    auto [T1, mse1] = registrationAtScale(scan, submap, T_init, 5.0);

                    tested++;
                    if (mse1 < best_mse) {
                        best_mse = mse1;
                        best_T = T1;
                        best_x = x;
                        best_y = y;
                        best_yaw = yaw;
                    }
                }
            }

            if ((ix + 1) % 5 == 0 || ix == nx - 1) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "  Progress: %d/%d rows, tested %d, best_mse=%.6f at (%.1f, %.1f, %.0f deg)",
                    ix + 1,
                    nx,
                    tested,
                    best_mse,
                    best_x,
                    best_y,
                    best_yaw * 180.0 / M_PI
                );
            }
        }

        // Refine the best candidate
        if (best_mse < mse_th * 5.0) {
            RCLCPP_INFO(this->get_logger(), "Coarse search found candidate. Refining...");

            double fine_step = grid_step / 4.0;
            for (double dx = -grid_step; dx <= grid_step; dx += fine_step) {
                for (double dy = -grid_step; dy <= grid_step; dy += fine_step) {
                    for (int iyaw = 0; iyaw < num_yaw * 2; ++iyaw) {
                        double yaw = best_yaw + (iyaw - num_yaw) * M_PI / (num_yaw * 4);

                        Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity();
                        T_init(0, 3) = best_x + dx;
                        T_init(1, 3) = best_y + dy;
                        T_init(2, 3) = avg_z;
                        T_init(0, 0) = std::cos(yaw);
                        T_init(0, 1) = -std::sin(yaw);
                        T_init(1, 0) = std::sin(yaw);
                        T_init(1, 1) = std::cos(yaw);

                        auto submap = cropGlobalMapInFOV(gm, T_init, *odom);
                        if (submap->size() < 100)
                            continue;

                        auto [T1, mse1] = registrationAtScale(scan, submap, T_init, 3.0);
                        auto [T2, mse2] = registrationAtScale(scan, submap, T1, 1.0);

                        if (mse2 < best_mse) {
                            best_mse = mse2;
                            best_T = T2;
                        }
                    }
                }
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "=== GLOBAL SEARCH RESULT: mse=%.6f (threshold=%.6f) ===",
            best_mse,
            mse_th
        );

        if (best_mse < mse_th) {
            T_map_to_odom_ = best_T;
            RCLCPP_INFO(this->get_logger(), "GLOBAL INIT SUCCESS!");
            publishMapToOdom(best_T, odom->header.stamp, "global search");
            return true;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "GLOBAL INIT FAILED, best_mse=%.6f > threshold=%.6f",
            best_mse,
            mse_th
        );
        return false;
    }

    // Multi-hypothesis initialization: try different yaw angles at current position
    bool tryMultiHypothesisInit() {
        if (!refreshCurrentScan())
            return false;
        auto scan = std::atomic_load(&cur_scan_);
        auto odom = std::atomic_load(&cur_odom_);
        auto gm = global_map_;
        if (!scan || !odom || !gm)
            return false;

        double best_mse = std::numeric_limits<double>::max();
        Eigen::Matrix4d best_T = Eigen::Matrix4d::Identity();

        double init_x = this->get_parameter("initial_x").as_double();
        double init_y = this->get_parameter("initial_y").as_double();
        double init_z = this->get_parameter("initial_z").as_double();

        RCLCPP_INFO(
            this->get_logger(),
            "Multi-hypothesis init: trying 8 yaw angles at (%.2f, %.2f, %.2f)",
            init_x,
            init_y,
            init_z
        );

        for (int i = 0; i < 8; ++i) {
            double yaw = i * M_PI / 4.0;

            Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity();
            T_init(0, 3) = init_x;
            T_init(1, 3) = init_y;
            T_init(2, 3) = init_z;
            T_init(0, 0) = std::cos(yaw);
            T_init(0, 1) = -std::sin(yaw);
            T_init(1, 0) = std::sin(yaw);
            T_init(1, 1) = std::cos(yaw);

            auto submap = cropGlobalMapInFOV(gm, T_init, *odom);
            if (submap->empty())
                continue;

            auto [T1, mse1] = registrationAtScale(scan, submap, T_init, 5.0);
            auto [T2, mse2] = registrationAtScale(scan, submap, T1, 1.0);

            RCLCPP_INFO(this->get_logger(), "  yaw=%.0f deg, mse=%.6f", yaw * 180.0 / M_PI, mse2);

            if (mse2 < best_mse) {
                best_mse = mse2;
                best_T = T2;
            }
        }

        double mse_th = std::max(1e-6, this->get_parameter("localization_th").as_double());
        if (best_mse < mse_th) {
            T_map_to_odom_ = best_T;
            RCLCPP_INFO(
                this->get_logger(),
                "Multi-hypothesis init SUCCESS! best_mse=%.6f",
                best_mse
            );
            publishMapToOdom(best_T, odom->header.stamp, "multi-hypothesis init");
            return true;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "Multi-hypothesis init FAILED, best_mse=%.6f > th=%.6f",
            best_mse,
            mse_th
        );
        return false;
    }

    void initializeAndRun() {
        while (alive_.load() && rclcpp::ok() && !global_map_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for global map ...");
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        // Wait for first scan and odom
        while (alive_.load() && rclcpp::ok()) {
            auto scan = std::atomic_load(&latest_scan_msg_);
            auto odom = std::atomic_load(&cur_odom_);
            if (scan && odom)
                break;
            if (!scan)
                RCLCPP_WARN(this->get_logger(), "Waiting for first scan ...");
            if (!odom)
                RCLCPP_WARN(this->get_logger(), "Waiting for odom ...");
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        // Check parameters
        bool use_initial_pose = this->get_parameter("use_initial_pose").as_bool();
        bool enable_multi_hypothesis = this->get_parameter("enable_multi_hypothesis").as_bool();
        bool enable_global_search = this->get_parameter("enable_global_search").as_bool();
        int max_attempts = this->get_parameter("max_init_attempts").as_int();

        if (use_initial_pose) {
            T_map_to_odom_ = getInitialPoseFromParams();
            auto odom = std::atomic_load(&cur_odom_);
            RCLCPP_INFO(
                this->get_logger(),
                "Using initial map3d->odom from parameters: x=%.2f, y=%.2f, yaw=%.2f",
                this->get_parameter("initial_x").as_double(),
                this->get_parameter("initial_y").as_double(),
                this->get_parameter("initial_yaw").as_double()
            );
            if (odom)
                publishMapToOdom(T_map_to_odom_, odom->header.stamp, "initial parameters");
        }

        publishInitialPose();

        // Initialization phase
        int attempt = 0;
        bool initialized = false;
        while (alive_.load() && rclcpp::ok() && !initialized) {
            attempt++;

            // A manual pose from RViz takes priority and is used as the seed.
            if (consumePendingManualPose()) {
                RCLCPP_INFO(this->get_logger(), "Applying manual pose from RViz during init");
            }

            // First try direct matching
            bool ok = globalLocalization(T_map_to_odom_);
            if (ok) {
                initialized = true;
                break;
            }

            // If direct matching fails, try multi-hypothesis
            if (enable_multi_hypothesis && attempt == 1) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Direct matching failed, trying multi-hypothesis init..."
                );
                if (tryMultiHypothesisInit()) {
                    initialized = true;
                    break;
                }
            }

            // If still failed, do full map search
            if (enable_global_search && attempt == 2) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Multi-hypothesis failed, starting GLOBAL GRID SEARCH..."
                );
                if (tryGlobalGridSearch()) {
                    initialized = true;
                    break;
                }
            }

            if (attempt >= max_attempts) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Initialization failed after %d attempts!",
                    attempt
                );
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Use RViz '2D Pose Estimate' on /initialpose_rviz"
                );
            }

            rclcpp::sleep_for(std::chrono::seconds(2));
        }

        if (initialized) {
            RCLCPP_INFO(this->get_logger(), "Initialize successfully!");
        } else {
            RCLCPP_WARN(this->get_logger(), "Waiting for manual pose...");
        }

        double freq = this->get_parameter("freq_localization").as_double();
        double period = (freq > 0.0) ? (1.0 / freq) : 2.0;
        auto sleep_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(period)
        );
        while (alive_.load() && rclcpp::ok()) {
            // Honor any manual pose set via RViz "2D Pose Estimate".
            consumePendingManualPose();
            globalLocalization(T_map_to_odom_);
            rclcpp::sleep_for(sleep_duration);
        }
    }

    // Callbacks
    void cbSaveCurOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::atomic_store(&cur_odom_, msg);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Received odom: pos=(%.2f,%.2f)", msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

    void cbSaveCurScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::atomic_store(&latest_scan_msg_, msg);
    }

    bool refreshCurrentScan() {
        auto msg = std::atomic_load(&latest_scan_msg_);
        if (!msg)
            return false;
        if (msg == processed_scan_msg_)
            return static_cast<bool>(std::atomic_load(&cur_scan_));

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        std::atomic_store(&cur_scan_, cloud);
        processed_scan_msg_ = msg;

        sensor_msgs::msg::PointCloud2 out;
        out = *msg; // keep data as-is
        out.header.frame_id = this->get_parameter("odom_frame").as_string();
        out.header.stamp = this->now();
        pub_pc_in_map_->publish(out);
        return true;
    }

    // Callback for /initialpose from RViz "2D Pose Estimate"
    void cbInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received initial pose from RViz");
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;

        // RViz "2D Pose Estimate" gives a 2D pose in the map frame. The 3D
        // localization state is map3d -> odom, where /odom is the LIO body pose.
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Matrix4d T_map_to_base = Eigen::Matrix4d::Identity();
        T_map_to_base.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
        T_map_to_base(0, 3) = p.x;
        T_map_to_base(1, 3) = p.y;
        T_map_to_base(2, 3) = p.z;

        auto odom = std::atomic_load(&cur_odom_);
        if (!odom) {
            // No odometry yet: cannot relate base_link to odom, store click as map3d->odom.
            {
                std::lock_guard<std::mutex> lk(pose_mutex_);
                pending_pose_ = T_map_to_base;
                manual_pose_pending_ = true;
            }
            RCLCPP_WARN(
                this->get_logger(),
                "No odom available, using initial pose directly as map3d->odom"
            );
            publishMapToOdom(T_map_to_base, this->now(), "rviz initial pose (no odom)");
            publishInitialPose();
            return;
        }

        Eigen::Matrix4d candidate = T_map_to_base * inverseSE3(poseToMat(*odom));
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            pending_pose_ = candidate;
            manual_pose_pending_ = true;
        }
        RCLCPP_INFO(this->get_logger(), "Updated map3d->odom from RViz initial pose");
        publishMapToOdom(candidate, this->now(), "rviz initial pose");
        publishInitialPose();
    }

    void cbInitGlobalMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (global_map_)
            return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        double voxel = this->get_parameter("map_voxel_size").as_double();
        // Downsample first for efficiency, then estimate normals once (static map)
        auto ds_map_xyz = voxelDownSample(cloud, std::max(0.01, voxel));
        // Estimate normals with radius based on voxel size
        double normal_radius = std::max(0.02, 2.5 * voxel);
        global_map_ = buildScanWithNormals(ds_map_xyz, normal_radius);
        RCLCPP_INFO(
            this->get_logger(),
            "Global map received. points=%zu (downsampled + normals)",
            global_map_->points.size()
        );
    }

    // Members
    std::atomic<bool> alive_ { true };
    std::thread worker_;
    pcl::PointCloud<pcl::PointNormal>::Ptr global_map_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_scan_msg_;
    sensor_msgs::msg::PointCloud2::SharedPtr processed_scan_msg_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_scan_;
    nav_msgs::msg::Odometry::SharedPtr cur_odom_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_in_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_to_odom_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_registered_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_aft_mapped_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map3d_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;

    // TF (RViz base_link pose -> map3d->odom conversion)
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    Eigen::Matrix4d T_map_to_odom_ = Eigen::Matrix4d::Identity();

    // Manual pose hand-off from the RViz callback thread to the worker thread.
    std::mutex pose_mutex_;
    Eigen::Matrix4d pending_pose_ = Eigen::Matrix4d::Identity();
    bool manual_pose_pending_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
