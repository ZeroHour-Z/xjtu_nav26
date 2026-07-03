#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/extract_indices.h> // ★ 新增
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

// TF2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class TraversabilityCostmapNode: public rclcpp::Node {
public:
    TraversabilityCostmapNode():
        Node("traversability_costmap_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
        // ---- Parameters ----
        input_topic_ =
            this->declare_parameter<std::string>("input_topic", "/cloud_registered_body");
        RCLCPP_WARN(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        gravity_frame_ = this->declare_parameter<std::string>("gravity_frame", "camera_init");
        // frame_id_ 仅用于日志/兼容性，输出点云的 frame_id 将设为 gravity_frame_
        frame_id_ = this->declare_parameter<std::string>("frame_id", "map");

        resolution_ = this->declare_parameter<double>("resolution", 0.05); // m
        width_m_ = this->declare_parameter<double>("width_m", 20.0); // m
        height_m_ = this->declare_parameter<double>("height_m", 20.0); // m
        origin_x_ = this->declare_parameter<double>("origin_x", -10.0); // m
        origin_y_ = this->declare_parameter<double>("origin_y", -10.0); // m

        z_clip_min_ = this->declare_parameter<double>("z_clip_min", -2.0);
        z_clip_max_ = this->declare_parameter<double>("z_clip_max", 2.0);

        min_points_per_cell_ = this->declare_parameter<int>("min_points_per_cell", 3);
        step_threshold_m_ = this->declare_parameter<double>("step_threshold_m", 0.07);
        step_max_threshold_m_ =
            this->declare_parameter<double>("step_max_threshold_m", 0.30); // 更合理的上限

        // Density-based criterion parameters
        density_min_pts_per_m3_ =
            this->declare_parameter<double>("density_min_pts_per_m3", 30.0); // pts/m^3
        min_points_for_density_ = this->declare_parameter<int>("min_points_for_density", 3);

        width_px_ = static_cast<int>(std::lround(width_m_ / resolution_));
        height_px_ = static_cast<int>(std::lround(height_m_ / resolution_));

        RCLCPP_INFO(
            get_logger(),
            "Grid %dx%d @ %.3fm, origin(%.2f, %.2f), gravity_frame=%s",
            width_px_,
            height_px_,
            resolution_,
            origin_x_,
            origin_y_,
            gravity_frame_.c_str()
        );

        // Publishers
        ground_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/traversability/ground", 10);
        obstacle_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/traversability/obstacles", 10);

        // Subscriber (sensor data QoS)
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&TraversabilityCostmapNode::pcCallback, this, std::placeholders::_1)
        );
    }

private:
    static inline float percentileSorted(const std::vector<float>& a, double p) {
        if (a.empty())
            return std::numeric_limits<float>::quiet_NaN();
        const double idx = p * (static_cast<double>(a.size()) - 1.0);
        const size_t lo = static_cast<size_t>(std::floor(idx));
        const size_t hi = static_cast<size_t>(std::ceil(idx));
        if (hi == lo)
            return a[lo];
        const double w = idx - static_cast<double>(lo);
        return static_cast<float>(a[lo] + (a[hi] - a[lo]) * w);
    }

    // 仅应用旋转，不应用平移：p' = R * p
    bool getRotationOnly(
        const std::string& target_frame,
        const std::string& source_frame,
        tf2::Quaternion& out_q
    ) {
        try {
            auto tf = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            tf2::fromMsg(tf.transform.rotation, out_q);
            out_q.normalize();
            return true;
        } catch (const std::exception& e) {
            RCLCPP_WARN(
                get_logger(),
                "TF lookup failed (%s -> %s): %s",
                source_frame.c_str(),
                target_frame.c_str(),
                e.what()
            );
            return false;
        }
    }

    void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // A) 同时准备两份：1) PCL XYZ（用于计算/索引定位）；2) PCLPointCloud2（用于按索引提取并保持原字段）
        pcl::PointCloud<pcl::PointXYZ> cloud_in_xyz;
        try {
            pcl::fromROSMsg(*msg, cloud_in_xyz);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "fromROSMsg failed: %s", e.what());
            return;
        }
        if (cloud_in_xyz.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Empty point cloud");
            // The type of msg->header is std_msgs::msg::Header, but publishEmptyClouds expects a shared_ptr.
            // Create a shared_ptr from the header to pass to the function.
            publishEmptyClouds(msg->header);
            return;
        }

        pcl::PCLPointCloud2 pcl2_in;
        pcl_conversions::toPCL(*msg, pcl2_in); // 完整保留字段、密度、组织结构等

        // B) 获取仅旋转的 TF
        tf2::Quaternion q_s2g;
        if (!getRotationOnly(gravity_frame_, msg->header.frame_id, q_s2g)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *this->get_clock(),
                2000,
                "Use identity rotation due to missing TF."
            );
            q_s2g.setValue(0, 0, 0, 1);
        }
        tf2::Matrix3x3 R(q_s2g);

        const int width = width_px_;
        const int height = height_px_;

        // C) 将点分配到栅格：只保存 index 和 旋转后的 z
        struct CellPoints {
            std::vector<int> idxs; // 原始索引
            std::vector<float> zs; // 旋转后的 z（用于高度/密度判断）
        };
        std::unordered_map<int, CellPoints> cells;
        cells.reserve(cloud_in_xyz.size() / 8 + 1);

        for (int i = 0; i < static_cast<int>(cloud_in_xyz.size()); ++i) {
            const auto& p = cloud_in_xyz.points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            tf2::Vector3 v(p.x, p.y, p.z);
            tf2::Vector3 vr = R * v;

            // z 高度筛选（在 gravity frame 中）
            if (vr.getZ() < z_clip_min_ || vr.getZ() > z_clip_max_)
                continue;

            const int ix = static_cast<int>(std::floor((vr.getX() - origin_x_) / resolution_));
            const int iy = static_cast<int>(std::floor((vr.getY() - origin_y_) / resolution_));
            if (ix < 0 || ix >= width || iy < 0 || iy >= height)
                continue;

            const int idx = iy * width + ix;
            auto& c = cells[idx];
            c.idxs.push_back(i); // 记录原始点索引
            c.zs.push_back(static_cast<float>(vr.getZ()));
        }

        // D) 逐格判定，并把“索引”分到 ground / obstacle
        std::vector<int> ground_indices;
        std::vector<int> obstacle_indices;
        ground_indices.reserve(cloud_in_xyz.size());
        obstacle_indices.reserve(cloud_in_xyz.size());

        size_t n_obs_cells = 0, n_lethal_cells = 0;
        size_t n_step_only = 0, n_density_only = 0, n_both = 0;

        for (auto& kv: cells) {
            auto& zs = kv.second.zs;
            auto& idxs = kv.second.idxs;
            const int num_points = static_cast<int>(zs.size());

            // per-cell 体积（xy 为分辨率，z 用该格内厚度）
            double cell_volume_m3 = std::numeric_limits<double>::epsilon();
            if (num_points > 0) {
                auto [min_it, max_it] = std::minmax_element(zs.begin(), zs.end());
                const double z_min = static_cast<double>(*min_it);
                const double z_max = static_cast<double>(*max_it);
                const double thickness = std::max(0.0, z_max - z_min);
                cell_volume_m3 = resolution_ * resolution_
                    * std::max(thickness, std::numeric_limits<double>::epsilon());
            }

            // 密度判据
            const double density_pts_per_m3 = (cell_volume_m3 > 0.0)
                ? static_cast<double>(num_points) / cell_volume_m3
                : std::numeric_limits<double>::infinity();
            const bool density_ok = (num_points >= min_points_for_density_)
                && (density_pts_per_m3 > density_min_pts_per_m3_);

            bool step_hazard = false;
            if (num_points >= min_points_per_cell_) {
                std::sort(zs.begin(), zs.end());
                const float q10 = percentileSorted(zs, 0.05f);
                const float q90 = percentileSorted(zs, 0.95f);
                const float dh = q90 - q10;
                // if ((dh >= static_cast<float>(step_threshold_m_)) &&
                //     (dh <= static_cast<float>(step_max_threshold_m_))) {
                //   step_hazard = true;
                // }
                if (dh >= static_cast<float>(step_threshold_m_)) {
                    step_hazard = true;
                }
            }

            // const bool density_hazard = density_ok;

            bool is_obstacle = false;
            if (step_hazard && density_ok) {
                ++n_both;
                ++n_lethal_cells;
                is_obstacle = true;
            } else if (step_hazard) {
                ++n_step_only;
                ++n_lethal_cells;
                is_obstacle = false;
            } else if (density_ok) {
                ++n_density_only;
                ++n_lethal_cells;
                is_obstacle = false;
            }

            if (is_obstacle) {
                obstacle_indices.insert(obstacle_indices.end(), idxs.begin(), idxs.end());
            } else {
                ground_indices.insert(ground_indices.end(), idxs.begin(), idxs.end());
            }

            ++n_obs_cells;
        }

        // E) 用索引在“原始点云”上提取并发布（保持原始 header）
        sensor_msgs::msg::PointCloud2 ground_msg, obstacle_msg;
        ground_msg.header = msg->header; // ★ 保持原始 header
        obstacle_msg.header = msg->header; // ★ 保持原始 header

        // 转成 PCL 索引类型
        pcl::PointIndices::Ptr ground_pi(new pcl::PointIndices());
        pcl::PointIndices::Ptr obstacle_pi(new pcl::PointIndices());
        ground_pi->indices = std::move(ground_indices);
        obstacle_pi->indices = std::move(obstacle_indices);

        // 提取
        pcl::ExtractIndices<pcl::PCLPointCloud2> ex;
        ex.setInputCloud(pcl::PCLPointCloud2ConstPtr(new pcl::PCLPointCloud2(pcl2_in)));

        pcl::PCLPointCloud2 pcl2_ground, pcl2_obstacle;

        ex.setIndices(ground_pi);
        ex.setNegative(false);
        ex.filter(pcl2_ground);

        ex.setIndices(obstacle_pi);
        ex.setNegative(false);
        ex.filter(pcl2_obstacle);

        // 回到 ROS2 消息
        pcl_conversions::fromPCL(pcl2_ground, ground_msg);
        pcl_conversions::fromPCL(pcl2_obstacle, obstacle_msg);

        // header 可能被转换覆盖，再次确保保持原始 header
        ground_msg.header = msg->header;
        obstacle_msg.header = msg->header;

        ground_pub_->publish(ground_msg);
        obstacle_pub_->publish(obstacle_msg);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *this->get_clock(),
            5000,
            "cells observed=%zu, lethal=%zu (step=%zu, density=%zu, both=%zu) | ground idx=%zu, obstacle idx=%zu",
            n_obs_cells,
            n_lethal_cells,
            n_step_only,
            n_density_only,
            n_both,
            static_cast<size_t>(ground_pi->indices.size()),
            static_cast<size_t>(obstacle_pi->indices.size())
        );
    }

    void publishEmptyClouds(const std_msgs::msg::Header header) {
        sensor_msgs::msg::PointCloud2 empty;
        empty.header = header; // ★ 保持原始 header
        ground_pub_->publish(empty);
        obstacle_pub_->publish(empty);
    }

    // Params
    std::string input_topic_;
    std::string gravity_frame_;
    std::string frame_id_; // 保留参数以兼容，但不再作为输出 frame
    double resolution_;
    double width_m_, height_m_;
    double origin_x_, origin_y_;
    double z_clip_min_, z_clip_max_;
    int min_points_per_cell_;
    double step_threshold_m_;
    double step_max_threshold_m_;
    double density_min_pts_per_m3_;
    int min_points_for_density_;

    // Derived
    int width_px_;
    int height_px_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TraversabilityCostmapNode>());
    rclcpp::shutdown();
    return 0;
}
