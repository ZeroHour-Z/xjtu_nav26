/**
 * @file region_detector_node.cpp
 * @brief 区域检测节点 - 检测机器人是否在特殊区域（如颠簸路段）
 * 
 * 功能：
 * 1. 定义多个特殊区域（多边形）
 * 2. 订阅路径和TF，检测机器人是否即将进入特殊区域
 * 3. 发布区域状态和期望航向角
 * 4. 在RViz中可视化区域
 */

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <string>
#include <vector>

// 区域类型定义（与 rm_communication/protocol.h 保持一致）
enum RegionType {
    REGION_LOCAL = 0,
    REGION_FLAT,
    REGION_HOLE,
    REGION_SLOPE,
    REGION_STEP,
    REGION_FLUCTUATE,
};

enum TargetRegion {
    TARGET_SELF_BASE_AREA = 0,
    TARGET_CENTRAL_HIGHLAND_AREA,
    TARGET_ENEMY_BASE_AREA,
};

// 区域配置结构
struct RegionConfig {
    std::string name;
    RegionType type;
    std::vector<std::pair<double, double>> polygon;
    double yaw_desired; // 进入该区域时的期望航向角
};

class RegionDetectorNode: public rclcpp::Node {
public:
    RegionDetectorNode():
        Node("region_detector_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
        // 声明参数
        this->declare_parameter<double>("lookahead_distance", 0.5);
        this->declare_parameter<double>("publish_rate", 20.0);
        this->declare_parameter<std::string>("plan_topic", "/plan");
        this->declare_parameter<std::string>("chase_goal_topic", "/chase_goal_pose");
        this->declare_parameter<double>("chase_plan_goal_tolerance", 0.75);

        // 颠簸区域参数（可以定义多个区域）
        // 格式：[x1, y1, x2, y2, x3, y3, ...]
        this->declare_parameter<std::vector<double>>("fluctuate_region_1", std::vector<double> {});
        this->declare_parameter<double>("fluctuate_region_1_yaw", 0.0);

        this->declare_parameter<std::vector<double>>("fluctuate_region_2", std::vector<double> {});
        this->declare_parameter<double>("fluctuate_region_2_yaw", 0.0);

        this->declare_parameter<std::vector<double>>("fluctuate_region_3", std::vector<double> {});
        this->declare_parameter<double>("fluctuate_region_3_yaw", 0.0);

        this->declare_parameter<std::vector<double>>("fluctuate_region_4", std::vector<double> {});
        this->declare_parameter<double>("fluctuate_region_4_yaw", 0.0);

        // 地图三区域多边形参数（map坐标系）
        this->declare_parameter<std::vector<double>>("self_base_region", std::vector<double> {});
        this->declare_parameter<std::vector<double>>(
            "central_highland_region",
            std::vector<double> {}
        );
        this->declare_parameter<std::vector<double>>("enemy_base_region", std::vector<double> {});
        this->declare_parameter<std::string>("battle_region_frame", "map");

        // 加载区域配置
        loadRegions();
        loadBattleRegionPolygons();
        this->get_parameter("battle_region_frame", battle_region_frame_);

        // 发布器
        region_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/region_type", 10);
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("/region_yaw_desired", 10);
        path_through_fluctuate_pub_ =
            this->create_publisher<std_msgs::msg::Bool>("/path_through_fluctuate_region", 10);
        chase_path_through_fluctuate_pub_ =
            this->create_publisher<std_msgs::msg::Bool>("/chase_path_through_fluctuate_region", 10);
        target_region_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/target_region", 10);
        self_region_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/self_region", 10);
        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/region_markers", 10);

        // 订阅路径与通信包
        this->get_parameter("plan_topic", plan_topic_);
        this->get_parameter("chase_goal_topic", chase_goal_topic_);
        this->get_parameter("chase_plan_goal_tolerance", chase_plan_goal_tolerance_);

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            plan_topic_,
            10,
            std::bind(&RegionDetectorNode::onPath, this, std::placeholders::_1)
        );

        if (!chase_goal_topic_.empty()) {
            chase_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                chase_goal_topic_,
                10,
                std::bind(&RegionDetectorNode::onChaseGoal, this, std::placeholders::_1)
            );
        }

        rx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/rx_packet",
            10,
            std::bind(&RegionDetectorNode::onRxPacket, this, std::placeholders::_1)
        );

        // 定时器
        double rate = 20.0;
        this->get_parameter("publish_rate", rate);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&RegionDetectorNode::onTimer, this)
        );

        marker_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RegionDetectorNode::publishMarkers, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Region detector started with %zu regions",
            regions_.size()
        );
    }

    bool loadPolygonParam(
        const std::string& param_name,
        std::vector<std::pair<double, double>>& polygon_out
    ) {
        std::vector<double> data;
        this->get_parameter(param_name, data);
        polygon_out.clear();
        if (data.size() < 6 || data.size() % 2 != 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid polygon param '%s': size=%zu, need even size >= 6",
                param_name.c_str(),
                data.size()
            );
            return false;
        }

        for (size_t i = 0; i < data.size(); i += 2) {
            polygon_out.push_back({ data[i], data[i + 1] });
        }
        return true;
    }

    void loadBattleRegionPolygons() {
        bool ok_self = loadPolygonParam("self_base_region", self_base_polygon_);
        bool ok_central = loadPolygonParam("central_highland_region", central_polygon_);
        bool ok_enemy = loadPolygonParam("enemy_base_region", enemy_base_polygon_);

        if (ok_self && ok_central && ok_enemy) {
            RCLCPP_INFO(
                this->get_logger(),
                "Loaded battle polygons: self=%zu, central=%zu, enemy=%zu vertices",
                self_base_polygon_.size(),
                central_polygon_.size(),
                enemy_base_polygon_.size()
            );
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Battle region polygons are incomplete, classification may fallback to central_highland_area"
            );
        }
    }

private:
    // 加载区域配置
    void loadRegions() {
        regions_.clear();

        // 加载颠簸区域1
        loadSingleRegion(
            "fluctuate_region_1",
            "fluctuate_region_1_yaw",
            "Fluctuate1",
            REGION_FLUCTUATE
        );
        // 加载颠簸区域2
        loadSingleRegion(
            "fluctuate_region_2",
            "fluctuate_region_2_yaw",
            "Fluctuate2",
            REGION_FLUCTUATE
        );
        // 加载颠簸区域3
        loadSingleRegion(
            "fluctuate_region_3",
            "fluctuate_region_3_yaw",
            "Fluctuate3",
            REGION_FLUCTUATE
        );
        // 加载颠簸区域4
        loadSingleRegion(
            "fluctuate_region_4",
            "fluctuate_region_4_yaw",
            "Fluctuate4",
            REGION_FLUCTUATE
        );
    }

    void loadSingleRegion(
        const std::string& polygon_param,
        const std::string& yaw_param,
        const std::string& name,
        RegionType type
    ) {
        std::vector<double> polygon_data;
        this->get_parameter(polygon_param, polygon_data);

        if (polygon_data.size() >= 6 && polygon_data.size() % 2 == 0) {
            RegionConfig region;
            region.name = name;
            region.type = type;
            this->get_parameter(yaw_param, region.yaw_desired);

            for (size_t i = 0; i < polygon_data.size(); i += 2) {
                region.polygon.push_back({ polygon_data[i], polygon_data[i + 1] });
            }

            regions_.push_back(region);
            RCLCPP_INFO(
                this->get_logger(),
                "Loaded region '%s' with %zu vertices, yaw=%.2f",
                name.c_str(),
                region.polygon.size(),
                region.yaw_desired
            );
        }
    }

    // 判断点是否在多边形内 (射线法)
    bool isPointInPolygon(
        double x,
        double y,
        const std::vector<std::pair<double, double>>& polygon
    ) const {
        if (polygon.size() < 3)
            return false;

        bool inside = false;
        size_t n = polygon.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            double xi = polygon[i].first, yi = polygon[i].second;
            double xj = polygon[j].first, yj = polygon[j].second;

            if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
                inside = !inside;
            }
        }
        return inside;
    }

    // 计算点到多边形边界的最近距离
    double distanceToPolygon(
        double x,
        double y,
        const std::vector<std::pair<double, double>>& polygon
    ) const {
        if (polygon.size() < 3)
            return std::numeric_limits<double>::max();

        double min_dist = std::numeric_limits<double>::max();
        size_t n = polygon.size();

        for (size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            double x1 = polygon[i].first, y1 = polygon[i].second;
            double x2 = polygon[j].first, y2 = polygon[j].second;

            double dx = x2 - x1;
            double dy = y2 - y1;
            double t = std::max(
                0.0,
                std::min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy + 1e-9))
            );
            double closest_x = x1 + t * dx;
            double closest_y = y1 + t * dy;
            double dist = std::hypot(x - closest_x, y - closest_y);
            min_dist = std::min(min_dist, dist);
        }
        return min_dist;
    }

    bool computePathYawInRegion(size_t region_index, double robot_x, double robot_y, double& yaw)
        const {
        if (!has_path_ || region_index >= regions_.size() || last_path_.poses.size() < 2) {
            return false;
        }

        const auto& polygon = regions_[region_index].polygon;
        size_t closest_inside_idx = 0;
        double closest_dist_sq = std::numeric_limits<double>::max();
        bool found_inside = false;

        for (size_t i = 0; i < last_path_.poses.size(); ++i) {
            const auto& p = last_path_.poses[i].pose.position;
            if (!isPointInPolygon(p.x, p.y, polygon)) {
                continue;
            }

            const double dx = p.x - robot_x;
            const double dy = p.y - robot_y;
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq < closest_dist_sq) {
                closest_dist_sq = dist_sq;
                closest_inside_idx = i;
                found_inside = true;
            }
        }

        if (!found_inside) {
            return false;
        }

        auto segmentYaw = [&](size_t from, size_t to, double& out_yaw) {
            const auto& a = last_path_.poses[from].pose.position;
            const auto& b = last_path_.poses[to].pose.position;
            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            if (std::hypot(dx, dy) < 1e-4) {
                return false;
            }
            out_yaw = std::atan2(dy, dx);
            return true;
        };

        for (size_t i = closest_inside_idx; i + 1 < last_path_.poses.size(); ++i) {
            const auto& p = last_path_.poses[i + 1].pose.position;
            if (isPointInPolygon(p.x, p.y, polygon) && segmentYaw(i, i + 1, yaw)) {
                return true;
            }
        }

        for (size_t i = closest_inside_idx; i > 0; --i) {
            const auto& p = last_path_.poses[i - 1].pose.position;
            if (isPointInPolygon(p.x, p.y, polygon) && segmentYaw(i - 1, i, yaw)) {
                return true;
            }
        }

        return false;
    }

    bool updatePathRegions(const nav_msgs::msg::Path& path, std::set<size_t>& path_regions) const {
        bool path_through_fluctuate = false;
        path_regions.clear();
        for (size_t i = 0; i < regions_.size(); ++i) {
            for (const auto& pose: path.poses) {
                if (isPointInPolygon(
                        pose.pose.position.x,
                        pose.pose.position.y,
                        regions_[i].polygon
                    ))
                {
                    path_regions.insert(i);
                    if (regions_[i].type == REGION_FLUCTUATE) {
                        path_through_fluctuate = true;
                    }
                    break;
                }
            }
        }
        return path_through_fluctuate;
    }

    bool pathMatchesLatestChaseGoal(const nav_msgs::msg::Path& path) const {
        if (!has_chase_goal_ || path.poses.empty()) {
            return false;
        }

        const auto& end = path.poses.back().pose.position;
        const auto& goal = latest_chase_goal_.pose.position;
        const double dist = std::hypot(end.x - goal.x, end.y - goal.y);
        return dist <= chase_plan_goal_tolerance_;
    }

    // 路径回调
    void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
        last_path_ = *msg;
        has_path_ = true;

        // 普通路径状态仍然跟随 Nav2 的 /plan，用于区域预瞄和通用调试。
        path_through_fluctuate_region_ = updatePathRegions(*msg, path_regions_);

        publishPathThroughFluctuate();

        if (pathMatchesLatestChaseGoal(*msg)) {
            chase_path_through_fluctuate_region_ =
                updatePathRegions(*msg, chase_path_regions_);
            publishChasePathThroughFluctuate();
        }

        if (!path_regions_.empty()) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Path goes through %zu special region(s)",
                path_regions_.size()
            );
        }
    }

    void onChaseGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latest_chase_goal_ = *msg;
        has_chase_goal_ = true;
        chase_path_regions_.clear();
        chase_path_through_fluctuate_region_ = false;
        publishChasePathThroughFluctuate();
    }

    void publishPathThroughFluctuate() {
        std_msgs::msg::Bool msg;
        msg.data = path_through_fluctuate_region_;
        path_through_fluctuate_pub_->publish(msg);
    }

    void publishChasePathThroughFluctuate() {
        std_msgs::msg::Bool msg;
        msg.data = chase_path_through_fluctuate_region_;
        chase_path_through_fluctuate_pub_->publish(msg);
    }

    void onRxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        constexpr size_t kRxPacketSize = 64;
        constexpr uint8_t kHeader = 0x72;
        constexpr uint8_t kTailExpected = 0x21;
        // navCommand_t(打包后)当前布局：
        // 0:frame_header, 1:color, 2:eSentryState, 3-4:hp, 5-6:bullet,
        // 7-10:target_x, 11-14:target_y, 15:is_revive,
        // 16-19:enemy_x, 20-23:enemy_y
        constexpr size_t kEnemyXOffset = 16;
        constexpr size_t kEnemyYOffset = 20;
        constexpr size_t kTailOffset = 63;

        if (msg->data.size() != kRxPacketSize || msg->data.empty() || msg->data[0] != kHeader) {
            return;
        }

        if (msg->data[kTailOffset] != kTailExpected) {
            return;
        }

        float enemy_x = 0.0f;
        float enemy_y = 0.0f;
        std::memcpy(&enemy_x, msg->data.data() + kEnemyXOffset, sizeof(float));
        std::memcpy(&enemy_y, msg->data.data() + kEnemyYOffset, sizeof(float));

        latest_enemy_x_ = enemy_x;
        latest_enemy_y_ = enemy_y;
        has_enemy_pose_ = true;
    }

    uint8_t classifyBattleRegion(float x, float y) const {
        if (isPointInPolygon(x, y, self_base_polygon_)) {
            return static_cast<uint8_t>(TargetRegion::TARGET_SELF_BASE_AREA);
        }
        if (isPointInPolygon(x, y, central_polygon_)) {
            return static_cast<uint8_t>(TargetRegion::TARGET_CENTRAL_HIGHLAND_AREA);
        }
        if (isPointInPolygon(x, y, enemy_base_polygon_)) {
            return static_cast<uint8_t>(TargetRegion::TARGET_ENEMY_BASE_AREA);
        }
        return static_cast<uint8_t>(TargetRegion::TARGET_CENTRAL_HIGHLAND_AREA);
    }

    bool transformEnemyOdomToMap(
        double enemy_odom_x,
        double enemy_odom_y,
        double& map_x,
        double& map_y
    ) {
        geometry_msgs::msg::PoseStamped enemy_in_odom;
        enemy_in_odom.header.stamp = this->now();
        enemy_in_odom.header.frame_id = "odom";
        enemy_in_odom.pose.position.x = enemy_odom_x;
        enemy_in_odom.pose.position.y = enemy_odom_y;
        enemy_in_odom.pose.position.z = 0.0;
        enemy_in_odom.pose.orientation.w = 1.0;

        try {
            auto enemy_in_map =
                tf_buffer_.transform(enemy_in_odom, "map", tf2::durationFromSec(0.05));
            map_x = enemy_in_map.pose.position.x;
            map_y = enemy_in_map.pose.position.y;
            return true;
        } catch (const tf2::TransformException&) {
            return false;
        }
    }

    // 获取当前机器人在 map 坐标系的位置
    bool getRobotPose(double& x, double& y, double& yaw) {
        try {
            auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            x = tf.transform.translation.x;
            y = tf.transform.translation.y;

            tf2::Quaternion q;
            q.setX(tf.transform.rotation.x);
            q.setY(tf.transform.rotation.y);
            q.setZ(tf.transform.rotation.z);
            q.setW(tf.transform.rotation.w);
            double roll, pitch;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            return true;
        } catch (const tf2::TransformException& ex) {
            return false;
        }
    }

    // 定时器回调
    void onTimer() {
        double robot_x, robot_y, robot_yaw;
        if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
            return;
        }
        latest_self_map_x_ = static_cast<float>(robot_x);
        latest_self_map_y_ = static_cast<float>(robot_y);
        has_self_map_pose_ = true;

        double lookahead_dist = 0.5;
        this->get_parameter("lookahead_distance", lookahead_dist);

        // 检测当前区域
        RegionType current_region = REGION_FLAT;
        double yaw_desired = 0.0;
        bool in_special_region = false;

        for (size_t i = 0; i < regions_.size(); ++i) {
            const auto& region = regions_[i];

            // 检查是否在区域内
            if (isPointInPolygon(robot_x, robot_y, region.polygon)) {
                current_region = region.type;
                yaw_desired = region.yaw_desired;
                if (region.type == REGION_FLUCTUATE) {
                    (void)computePathYawInRegion(i, robot_x, robot_y, yaw_desired);
                }
                in_special_region = true;

                // RCLCPP_INFO_THROTTLE(
                //     this->get_logger(),
                //     *this->get_clock(),
                //     500,
                //     "Robot IN region '%s'! pos=(%.2f, %.2f)",
                //     region.name.c_str(),
                //     robot_x,
                //     robot_y
                // );
                break;
            }

            // 检查是否即将进入区域（路径经过且距离近）
            if (path_regions_.count(i) > 0) {
                double dist = distanceToPolygon(robot_x, robot_y, region.polygon);
                if (dist < lookahead_dist) {
                    current_region = region.type;
                    yaw_desired = region.yaw_desired;
                    if (region.type == REGION_FLUCTUATE) {
                        (void)computePathYawInRegion(i, robot_x, robot_y, yaw_desired);
                    }
                    in_special_region = true;

                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        500,
                        "Robot APPROACHING region '%s'! dist=%.2f",
                        region.name.c_str(),
                        dist
                    );
                    break;
                }
            }
        }

        // 发布区域类型
        std_msgs::msg::UInt8 region_msg;
        region_msg.data = static_cast<uint8_t>(current_region);
        region_pub_->publish(region_msg);

        // 发布期望航向角
        std_msgs::msg::Float32 yaw_msg;
        yaw_msg.data = static_cast<float>(yaw_desired);
        yaw_pub_->publish(yaw_msg);
        publishPathThroughFluctuate();
        publishChasePathThroughFluctuate();

        if (has_enemy_pose_) {
            double enemy_map_x = 0.0;
            double enemy_map_y = 0.0;
            if (transformEnemyOdomToMap(latest_enemy_x_, latest_enemy_y_, enemy_map_x, enemy_map_y))
            {
                std_msgs::msg::UInt8 target_region_msg;
                target_region_msg.data = classifyBattleRegion(
                    static_cast<float>(enemy_map_x),
                    static_cast<float>(enemy_map_y)
                );
                target_region_pub_->publish(target_region_msg);
                // RCLCPP_INFO_THROTTLE(
                //     this->get_logger(),
                //     *this->get_clock(),
                //     500,
                //     "Battle region classify (enemy): odom(%.2f, %.2f) -> map(%.2f, %.2f) => target_region=%u",
                //     latest_enemy_x_,
                //     latest_enemy_y_,
                //     enemy_map_x,
                //     enemy_map_y,
                //     static_cast<unsigned int>(target_region_msg.data)
                // );
            } else {
                // RCLCPP_WARN_THROTTLE(
                //     this->get_logger(),
                //     *this->get_clock(),
                //     1000,
                //     "Battle region classify (enemy): failed odom->map transform for odom(%.2f, %.2f)",
                //     latest_enemy_x_,
                //     latest_enemy_y_
                // );
            }
        }

        if (has_self_map_pose_) {
            std_msgs::msg::UInt8 self_region_msg;
            self_region_msg.data = classifyBattleRegion(latest_self_map_x_, latest_self_map_y_);
            self_region_pub_->publish(self_region_msg);
            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     500,
            //     "Battle region classify (self): map(%.2f, %.2f) => self_region=%u",
            //     latest_self_map_x_,
            //     latest_self_map_y_,
            //     static_cast<unsigned int>(self_region_msg.data)
            // );
        }

        // 更新状态用于可视化
        current_region_ = current_region;
        in_special_region_ = in_special_region;
    }

    // 发布可视化 Marker
    void publishMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        for (size_t i = 0; i < regions_.size(); ++i) {
            const auto& region = regions_[i];

            // 边界线
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = battle_region_frame_;
            line_marker.header.stamp = this->now();
            line_marker.ns = "region_boundary";
            line_marker.id = id++;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.05;

            // 根据状态设置颜色
            bool is_active = (in_special_region_ && path_regions_.count(i) > 0);
            if (is_active) {
                line_marker.color.r = 1.0f;
                line_marker.color.g = 0.0f;
                line_marker.color.b = 0.0f;
                line_marker.color.a = 1.0f;
            } else if (path_regions_.count(i) > 0) {
                line_marker.color.r = 1.0f;
                line_marker.color.g = 0.5f;
                line_marker.color.b = 0.0f;
                line_marker.color.a = 0.8f;
            } else {
                line_marker.color.r = 1.0f;
                line_marker.color.g = 1.0f;
                line_marker.color.b = 0.0f;
                line_marker.color.a = 0.6f;
            }

            for (const auto& pt: region.polygon) {
                geometry_msgs::msg::Point p;
                p.x = pt.first;
                p.y = pt.second;
                p.z = 0.05;
                line_marker.points.push_back(p);
            }
            // 闭合
            if (!region.polygon.empty()) {
                geometry_msgs::msg::Point p;
                p.x = region.polygon.front().first;
                p.y = region.polygon.front().second;
                p.z = 0.05;
                line_marker.points.push_back(p);
            }

            line_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            marker_array.markers.push_back(line_marker);

            // 填充区域
            visualization_msgs::msg::Marker fill_marker;
            fill_marker.header.frame_id = battle_region_frame_;
            fill_marker.header.stamp = this->now();
            fill_marker.ns = "region_fill";
            fill_marker.id = id++;
            fill_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            fill_marker.action = visualization_msgs::msg::Marker::ADD;
            fill_marker.scale.x = 1.0;
            fill_marker.scale.y = 1.0;
            fill_marker.scale.z = 1.0;

            if (is_active) {
                fill_marker.color.r = 1.0f;
                fill_marker.color.g = 0.0f;
                fill_marker.color.b = 0.0f;
                fill_marker.color.a = 0.3f;
            } else {
                fill_marker.color.r = 1.0f;
                fill_marker.color.g = 1.0f;
                fill_marker.color.b = 0.0f;
                fill_marker.color.a = 0.15f;
            }

            // 扇形分解多边形
            if (region.polygon.size() >= 3) {
                for (size_t j = 1; j < region.polygon.size() - 1; ++j) {
                    geometry_msgs::msg::Point p0, p1, p2;
                    p0.x = region.polygon[0].first;
                    p0.y = region.polygon[0].second;
                    p0.z = 0.02;

                    p1.x = region.polygon[j].first;
                    p1.y = region.polygon[j].second;
                    p1.z = 0.02;

                    p2.x = region.polygon[j + 1].first;
                    p2.y = region.polygon[j + 1].second;
                    p2.z = 0.02;

                    fill_marker.points.push_back(p0);
                    fill_marker.points.push_back(p1);
                    fill_marker.points.push_back(p2);
                }
            }

            fill_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            marker_array.markers.push_back(fill_marker);

            // 区域名称标签
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = battle_region_frame_;
            text_marker.header.stamp = this->now();
            text_marker.ns = "region_label";
            text_marker.id = id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;

            // 计算多边形中心
            double cx = 0, cy = 0;
            for (const auto& pt: region.polygon) {
                cx += pt.first;
                cy += pt.second;
            }
            cx /= region.polygon.size();
            cy /= region.polygon.size();

            text_marker.pose.position.x = cx;
            text_marker.pose.position.y = cy;
            text_marker.pose.position.z = 0.5;
            text_marker.scale.z = 0.3;
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0f;
            text_marker.text = region.name;
            text_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            marker_array.markers.push_back(text_marker);
        }

        auto add_battle_region_markers = [&](const std::vector<std::pair<double, double>>& polygon,
                                             const std::string& name,
                                             float r,
                                             float g,
                                             float b) {
            if (polygon.size() < 3) {
                return;
            }

            bool self_inside = has_self_map_pose_
                && isPointInPolygon(latest_self_map_x_, latest_self_map_y_, polygon);
            float line_alpha = self_inside ? 1.0f : 0.8f;
            float fill_alpha = self_inside ? 0.35f : 0.18f;

            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = "map";
            line_marker.header.stamp = this->now();
            line_marker.ns = "battle_region_boundary";
            line_marker.id = id++;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.06;
            line_marker.color.r = r;
            line_marker.color.g = g;
            line_marker.color.b = b;
            line_marker.color.a = line_alpha;

            for (const auto& pt: polygon) {
                geometry_msgs::msg::Point p;
                p.x = pt.first;
                p.y = pt.second;
                p.z = 0.08;
                line_marker.points.push_back(p);
            }
            geometry_msgs::msg::Point p_close;
            p_close.x = polygon.front().first;
            p_close.y = polygon.front().second;
            p_close.z = 0.08;
            line_marker.points.push_back(p_close);
            line_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            marker_array.markers.push_back(line_marker);

            visualization_msgs::msg::Marker fill_marker;
            fill_marker.header.frame_id = "map";
            fill_marker.header.stamp = this->now();
            fill_marker.ns = "battle_region_fill";
            fill_marker.id = id++;
            fill_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            fill_marker.action = visualization_msgs::msg::Marker::ADD;
            fill_marker.scale.x = 1.0;
            fill_marker.scale.y = 1.0;
            fill_marker.scale.z = 1.0;
            fill_marker.color.r = r;
            fill_marker.color.g = g;
            fill_marker.color.b = b;
            fill_marker.color.a = fill_alpha;

            for (size_t j = 1; j < polygon.size() - 1; ++j) {
                geometry_msgs::msg::Point p0, p1, p2;
                p0.x = polygon[0].first;
                p0.y = polygon[0].second;
                p0.z = 0.04;
                p1.x = polygon[j].first;
                p1.y = polygon[j].second;
                p1.z = 0.04;
                p2.x = polygon[j + 1].first;
                p2.y = polygon[j + 1].second;
                p2.z = 0.04;
                fill_marker.points.push_back(p0);
                fill_marker.points.push_back(p1);
                fill_marker.points.push_back(p2);
            }
            fill_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            marker_array.markers.push_back(fill_marker);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = this->now();
            text_marker.ns = "battle_region_label";
            text_marker.id = id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;

            double cx = 0.0;
            double cy = 0.0;
            for (const auto& pt: polygon) {
                cx += pt.first;
                cy += pt.second;
            }
            cx /= static_cast<double>(polygon.size());
            cy /= static_cast<double>(polygon.size());

            text_marker.pose.position.x = cx;
            text_marker.pose.position.y = cy;
            text_marker.pose.position.z = 0.8;
            text_marker.scale.z = 0.32;
            text_marker.color.r = r;
            text_marker.color.g = g;
            text_marker.color.b = b;
            text_marker.color.a = 1.0f;
            text_marker.text = name;
            text_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            marker_array.markers.push_back(text_marker);
        };

        add_battle_region_markers(self_base_polygon_, "SelfBase", 0.2f, 0.4f, 1.0f);
        add_battle_region_markers(central_polygon_, "Central", 0.2f, 1.0f, 0.2f);
        add_battle_region_markers(enemy_base_polygon_, "EnemyBase", 1.0f, 0.25f, 0.25f);

        marker_pub_->publish(marker_array);
    }

    // 成员变量
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr region_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_through_fluctuate_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr chase_path_through_fluctuate_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr target_region_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr self_region_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr chase_goal_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr marker_timer_;

    std::vector<RegionConfig> regions_;
    nav_msgs::msg::Path last_path_;
    bool has_path_ { false };
    std::set<size_t> path_regions_; // 路径经过的区域索引
    bool path_through_fluctuate_region_ { false };
    std::set<size_t> chase_path_regions_;
    bool chase_path_through_fluctuate_region_ { false };
    geometry_msgs::msg::PoseStamped latest_chase_goal_;
    bool has_chase_goal_ { false };
    std::string plan_topic_ { "/plan" };
    std::string chase_goal_topic_ { "/chase_goal_pose" };
    double chase_plan_goal_tolerance_ { 0.75 };

    std::vector<std::pair<double, double>> self_base_polygon_;
    std::vector<std::pair<double, double>> central_polygon_;
    std::vector<std::pair<double, double>> enemy_base_polygon_;
    std::string battle_region_frame_ { "map" };

    float latest_enemy_x_ { 0.0f };
    float latest_enemy_y_ { 0.0f };
    float latest_self_map_x_ { 0.0f };
    float latest_self_map_y_ { 0.0f };
    bool has_enemy_pose_ { false };
    bool has_self_map_pose_ { false };

    RegionType current_region_ { REGION_FLAT };
    bool in_special_region_ { false };
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RegionDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
