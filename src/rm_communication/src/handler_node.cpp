/**
 * @file handler_node.cpp
 * @brief 通信处理节点 - 负责与电控的通信协议转换
 * 
 * 功能：
 * 1. 接收电控数据并解析
 * 2. 发送导航信息给电控
 * 3. 订阅 region_detector 发布的区域状态
 * 4. 根据电控状态设置行为树参数（chase, hp, ammo）
 * 5. 发布追击点供行为树使用
 */

#include "protocol.h"
#include "rm_communication/packet_utils.hpp"

#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

class HandlerNode: public rclcpp::Node {
public:
    HandlerNode(): Node("handler_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // 声明追击相关参数
        this->declare_parameter<double>("chase_min_distance", 0.25); // 自身与敌人的最小追击距离
        this->declare_parameter<double>("stop_distance_threshold", 0.05); // 判定停车指令阈值
        this->declare_parameter<std::string>("chase_topic", "/chase_point");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<bool>("enable_chase", true); // 是否启用追击功能
        this->declare_parameter<std::string>("cmd_vel_frame", "map");
        this->declare_parameter<std::string>("angular_z_mode", "yaw_angle");
        this->declare_parameter<double>("yaw_rate_preview_time", 0.15);
        this->declare_parameter<double>("cmd_vel_predict_time", 0.08);
        this->declare_parameter<double>("wz_filter_tau", 0.05);
        this->declare_parameter<bool>("smooth_world_velocity", true);
        this->declare_parameter<double>("world_velocity_filter_tau", 0.12);
        this->declare_parameter<double>("world_velocity_accel_limit", 1.2);
        this->declare_parameter<std::string>("narrow_passage_topic", "/narrow_passage");
        this->declare_parameter<double>("chassis_trapped_radius", 1.0);
        this->declare_parameter<double>("chassis_trapped_timeout", 30.0);
        this->declare_parameter<double>("chassis_trapped_goal_tolerance", 0.30);
        this->declare_parameter<double>("motion_disallowed_clear_timeout", 5.0);
        this->declare_parameter<int>("tx_log_period_ms", 1000);

        chase_min_distance_ = this->get_parameter("chase_min_distance").as_double();
        stop_distance_threshold_ = this->get_parameter("stop_distance_threshold").as_double();
        chase_topic_ = this->get_parameter("chase_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        enable_chase_ = this->get_parameter("enable_chase").as_bool();
        cmd_vel_frame_ = this->get_parameter("cmd_vel_frame").as_string();
        angular_z_mode_ = this->get_parameter("angular_z_mode").as_string();
        yaw_rate_preview_time_ = this->get_parameter("yaw_rate_preview_time").as_double();
        cmd_vel_predict_time_ = this->get_parameter("cmd_vel_predict_time").as_double();
        wz_filter_tau_ = this->get_parameter("wz_filter_tau").as_double();
        smooth_world_velocity_ = this->get_parameter("smooth_world_velocity").as_bool();
        world_velocity_filter_tau_ = this->get_parameter("world_velocity_filter_tau").as_double();
        world_velocity_accel_limit_ =
            this->get_parameter("world_velocity_accel_limit").as_double();
        chassis_trapped_radius_ = this->get_parameter("chassis_trapped_radius").as_double();
        chassis_trapped_timeout_ = this->get_parameter("chassis_trapped_timeout").as_double();
        chassis_trapped_goal_tolerance_ =
            this->get_parameter("chassis_trapped_goal_tolerance").as_double();
        motion_disallowed_clear_timeout_ =
            this->get_parameter("motion_disallowed_clear_timeout").as_double();
        tx_log_period_ms_ = this->get_parameter("tx_log_period_ms").as_int();

        if (cmd_vel_frame_ != "base_link" && cmd_vel_frame_ != "map") {
            throw std::runtime_error("cmd_vel_frame must be 'base_link' or 'map'");
        }
        if (angular_z_mode_ != "yaw_rate" && angular_z_mode_ != "yaw_angle") {
            throw std::runtime_error("angular_z_mode must be 'yaw_rate' or 'yaw_angle'");
        }

        // 发布器
        patrol_group_pub_ = this->create_publisher<std_msgs::msg::String>("/patrol_group", 10);
        tx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/rm_comm/tx_packet", 10);

        // 追击目标点发布器（供行为树使用）
        chase_point_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(chase_topic_, 10);

        // 底盘卡住标志发布器（供决策层行为树读取，用于切换备选目标点）
        chassis_trapped_pub_ =
            this->create_publisher<std_msgs::msg::Bool>("/chassis_trapped", 10);

        RCLCPP_INFO(this->get_logger(), "Chase mode enabled: %s", enable_chase_ ? "true" : "false");

        // 电控数据订阅
        rx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/rx_packet",
            10,
            std::bind(&HandlerNode::onRxPacket, this, std::placeholders::_1)
        );

        // 速度命令订阅
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&HandlerNode::onCmdVel, this, std::placeholders::_1)
        );

        // 目标点订阅
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            rclcpp::QoS(10).best_effort(),
            std::bind(&HandlerNode::onGoalPose, this, std::placeholders::_1)
        );

        // 订阅区域检测节点发布的区域类型和期望航向角
        region_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/region_type",
            10,
            std::bind(&HandlerNode::onRegionType, this, std::placeholders::_1)
        );

        region_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/region_yaw_desired",
            10,
            std::bind(&HandlerNode::onRegionYaw, this, std::placeholders::_1)
        );

        target_region_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/target_region",
            10,
            std::bind(&HandlerNode::onTargetRegion, this, std::placeholders::_1)
        );

        self_region_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/self_region",
            10,
            std::bind(&HandlerNode::onSelfRegion, this, std::placeholders::_1)
        );

        narrow_passage_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            this->get_parameter("narrow_passage_topic").as_string(),
            10,
            std::bind(&HandlerNode::onNarrowPassage, this, std::placeholders::_1)
        );

        fluctuate_direction_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/fluctuate_direction",
            10,
            std::bind(&HandlerNode::onFluctuateDirection, this, std::placeholders::_1)
        );

        // 声明参数
        this->declare_parameter<double>("tx_hz", 50.0);
        this->declare_parameter<double>("target_x", 0.0);
        this->declare_parameter<double>("target_y", 0.0);
        this->declare_parameter<double>("yaw_desired", 0.0);

        double hz = 50.0;
        this->get_parameter("tx_hz", hz);
        tx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, hz))),
            std::bind(&HandlerNode::publishTxPacket, this)
        );

        // 定时更新位姿（50Hz 足够，降低 CPU 占用）
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50Hz
            std::bind(&HandlerNode::updateMapYaw, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "handler_node started, cmd_vel_frame=%s, angular_z_mode=%s",
            cmd_vel_frame_.c_str(),
            angular_z_mode_.c_str()
        );
    }

    ~HandlerNode() override = default;

private:
    // map 坐标系下的航向角和角速度（map→base_link，用于速度坐标系变换）
    double yaw_in_map_ { 0.0 };
    double prev_yaw_in_map_ { 0.0 };
    double estimated_wz_ { 0.0 };
    rclcpp::Time prev_tf_time_;
    bool has_prev_tf_ { false };

    // 区域检测状态（来自 region_detector_node）
    uint8_t current_region_ { 1 }; // 默认 flat
    float region_yaw_desired_ { 0.0f };
    bool region_active_ { false }; // 是否在特殊区域中
    bool narrow_passage_active_ { false };

    // 区域类型回调
    void onRegionType(const std_msgs::msg::UInt8::SharedPtr msg) {
        current_region_ = msg->data;
        region_active_ = (current_region_ != 1); // 非 flat 即为特殊区域
    }

    // 区域期望航向角回调
    void onRegionYaw(const std_msgs::msg::Float32::SharedPtr msg) {
        region_yaw_desired_ = msg->data;
    }

    void onTargetRegion(const std_msgs::msg::UInt8::SharedPtr msg) {
        nav_info_.target_region = msg->data;
    }

    void onSelfRegion(const std_msgs::msg::UInt8::SharedPtr msg) {
        nav_info_.self_region = msg->data;
    }

    void onNarrowPassage(const std_msgs::msg::Bool::SharedPtr msg) {
        narrow_passage_active_ = msg->data;
    }

    void onFluctuateDirection(const std_msgs::msg::UInt8::SharedPtr msg) {
        nav_info_.fluctuate_direction = msg->data;
    }

    // 定时从 TF 更新位姿：位置、航向角、角速度估计
    void updateMapYaw() {
        // map→base_link：用于速度坐标系变换（底盘 x 轴对齐 base_link）
        try {
            auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            tf2::Quaternion q;
            q.setX(tf.transform.rotation.x);
            q.setY(tf.transform.rotation.y);
            q.setZ(tf.transform.rotation.z);
            q.setW(tf.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // 估计角速度用于预测
            rclcpp::Time now = this->now();
            if (has_prev_tf_) {
                double dt = (now - prev_tf_time_).seconds();
                if (dt > 1e-6) {
                    double dyaw = yaw - prev_yaw_in_map_;
                    while (dyaw > M_PI)
                        dyaw -= 2.0 * M_PI;
                    while (dyaw < -M_PI)
                        dyaw += 2.0 * M_PI;
                    estimated_wz_ = dyaw / dt;
                    // 一阶低通：差分估计的角速度噪声大，乘 predict_time 会放大抖动，先滤波
                    const double alpha =
                        dt / (std::max(wz_filter_tau_, 1e-3) + dt);
                    filtered_wz_ += alpha * (estimated_wz_ - filtered_wz_);
                }
            }
            prev_yaw_in_map_ = yaw;
            prev_tf_time_ = now;
            has_prev_tf_ = true;
            yaw_in_map_ = yaw;
            nav_info_.yaw_current = static_cast<float>(yaw_in_map_);
        } catch (const tf2::TransformException& ex) {
            // TF not ready yet
        }

        // 从 odom→body TF 获取当前位置（odom帧）
        try {
            auto odom_tf = tf_buffer_.lookupTransform("odom", "body", tf2::TimePointZero);
            nav_info_.x_current = static_cast<float>(odom_tf.transform.translation.x);
            nav_info_.y_current = static_cast<float>(odom_tf.transform.translation.y);
        } catch (const tf2::TransformException&) {
            // odom→body not available yet
        }
    }

    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        const double vx = msg->linear.x;
        const double vy = msg->linear.y;

        if (std::abs(vx) < 1e-4 && std::abs(vy) < 1e-4) {
            nav_info_.x_speed = 0.0f;
            nav_info_.y_speed = 0.0f;
            smoothed_world_vx_ = 0.0;
            smoothed_world_vy_ = 0.0;
            has_smoothed_world_velocity_ = false;
            cmd_world_vx_ = 0.0;
            cmd_world_vy_ = 0.0;
            has_cmd_vel_ = false;
        } else if (cmd_vel_frame_ == "base_link") {
            // Nav2 标准 Twist: linear 在 base_link 坐标系下，angular.z 是角速度。
            double vx_base = vx;
            double vy_base = vy;
            if (smooth_world_velocity_) {
                filterBaseCommandInWorldFrame(vx_base, vy_base);
            }
            nav_info_.x_speed = static_cast<float>(vx_base);
            nav_info_.y_speed = static_cast<float>(vy_base);
            has_cmd_vel_ = false;  // base_link 链路不走 TX 时刻变换
        } else {
            // GVC 链路: linear 是 map 坐标系速度。这里只缓存，世界→底盘的变换推迟到
            // publishTxPacket(100Hz) 用最新 yaw 现算，避免 onCmdVel(50Hz) 锁定的旧角度
            // 在车自转期间过时，造成走偏与速度不均。
            cmd_world_vx_ = vx;
            cmd_world_vy_ = vy;
            has_cmd_vel_ = true;
        }

        if (angular_z_mode_ == "yaw_rate") {
            nav_info_.yaw_desired =
                static_cast<float>(normalizeAngle(yaw_in_map_ + msg->angular.z * yaw_rate_preview_time_));
        } else {
            nav_info_.yaw_desired = static_cast<float>(normalizeAngle(msg->angular.z));
        }
    }

    // 在 TX 时刻用最新 yaw 把缓存的 map 系速度变换到底盘系，补偿自转延迟
    void updateChassisSpeedFromWorldCmd() {
        if (cmd_vel_frame_ == "base_link" || !has_cmd_vel_) {
            return;
        }
        const double predicted_yaw =
            yaw_in_map_ + filtered_wz_ * cmd_vel_predict_time_;
        const double cos_yaw = std::cos(predicted_yaw);
        const double sin_yaw = std::sin(predicted_yaw);
        nav_info_.x_speed =
            static_cast<float>(cos_yaw * cmd_world_vx_ + sin_yaw * cmd_world_vy_);
        nav_info_.y_speed =
            static_cast<float>(-sin_yaw * cmd_world_vx_ + cos_yaw * cmd_world_vy_);
    }

    void filterBaseCommandInWorldFrame(double& vx_base, double& vy_base) {
        const rclcpp::Time now = this->now();
        const double cos_yaw = std::cos(yaw_in_map_);
        const double sin_yaw = std::sin(yaw_in_map_);

        double target_world_vx = cos_yaw * vx_base - sin_yaw * vy_base;
        double target_world_vy = sin_yaw * vx_base + cos_yaw * vy_base;

        if (!has_smoothed_world_velocity_) {
            smoothed_world_vx_ = target_world_vx;
            smoothed_world_vy_ = target_world_vy;
            last_cmd_filter_time_ = now;
            has_smoothed_world_velocity_ = true;
        } else {
            double dt = (now - last_cmd_filter_time_).seconds();
            last_cmd_filter_time_ = now;
            dt = std::clamp(dt, 1e-3, 0.1);

            const double alpha = dt / (std::max(world_velocity_filter_tau_, 1e-3) + dt);
            target_world_vx = smoothed_world_vx_ + alpha * (target_world_vx - smoothed_world_vx_);
            target_world_vy = smoothed_world_vy_ + alpha * (target_world_vy - smoothed_world_vy_);

            const double max_delta = std::max(world_velocity_accel_limit_, 0.1) * dt;
            double dvx = target_world_vx - smoothed_world_vx_;
            double dvy = target_world_vy - smoothed_world_vy_;
            const double delta_norm = std::hypot(dvx, dvy);
            if (delta_norm > max_delta && delta_norm > 1e-6) {
                const double scale = max_delta / delta_norm;
                dvx *= scale;
                dvy *= scale;
            }

            smoothed_world_vx_ += dvx;
            smoothed_world_vy_ += dvy;
        }

        vx_base = cos_yaw * smoothed_world_vx_ + sin_yaw * smoothed_world_vy_;
        vy_base = -sin_yaw * smoothed_world_vx_ + cos_yaw * smoothed_world_vy_;
    }

    void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped goal_pose = *msg;
        if (goal_pose.header.frame_id.empty()) {
            goal_pose.header.frame_id = map_frame_;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Goal pose has empty frame_id, assuming %s",
                map_frame_.c_str()
            );
        }

        geometry_msgs::msg::PoseStamped goal_in_map;
        if (!transformPoseToMap(goal_pose, goal_in_map, "goal_pose")) {
            return;
        }

        const double x = goal_in_map.pose.position.x;
        const double y = goal_in_map.pose.position.y;
        setTargetMap(x, y);
        has_navigation_target_ = true;
        resetChassisTrappedMonitor();
        RCLCPP_INFO(
            this->get_logger(),
            "Goal pose received: %s(%.3f, %.3f) -> %s(%.3f, %.3f)",
            goal_pose.header.frame_id.c_str(),
            msg->pose.position.x,
            msg->pose.position.y,
            map_frame_.c_str(),
            x,
            y
        );
    }

    void onRxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        constexpr size_t kRxPacketSize = sizeof(navCommand_t);
        constexpr uint8_t kHeader = 0x72;
        constexpr uint8_t kTailExpected = 0x21;

        if (msg->data.size() != kRxPacketSize) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid rx size: %zu != %zu",
                msg->data.size(),
                kRxPacketSize
            );
            return;
        }

        if (msg->data[0] != kHeader) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid frame header: expected 0x%02X, got 0x%02X",
                kHeader,
                msg->data[0]
            );
            return;
        }

        navCommand_t received_cmd;
        std::memcpy(&received_cmd, msg->data.data(), sizeof(navCommand_t));

        if (received_cmd.frame_tail != kTailExpected) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid frame tail: expected 0x21, got 0x%02X",
                received_cmd.frame_tail
            );
            return;
        }

        last_cmd_ = received_cmd;

        // 根据电控状态更新行为树参数
        updateBehaviorTreeParams(received_cmd);

        if (received_cmd.eSentryState == sentry_state_e::attack
            || received_cmd.eSentryState == sentry_state_e::pursuit) {
            // 发布追击目标点（使用电控下发的 target_x/target_y）
            publishChasePoint(received_cmd);
        }
    }

    // 根据电控状态更新行为树参数
    void updateBehaviorTreeParams(const navCommand_t& cmd) {
        static const std::string kBtNodeName = "/rm_bt_decision_node";

        // 根据电控状态确定是否需要追击
        // attack(1) 和 pursuit(7) 状态都触发追击
        bool should_chase =
            (cmd.eSentryState == sentry_state_e::attack
             || cmd.eSentryState == sentry_state_e::pursuit);

        bool chase_enabled = should_chase;

        // 初始化参数客户端（异步客户端：不会在订阅回调里 spin 本节点导致死锁）
        if (!bt_param_client_) {
            bt_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
                this->shared_from_this(),
                kBtNodeName
            );
        }

        if (!bt_param_client_->service_is_ready()) {
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "BT node parameter service not ready, skip BT params this cycle"
            );
            return;
        }

        // 巡逻状态
        bool patrol_enabled = (cmd.eSentryState == sentry_state_e::patrol);
        bool standby_enabled = (cmd.eSentryState == sentry_state_e::standby);
        bool supply_enabled = (cmd.eSentryState == sentry_state_e::supply);
        bool constrained_defense_enabled =
            (cmd.eSentryState == sentry_state_e::constrained_defense);
        bool go_attack_outpost_enabled = (cmd.eSentryState == sentry_state_e::go_attack_outpost);
        bool rush_base_enabled = (cmd.eSentryState == sentry_state_e::rush_base);
        bool hit_energy_buff_enabled = (cmd.eSentryState == sentry_state_e::hit_energy_buff);
        bool occupy_point_enabled = (cmd.eSentryState == sentry_state_e::occupy_point);
        bool repel_enabled = (cmd.eSentryState == sentry_state_e::repel);

        if (cmd.eSentryState != last_bt_param_state_) {
            last_bt_param_state_ = cmd.eSentryState;
            const auto state_it = state_map.find(cmd.eSentryState);
            const std::string state_name =
                state_it == state_map.end() ? "unknown" : state_it->second;
            RCLCPP_INFO(
                this->get_logger(),
                "BT state update: state=%u(%s), chase=%s, patrol=%s, standby=%s, supply=%s, "
                "constrained_defense=%s, go_attack_outpost=%s, rush_base=%s, "
                "hit_energy_buff=%s, occupy_point=%s, repel=%s",
                cmd.eSentryState,
                state_name.c_str(),
                chase_enabled ? "true" : "false",
                patrol_enabled ? "true" : "false",
                standby_enabled ? "true" : "false",
                supply_enabled ? "true" : "false",
                constrained_defense_enabled ? "true" : "false",
                go_attack_outpost_enabled ? "true" : "false",
                rush_base_enabled ? "true" : "false",
                hit_energy_buff_enabled ? "true" : "false",
                occupy_point_enabled ? "true" : "false",
                repel_enabled ? "true" : "false"
            );
        }

        // 设置行为树参数
        std::vector<rclcpp::Parameter> params = {
            rclcpp::Parameter("chase", chase_enabled),
            rclcpp::Parameter("patrol", patrol_enabled),
            rclcpp::Parameter("standby", standby_enabled),
            rclcpp::Parameter("supply", supply_enabled),
            rclcpp::Parameter("constrained_defense", constrained_defense_enabled),
            rclcpp::Parameter("constrained_defence", constrained_defense_enabled),
            rclcpp::Parameter("go_attack_outpost", go_attack_outpost_enabled),
            rclcpp::Parameter("rush_base", rush_base_enabled),
            rclcpp::Parameter("hit_energy_buff", hit_energy_buff_enabled),
            rclcpp::Parameter("occupy_point", occupy_point_enabled),
            rclcpp::Parameter("repel", repel_enabled),
            rclcpp::Parameter("patrol_region", static_cast<int>(cmd.patrol_region)),
            rclcpp::Parameter("hp", static_cast<double>(cmd.hp_remain)),
            rclcpp::Parameter("ammo", static_cast<double>(cmd.bullet_remain)),
        };

        try {
            bt_param_client_->set_parameters(
                params,
                [this](
                    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future
                ) {
                    try {
                        for (const auto& result: future.get()) {
                            if (!result.successful) {
                                RCLCPP_WARN_THROTTLE(
                                    this->get_logger(),
                                    *this->get_clock(),
                                    2000,
                                    "BT param rejected: %s",
                                    result.reason.c_str()
                                );
                            }
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            2000,
                            "Failed to read BT param result: %s",
                            e.what()
                        );
                    }
                }
            );
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Set BT params: chase=%s, patrol=%s, standby=%s, supply=%s, "
                "constrained_defense=%s, go_attack_outpost=%s, rush_base=%s, "
                "hit_energy_buff=%s, occupy_point=%s, repel=%s, hp=%u, ammo=%u, "
                "state=%d, patrol_region=%u, terrain_region=%d",
                chase_enabled ? "true" : "false",
                patrol_enabled ? "true" : "false",
                standby_enabled ? "true" : "false",
                supply_enabled ? "true" : "false",
                constrained_defense_enabled ? "true" : "false",
                go_attack_outpost_enabled ? "true" : "false",
                rush_base_enabled ? "true" : "false",
                hit_energy_buff_enabled ? "true" : "false",
                occupy_point_enabled ? "true" : "false",
                repel_enabled ? "true" : "false",
                cmd.hp_remain,
                cmd.bullet_remain,
                static_cast<int>(cmd.eSentryState),
                static_cast<unsigned int>(cmd.patrol_region),
                current_region_
            );
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Failed to dispatch BT params: %s",
                e.what()
            );
        }

        // 根据巡逻状态发布巡逻组
        if (cmd.eSentryState == sentry_state_e::patrol) {
            std_msgs::msg::String patrol_msg;
            // 可以根据其他条件选择巡逻组，这里默认 A 组
            patrol_msg.data = "A";
            patrol_group_pub_->publish(patrol_msg);

            // 【关键修复】进入 patrol 模式时，发布当前位置作为目标点，清除之前的导航任务
            try {
                auto current_tf =
                    tf_buffer_.lookupTransform(map_frame_, "base_link", tf2::TimePointZero);

                geometry_msgs::msg::PoseStamped patrol_stop_pose;
                patrol_stop_pose.header.stamp = this->now();
                patrol_stop_pose.header.frame_id = map_frame_;
                patrol_stop_pose.pose.position.x = current_tf.transform.translation.x;
                patrol_stop_pose.pose.position.y = current_tf.transform.translation.y;
                patrol_stop_pose.pose.position.z = 0.0;
                patrol_stop_pose.pose.orientation = current_tf.transform.rotation;

                chase_point_pub_->publish(patrol_stop_pose);

                // RCLCPP_INFO_THROTTLE(
                //     this->get_logger(),
                //     *this->get_clock(),
                //     500,
                //     "Patrol mode: Published current position to clear previous nav goal: map(%.2f, %.2f)",
                //     patrol_stop_pose.pose.position.x,
                //     patrol_stop_pose.pose.position.y
                // );
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Failed to get current position for patrol stop: %s",
                    ex.what()
                );
            }
        }

        // 记录状态变化
        if (cmd.eSentryState != last_sentry_state_) {
            last_sentry_state_ = static_cast<sentry_state_e>(cmd.eSentryState);
        }
    }

    // 发布追击目标点
    void publishChasePoint(const navCommand_t& cmd) {
        // 检查追击功能是否启用
        if (!enable_chase_) {
            return;
        }

        // 【关键修改】检查导航目标有效性：电控发送 odom 坐标系下的 target 点
        // 当电控想要停止时，会发送车体当前位置 (x_current, y_current)
        // 因此需要检查目标点与当前位置的距离，而不是与原点的距离

        double dx_odom = cmd.target_x - nav_info_.x_current;
        double dy_odom = cmd.target_y - nav_info_.y_current;
        double distance_in_odom = std::sqrt(dx_odom * dx_odom + dy_odom * dy_odom);

        if (distance_in_odom < stop_distance_threshold_) {
            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(),
            //     *this->get_clock(),
            //     500,
            //     "Stop command detected: target odom(%.2f, %.2f) ≈ current odom(%.2f, %.2f), dist=%.3fm - Publishing current position as stop target",
            //     cmd.target_x,
            //     cmd.target_y,
            //     nav_info_.x_current,
            //     nav_info_.y_current,
            //     distance_in_odom
            // );

            // 主动发布当前位置作为目标点，让车立即停止
            // 获取当前车体在 map 坐标系下的位置
            try {
                auto current_tf =
                    tf_buffer_.lookupTransform(map_frame_, "base_link", tf2::TimePointZero);

                // 发布当前位置作为追击目标
                geometry_msgs::msg::PoseStamped stop_pose;
                stop_pose.header.stamp = this->now();
                stop_pose.header.frame_id = map_frame_;
                stop_pose.pose.position.x = current_tf.transform.translation.x;
                stop_pose.pose.position.y = current_tf.transform.translation.y;
                stop_pose.pose.position.z = 0.0;
                stop_pose.pose.orientation = current_tf.transform.rotation;

                chase_point_pub_->publish(stop_pose);

                // RCLCPP_INFO_THROTTLE(
                //     this->get_logger(),
                //     *this->get_clock(),
                //     500,
                //     "Published STOP target at current position: map(%.2f, %.2f)",
                //     stop_pose.pose.position.x,
                //     stop_pose.pose.position.y
                // );
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Failed to get current position for stop command: %s",
                    ex.what()
                );
            }

            return;
        }

        // 电控发来的 target 坐标是 odom 坐标系，需要转换到 map 坐标系
        geometry_msgs::msg::PoseStamped target_in_odom;
        target_in_odom.header.stamp = this->now();
        target_in_odom.header.frame_id = "odom";
        target_in_odom.pose.position.x = cmd.target_x;
        target_in_odom.pose.position.y = cmd.target_y;
        target_in_odom.pose.position.z = 0.0;
        target_in_odom.pose.orientation.w = 1.0;

        geometry_msgs::msg::PoseStamped target_in_map;
        try {
            // 获取 odom -> map 的变换并转换坐标
            target_in_map =
                tf_buffer_.transform(target_in_odom, map_frame_, tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Failed to transform target position from odom to %s: %s",
                map_frame_.c_str(),
                ex.what()
            );
            return;
        }

        double target_map_x = target_in_map.pose.position.x;
        double target_map_y = target_in_map.pose.position.y;

        // 构建 PoseStamped 消息（已经是 map 坐标系）
        geometry_msgs::msg::PoseStamped chase_pose;
        chase_pose.header.stamp = this->now();
        chase_pose.header.frame_id = map_frame_;
        chase_pose.pose.position.x = target_map_x;
        chase_pose.pose.position.y = target_map_y;
        chase_pose.pose.position.z = 0.0;
        // 默认朝向
        chase_pose.pose.orientation.x = 0.0;
        chase_pose.pose.orientation.y = 0.0;
        chase_pose.pose.orientation.z = 0.0;
        chase_pose.pose.orientation.w = 1.0;

        chase_point_pub_->publish(chase_pose);

        // 同步更新 map 目标缓存；TX 发包前会转换为 odom 坐标发给电控。
        setTargetMap(target_map_x, target_map_y);
        has_navigation_target_ = true;
        resetChassisTrappedMonitor();

        // RCLCPP_INFO_THROTTLE(
        //     this->get_logger(),
        //     *this->get_clock(),
        //     500,
        //     "Published chase point: odom(%.2f, %.2f) -> map(%.2f, %.2f), self_odom(%.2f, %.2f), dist=%.2fm",
        //     cmd.target_x,
        //     cmd.target_y,
        //     target_map_x,
        //     target_map_y,
        //     nav_info_.x_current,
        //     nav_info_.y_current,
        //     distance_in_odom
        // );
    }

    void publishTxPacket() {
        nav_info_.frame_header = 0x72;
        nav_info_.frame_tail = 0x4D;

        // 用最新 yaw 把缓存的 map 系速度命令变换到底盘系（补偿边转边走的自转延迟）
        updateChassisSpeedFromWorldCmd();

        // 使用 region_detector 发布的区域类型
        // 注意：颠簸区域（fluctuate）对电控上报为 hole，保持行为一致
        nav_info_.sentry_region = (current_region_ == sentry_region::fluctuate)
            ? static_cast<uint8_t>(sentry_region::hole)
            : current_region_;
        
        // yaw_desired is always the path tangent angle from GVC. The controller
        // decides whether to use it based on the narrow-passage flag below.
        nav_info_.narrow_passage = narrow_passage_active_ ? 1 : 0;

        double target_map_x = 0.0, target_map_y = 0.0;
        (void)this->get_parameter("target_x", target_map_x);
        (void)this->get_parameter("target_y", target_map_y);
        updateTxTargetInOdom(target_map_x, target_map_y);
        updateChassisTrappedFlag();

        // 把卡住标志发布到 ROS 话题，供决策层读取（决策层据此切换备选目标点）
        if (chassis_trapped_pub_) {
            std_msgs::msg::Bool trapped_msg;
            trapped_msg.data = (nav_info_.chises_trapped != 0);
            chassis_trapped_pub_->publish(trapped_msg);
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            tx_log_period_ms_,
            "TX -> x_speed: %.3f y_speed: %.3f x_current: %.3f y_current: %.3f "
            "x_target: %.3f y_target: %.3f yaw_current: %.3f yaw_desired: %.3f sentry_region: %d "
            "target_region: %u self_region: %u motion: %u trapped: %u narrow: %u fluc_dir: %u",
            nav_info_.x_speed,
            nav_info_.y_speed,
            nav_info_.x_current,
            nav_info_.y_current,
            nav_info_.x_target,
            nav_info_.y_target,
            nav_info_.yaw_current,
            nav_info_.yaw_desired,
            nav_info_.sentry_region,
            static_cast<unsigned int>(nav_info_.target_region),
            static_cast<unsigned int>(nav_info_.self_region),
            static_cast<unsigned int>(last_cmd_.motion_allowed),
            static_cast<unsigned int>(nav_info_.chises_trapped),
            static_cast<unsigned int>(nav_info_.narrow_passage),
            static_cast<unsigned int>(nav_info_.fluctuate_direction)
        );

        std_msgs::msg::UInt8MultiArray out_msg;
        const uint8_t* byte_ptr = reinterpret_cast<const uint8_t*>(&nav_info_);
        size_t data_size = sizeof(navInfo_t);
        out_msg.data.assign(byte_ptr, byte_ptr + data_size);
        tx_pub_->publish(out_msg);
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    void setTargetMap(double x, double y) {
        this->set_parameter(rclcpp::Parameter("target_x", x));
        this->set_parameter(rclcpp::Parameter("target_y", y));
    }

    void resetChassisTrappedMonitor() {
        trapped_monitor_active_ = false;
        chassis_trapped_active_ = false;
        trapped_center_x_ = 0.0;
        trapped_center_y_ = 0.0;
        trapped_region_enter_time_ = this->now();
        motion_disallowed_timing_active_ = false;
        nav_info_.chises_trapped = 0;
    }

    void updateChassisTrappedFlag() {
        if (last_cmd_.motion_allowed == 0) {
            const rclcpp::Time now = this->now();
            if (!motion_disallowed_timing_active_) {
                motion_disallowed_timing_active_ = true;
                motion_disallowed_since_ = now;
            }

            const double disallowed_duration = (now - motion_disallowed_since_).seconds();
            if (disallowed_duration >= motion_disallowed_clear_timeout_) {
                resetChassisTrappedMonitor();
                return;
            }
        } else {
            motion_disallowed_timing_active_ = false;
        }

        if (!has_navigation_target_) {
            resetChassisTrappedMonitor();
            return;
        }

        const double dx_goal = nav_info_.x_target - nav_info_.x_current;
        const double dy_goal = nav_info_.y_target - nav_info_.y_current;
        const double dist_to_goal = std::hypot(dx_goal, dy_goal);
        if (dist_to_goal <= chassis_trapped_goal_tolerance_) {
            resetChassisTrappedMonitor();
            has_navigation_target_ = false;
            return;
        }

        const rclcpp::Time now = this->now();
        if (!trapped_monitor_active_) {
            trapped_monitor_active_ = true;
            trapped_center_x_ = nav_info_.x_current;
            trapped_center_y_ = nav_info_.y_current;
            trapped_region_enter_time_ = now;
            chassis_trapped_active_ = false;
            nav_info_.chises_trapped = 0;
            return;
        }

        const double dx_center = nav_info_.x_current - trapped_center_x_;
        const double dy_center = nav_info_.y_current - trapped_center_y_;
        const double dist_from_center = std::hypot(dx_center, dy_center);
        if (dist_from_center > chassis_trapped_radius_) {
            trapped_center_x_ = nav_info_.x_current;
            trapped_center_y_ = nav_info_.y_current;
            trapped_region_enter_time_ = now;
            chassis_trapped_active_ = false;
            nav_info_.chises_trapped = 0;
            return;
        }

        const double stuck_duration = (now - trapped_region_enter_time_).seconds();
        const bool trapped = stuck_duration >= chassis_trapped_timeout_;
        if (trapped != chassis_trapped_active_) {
            chassis_trapped_active_ = trapped;
            RCLCPP_WARN(
                this->get_logger(),
                "Chassis trapped flag %s: stayed within %.2fm for %.1fs, dist_to_goal=%.2fm",
                trapped ? "ON" : "OFF",
                chassis_trapped_radius_,
                stuck_duration,
                dist_to_goal
            );
        }
        nav_info_.chises_trapped = chassis_trapped_active_ ? 1 : 0;
    }

    bool transformPoseToMap(
        const geometry_msgs::msg::PoseStamped& pose,
        geometry_msgs::msg::PoseStamped& pose_in_map,
        const char* source_name
    ) {
        if (pose.header.frame_id == map_frame_) {
            pose_in_map = pose;
            return true;
        }

        try {
            pose_in_map = tf_buffer_.transform(pose, map_frame_, tf2::durationFromSec(0.1));
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Failed to transform %s from %s to %s: %s",
                source_name,
                pose.header.frame_id.c_str(),
                map_frame_.c_str(),
                ex.what()
            );
            return false;
        }
    }

    bool updateTxTargetInOdom(double target_map_x, double target_map_y) {
        if (map_frame_ == "odom") {
            nav_info_.x_target = static_cast<float>(target_map_x);
            nav_info_.y_target = static_cast<float>(target_map_y);
            return true;
        }

        geometry_msgs::msg::PoseStamped target_in_map;
        target_in_map.header.stamp = this->now();
        target_in_map.header.frame_id = map_frame_;
        target_in_map.pose.position.x = target_map_x;
        target_in_map.pose.position.y = target_map_y;
        target_in_map.pose.position.z = 0.0;
        target_in_map.pose.orientation.w = 1.0;

        try {
            geometry_msgs::msg::PoseStamped target_in_odom;
            const auto map_to_odom_tf =
                tf_buffer_.lookupTransform("odom", map_frame_, tf2::TimePointZero);
            tf2::doTransform(target_in_map, target_in_odom, map_to_odom_tf);
            nav_info_.x_target = static_cast<float>(target_in_odom.pose.position.x);
            nav_info_.y_target = static_cast<float>(target_in_odom.pose.position.y);
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Failed to transform TX target from %s to odom, keeping previous odom target: %s",
                map_frame_.c_str(),
                ex.what()
            );
            return false;
        }
    }

    // 发布器
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr patrol_group_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr chase_point_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr chassis_trapped_pub_;

    // 上次状态（用于检测状态变化）
    sentry_state_e last_sentry_state_ { sentry_state_e::standby };

    // 订阅器
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr region_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr region_yaw_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr target_region_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr self_region_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr narrow_passage_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fluctuate_direction_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr tx_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // TF 监听器
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 数据
    navInfo_t nav_info_ {};
    navCommand_t last_cmd_ {};
    uint8_t last_bt_param_state_ { 255 };

    // 追击相关参数
    double chase_min_distance_ { 0.25 };
    double stop_distance_threshold_ { 0.05 };
    std::string chase_topic_ { "/chase_point" };
    std::string map_frame_ { "map" };
    std::string cmd_vel_frame_ { "map" };
    std::string angular_z_mode_ { "yaw_angle" };
    double yaw_rate_preview_time_ { 0.15 };
    // map 系速度的世界→底盘变换在 TX 时刻(100Hz)用最新 yaw 现算，补偿边转边走漂移
    double cmd_vel_predict_time_ { 0.08 };  // TX 到电控执行的下游延迟，预测 yaw 提前量
    double wz_filter_tau_ { 0.05 };         // 角速度低通时间常数，抑制差分噪声放大
    double filtered_wz_ { 0.0 };            // 滤波后的角速度
    double cmd_world_vx_ { 0.0 };           // 缓存的 map 系速度命令
    double cmd_world_vy_ { 0.0 };
    bool has_cmd_vel_ { false };
    bool smooth_world_velocity_ { true };
    double world_velocity_filter_tau_ { 0.12 };
    double world_velocity_accel_limit_ { 1.2 };
    bool has_smoothed_world_velocity_ { false };
    double smoothed_world_vx_ { 0.0 };
    double smoothed_world_vy_ { 0.0 };
    rclcpp::Time last_cmd_filter_time_;
    bool enable_chase_ { true };

    double chassis_trapped_radius_ { 1.0 };
    double chassis_trapped_timeout_ { 30.0 };
    double chassis_trapped_goal_tolerance_ { 0.30 };
    double motion_disallowed_clear_timeout_ { 5.0 };
    int tx_log_period_ms_ { 1000 };
    bool has_navigation_target_ { false };
    bool trapped_monitor_active_ { false };
    bool chassis_trapped_active_ { false };
    bool motion_disallowed_timing_active_ { false };
    double trapped_center_x_ { 0.0 };
    double trapped_center_y_ { 0.0 };
    rclcpp::Time trapped_region_enter_time_;
    rclcpp::Time motion_disallowed_since_;

    // BT 节点参数客户端
    std::shared_ptr<rclcpp::AsyncParametersClient> bt_param_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandlerNode>());
    rclcpp::shutdown();
    return 0;
}
