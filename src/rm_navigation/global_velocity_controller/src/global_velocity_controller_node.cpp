#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

// 从原代码中保留的仿真器，用于闭环测试
#include "global_velocity_controller/simulator_2d.hpp"
// 注意: gvc::Simulator2D 依赖于 types.hpp, 这里假设它存在
#include "global_velocity_controller/types.hpp"

const double SIM_OMEGA = 1.0;

using std::placeholders::_1;

class SimplifiedControllerNode: public rclcpp::Node {
public:
    SimplifiedControllerNode():
        Node("simplified_velocity_controller"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_),
        tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {
        // --- 参数声明 ---
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("base_frame", "base_link");
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        declare_parameter<std::string>("path_topic", "/plan");
        declare_parameter<std::string>("global_costmap_topic", "/global_costmap/costmap");
        declare_parameter<std::string>(
            "fluctuate_distance_topic",
            "/distance_to_fluctuate_region"
        );
        declare_parameter<std::string>("region_topic", "/region_type");
        declare_parameter<int>("fluctuate_region_value", 5);

        // 位控PID增益
        declare_parameter<double>("kp_xy", 1.5);
        declare_parameter<double>("ki_xy", 0.1);
        declare_parameter<double>("kd_xy", 0.2);
        declare_parameter<double>("kp_yaw", 2.0);

        // 速度和加速度限制
        declare_parameter<double>("max_vx", 0.8);
        declare_parameter<double>("max_vy", 0.0); // 通常对于非全向机器人，vy为0
        declare_parameter<double>("max_wz", 1.2);
        declare_parameter<double>("cmd_accel_limit_linear", 2.0);
        declare_parameter<double>("cmd_accel_limit_angular", 4.0);
        declare_parameter<double>("fluctuate_decel_distance", 2.0);
        declare_parameter<double>("fluctuate_prepare_distance", 1.1);
        declare_parameter<double>("fluctuate_prepare_max_speed", 0.35);
        declare_parameter<double>("fluctuate_min_approach_speed", 0.22);

        // 路径跟踪参数
        declare_parameter<double>("goal_tolerance", 0.1);
        // 航向预测时间因子（0.0~1.0，通常0.5~1.0）
        declare_parameter<double>("prediction_time_factor", 1.0);

        // --- 新增: 动态前瞻距离参数 ---
        declare_parameter<bool>("enable_dynamic_lookahead", true);
        declare_parameter<double>("lookahead_distance", 0.5); // 如果动态关闭，则使用此固定值
        declare_parameter<double>("min_lookahead_distance", 0.3);
        declare_parameter<double>("max_lookahead_distance", 1.0);
        declare_parameter<double>("curvature_window_distance", 2.0); // 计算曲率的路径长度
        declare_parameter<double>("curvature_low", 0.1); // 曲率阈值下限
        declare_parameter<double>("curvature_high", 1.0); // 曲率阈值上限
        declare_parameter<double>("path_yaw_filter_tau", 0.20);
        declare_parameter<double>("path_yaw_max_rate", 3.0);

        // 路径跟随律 (沿切线前馈 + 横向纠偏) 与终点位置闭环减速
        declare_parameter<bool>("use_path_following", true);
        declare_parameter<double>("cruise_speed", 1.0);
        declare_parameter<double>("kp_cross", 1.5);
        declare_parameter<double>("approach_distance", 0.7);
        declare_parameter<double>("approach_gain", 1.5);
        declare_parameter<double>("brake_latency", 0.1);
        declare_parameter<double>("brake_stop_margin", 0.15);

        // 过弯限速：预瞄前方累计转角，按“过弯甩出量 d≈v^2*sin(Φ/2)/a”反解安全速度。
        // 直线保持全速，弯道自动压速。Theta* 是折线路径，用窗口累计转角而非逐点曲率。
        declare_parameter<bool>("enable_corner_slowdown", true);
        declare_parameter<double>("corner_lookahead_distance", 1.0);
        declare_parameter<double>("corner_cut_tolerance", 0.15);
        declare_parameter<double>("corner_min_speed", 0.3);

        // 窄道检测与通过策略
        declare_parameter<bool>("narrow_detection_enabled", true);
        declare_parameter<std::string>("narrow_passage_topic", "/narrow_passage");
        declare_parameter<std::string>("narrow_passage_width_topic", "/narrow_passage_width");
        declare_parameter<double>("narrow_lookahead_distance", 1.5);
        declare_parameter<double>("narrow_sample_step", 0.10);
        declare_parameter<double>("narrow_scan_step", 0.03);
        declare_parameter<double>("narrow_scan_max_side_distance", 1.2);
        declare_parameter<double>("narrow_enter_width", 0.90);
        declare_parameter<double>("narrow_exit_width", 1.10);
        declare_parameter<double>("narrow_min_length", 0.40);
        declare_parameter<int>("narrow_obstacle_cost_threshold", 99);
        declare_parameter<bool>("narrow_treat_unknown_as_blocked", true);
        declare_parameter<double>("narrow_speed_limit", 0.35);

        // 仿真相关参数
        declare_parameter<bool>("simulate", true);
        declare_parameter<double>("sim_init_x", 0.0);
        declare_parameter<double>("sim_init_y", 0.0);
        declare_parameter<double>("sim_init_yaw", 0.0);
        declare_parameter<double>("max_dt", 0.05);

        // Escape mode params (from gvc_old)
        declare_parameter<int>("escape_free_cost_value", 0);
        declare_parameter<int>("escape_target_cost_threshold", 0);
        declare_parameter<int>("escape_enter_cost_threshold", 1);
        declare_parameter<int>("escape_lethal_threshold", 100);
        declare_parameter<bool>("escape_treat_unknown_as_lethal", true);
        declare_parameter<double>("escape_speed", 0.4);
        declare_parameter<double>("escape_goal_tolerance", 0.05);
        declare_parameter<int>("escape_max_radius_cells", 200);
        declare_parameter<double>("escape_enter_delay_s", 2.0);

        // --- 获取参数 ---
        map_frame_ = get_parameter("map_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        goal_tolerance_ = get_parameter("goal_tolerance").as_double();
        max_dt_ = get_parameter("max_dt").as_double();
        prediction_time_factor_ = get_parameter("prediction_time_factor").as_double();

        kp_xy_ = get_parameter("kp_xy").as_double();
        ki_xy_ = get_parameter("ki_xy").as_double();
        kd_xy_ = get_parameter("kd_xy").as_double();
        kp_yaw_ = get_parameter("kp_yaw").as_double();

        max_vx_ = get_parameter("max_vx").as_double();
        max_vy_ = get_parameter("max_vy").as_double();
        max_wz_ = get_parameter("max_wz").as_double();
        cmd_accel_limit_linear_ = get_parameter("cmd_accel_limit_linear").as_double();
        cmd_accel_limit_angular_ = get_parameter("cmd_accel_limit_angular").as_double();
        fluctuate_decel_distance_ = get_parameter("fluctuate_decel_distance").as_double();
        fluctuate_prepare_distance_ = get_parameter("fluctuate_prepare_distance").as_double();
        fluctuate_prepare_max_speed_ =
            get_parameter("fluctuate_prepare_max_speed").as_double();
        fluctuate_min_approach_speed_ =
            get_parameter("fluctuate_min_approach_speed").as_double();
        fluctuate_region_value_ =
            static_cast<uint8_t>(get_parameter("fluctuate_region_value").as_int());

        // --- 新增: 获取动态前瞻距离参数 ---
        enable_dynamic_lookahead_ = get_parameter("enable_dynamic_lookahead").as_bool();
        lookahead_distance_ = get_parameter("lookahead_distance").as_double();
        min_lookahead_distance_ = get_parameter("min_lookahead_distance").as_double();
        max_lookahead_distance_ = get_parameter("max_lookahead_distance").as_double();
        curvature_window_distance_ = get_parameter("curvature_window_distance").as_double();
        curvature_low_ = get_parameter("curvature_low").as_double();
        curvature_high_ = get_parameter("curvature_high").as_double();
        path_yaw_filter_tau_ = get_parameter("path_yaw_filter_tau").as_double();
        path_yaw_max_rate_ = get_parameter("path_yaw_max_rate").as_double();

        use_path_following_ = get_parameter("use_path_following").as_bool();
        cruise_speed_ = get_parameter("cruise_speed").as_double();
        kp_cross_ = get_parameter("kp_cross").as_double();
        approach_distance_ = get_parameter("approach_distance").as_double();
        approach_gain_ = get_parameter("approach_gain").as_double();
        brake_latency_ = get_parameter("brake_latency").as_double();
        brake_stop_margin_ = get_parameter("brake_stop_margin").as_double();

        enable_corner_slowdown_ = get_parameter("enable_corner_slowdown").as_bool();
        corner_lookahead_distance_ = get_parameter("corner_lookahead_distance").as_double();
        corner_cut_tolerance_ = get_parameter("corner_cut_tolerance").as_double();
        corner_min_speed_ = get_parameter("corner_min_speed").as_double();

        narrow_detection_enabled_ = get_parameter("narrow_detection_enabled").as_bool();
        narrow_lookahead_distance_ = get_parameter("narrow_lookahead_distance").as_double();
        narrow_sample_step_ = get_parameter("narrow_sample_step").as_double();
        narrow_scan_step_ = get_parameter("narrow_scan_step").as_double();
        narrow_scan_max_side_distance_ =
            get_parameter("narrow_scan_max_side_distance").as_double();
        narrow_enter_width_ = get_parameter("narrow_enter_width").as_double();
        narrow_exit_width_ = get_parameter("narrow_exit_width").as_double();
        narrow_min_length_ = get_parameter("narrow_min_length").as_double();
        narrow_obstacle_cost_threshold_ =
            get_parameter("narrow_obstacle_cost_threshold").as_int();
        narrow_treat_unknown_as_blocked_ =
            get_parameter("narrow_treat_unknown_as_blocked").as_bool();
        narrow_speed_limit_ = get_parameter("narrow_speed_limit").as_double();

        // Escape params
        escape_free_cost_value_ = get_parameter("escape_free_cost_value").as_int();
        escape_target_cost_threshold_ = get_parameter("escape_target_cost_threshold").as_int();
        escape_enter_cost_threshold_ = get_parameter("escape_enter_cost_threshold").as_int();
        escape_lethal_threshold_ = get_parameter("escape_lethal_threshold").as_int();
        escape_treat_unknown_as_lethal_ = get_parameter("escape_treat_unknown_as_lethal").as_bool();
        escape_speed_ = get_parameter("escape_speed").as_double();
        escape_goal_tolerance_ = get_parameter("escape_goal_tolerance").as_double();
        escape_max_radius_cells_ = get_parameter("escape_max_radius_cells").as_int();
        escape_enter_delay_s_ = get_parameter("escape_enter_delay_s").as_double();

        // 配置仿真器
        simulate_ = get_parameter("simulate").as_bool();
        if (simulate_) {
            gvc::SimulatorConfig sim_config;
            sim_config.init_x = get_parameter("sim_init_x").as_double();
            sim_config.init_y = get_parameter("sim_init_y").as_double();
            sim_config.init_yaw = get_parameter("sim_init_yaw").as_double();
            simulator_ = gvc::Simulator2D(sim_config);
            RCLCPP_INFO(get_logger(), "Simulation mode is ON.");
        }

        // --- ROS接口 ---
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            get_parameter("cmd_vel_topic").as_string(),
            10
        );
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            get_parameter("path_topic").as_string(),
            10,
            std::bind(&SimplifiedControllerNode::onPath, this, _1)
        );

        costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            get_parameter("global_costmap_topic").as_string(),
            1,
            std::bind(&SimplifiedControllerNode::onCostmap, this, _1)
        );
        fluctuate_distance_sub_ = create_subscription<std_msgs::msg::Float32>(
            get_parameter("fluctuate_distance_topic").as_string(),
            10,
            std::bind(&SimplifiedControllerNode::onFluctuateDistance, this, _1)
        );
        region_sub_ = create_subscription<std_msgs::msg::UInt8>(
            get_parameter("region_topic").as_string(),
            10,
            std::bind(&SimplifiedControllerNode::onRegionType, this, _1)
        );
        narrow_passage_pub_ = create_publisher<std_msgs::msg::Bool>(
            get_parameter("narrow_passage_topic").as_string(),
            10
        );
        narrow_passage_width_pub_ = create_publisher<std_msgs::msg::Float32>(
            get_parameter("narrow_passage_width_topic").as_string(),
            10
        );

        // Timers: separate escape and normal control loops
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SimplifiedControllerNode::onControlTimer, this)
        );
        escape_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SimplifiedControllerNode::onEscapeTimer, this)
        );

        RCLCPP_INFO(get_logger(), "Simplified Position-Control PID Node has started.");
    }

private:
    void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            has_path_ = false;
            RCLCPP_WARN(get_logger(), "Received an empty path.");
        } else {
            last_path_ = *msg;
            has_path_ = true;
            narrow_passage_active_ = false;
            narrow_passage_width_m_ = -1.0;
            // 重置PID积分和之前的状态，开始新的路径跟踪
            resetControllerState();
            RCLCPP_INFO(get_logger(), "New path received with %zu poses.", msg->poses.size());
        }
    }

    void onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        last_costmap_ = *msg;
        has_costmap_ = true;
    }

    void onFluctuateDistance(const std_msgs::msg::Float32::SharedPtr msg) {
        distance_to_fluctuate_region_ = msg->data;
    }

    void onRegionType(const std_msgs::msg::UInt8::SharedPtr msg) {
        current_region_ = msg->data;
    }

    void onControlTimer() {
        const rclcpp::Time now = get_clock()->now();
        // If currently escaping, skip normal control (escape timer will publish)
        if (in_escape_mode_) {
            if (simulate_)
                publishSimulatedTransform(now);
            return;
        }

        // 1. 获取机器人当前位姿
        double current_x, current_y, current_yaw;
        if (!getCurrentPose(current_x, current_y, current_yaw)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Could not get current robot pose."
            );
            publishZeroTwist();
            return;
        }

        if (!has_path_) {
            // 没有路径也不在逃逸时，停止机器人，并在仿真下持续发布TF
            if (narrow_passage_active_) {
                narrow_passage_active_ = false;
                narrow_passage_width_m_ = -1.0;
                narrow_path_yaw_ = std::numeric_limits<double>::quiet_NaN();
                publishNarrowPassageState();
            }
            publishZeroTwist();
            if (simulate_)
                publishSimulatedTransform(now);
            return;
        }

        // 2. 在路径上寻找前瞻点(Lookahead Point)
        geometry_msgs::msg::Point lookahead_pt;
        geometry_msgs::msg::Point closest_pt;
        double path_yaw = current_yaw;

        bool is_final_goal = false;
        if (!findLookaheadPoint(
                current_x,
                current_y,
                lookahead_pt,
                closest_pt,
                path_yaw,
                is_final_goal
            ))
        {
            RCLCPP_INFO_ONCE(get_logger(), "Path tracking complete or invalid path.");
            has_path_ = false; // 标记路径结束
            narrow_passage_active_ = false;
            publishNarrowPassageState();
            publishZeroTwist();
            resetControllerState();
            return;
        }

        updateNarrowPassageState(current_x, current_y);

        // 3. 计算位置误差 (在map坐标系下)
        const double ex_lookahead = lookahead_pt.x - current_x;
        const double ey_lookahead = lookahead_pt.y - current_y;

        double ex_closest = closest_pt.x - current_x;
        double ey_closest = closest_pt.y - current_y;

        // if (std::hypot(ex_closest, ey_closest) > 0.2) {
        //   ex_closest = 0.0;
        //   ey_closest = 0.0;
        //   geometry_msgs::msg::Twist cmd;
        //   cmd_pub_->publish(cmd);
        //   return;
        // }

        // 如果接近最终目标点，则进行特殊处理
        const double dist_to_goal = std::hypot(ex_lookahead, ey_lookahead);

        // 【修复】如果目标点过近（可能是机器人当前位置），立即停止
        // 这处理了电控发送(0,0)点导致目标就在当前位置的情况
        if (dist_to_goal < 0.15) { // 15cm以内视为已到达
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                500,
                "Target too close (%.3fm), stopping immediately",
                dist_to_goal
            );
            publishZeroTwist();
            if (is_final_goal) {
                has_path_ = false; // 如果是最终目标，清除路径
                resetControllerState();
            }
            return;
        }

        if (is_final_goal && dist_to_goal < goal_tolerance_) {
            RCLCPP_INFO(get_logger(), "Goal reached!");
            has_path_ = false; // 标记路径结束
            publishZeroTwist();
            resetControllerState();
            return;
        }

        // 4. 计算时间差 dt
        if (!has_prev_time_) {
            prev_time_ = now;
            has_prev_time_ = true;
            // --- 新增: 第一次循环时也记录航向角 ---
            if (!has_prev_yaw_) {
                prev_yaw_ = current_yaw;
                has_prev_yaw_ = true;
            }
            return;
        }
        double dt = (now - prev_time_).seconds();
        prev_time_ = now;
        if (dt <= 0.0)
            return;
        dt = std::min(dt, max_dt_);

        // --- 核心修改 1: 航向角预测 ---
        double predicted_yaw = current_yaw; // 默认情况下，预测值等于当前值
        if (has_prev_yaw_ && dt > 1e-6) {
            // 根据前后两次的角度差，估算当前角速度
            const double estimated_wz = normalizeAngle(current_yaw - prev_yaw_) / dt;

            // 预测一个短暂未来(例如半个控制周期)的航向角
            // 可调参数 prediction_time_factor_，建议设为0.5~1.0
            predicted_yaw = current_yaw + estimated_wz * dt * prediction_time_factor_;
            predicted_yaw = normalizeAngle(predicted_yaw);
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "Predicted yaw: %.3f (current: %.3f, estimated_wz: %.3f)",
                predicted_yaw,
                current_yaw,
                estimated_wz
            );
        }
        prev_yaw_ = current_yaw; // 更新上一次的航向角
        has_prev_yaw_ = true;

        // 5. 计算期望速度 (map 坐标系)
        double vx_map_cmd = 0.0;
        double vy_map_cmd = 0.0;

        if (use_path_following_) {
            // 到最终目标点的距离
            const auto& goal_pos = last_path_.poses.back().pose.position;
            const double gdx = goal_pos.x - current_x;
            const double gdy = goal_pos.y - current_y;
            const double dist_to_final = std::hypot(gdx, gdy);
            const double speed_now = std::hypot(last_cmd_vx_, last_cmd_vy_);

            if (dist_to_final <= approach_distance_) {
                // 终点接近段：对目标点做位置闭环 P 控制，速度随剩余距离线性下降。
                // 冲过目标时方向矢量自动翻转把车拉回，自带阻尼，不会像开环前馈那样反复过冲。
                // 减去停车余量与 latency*速度（超前阻尼项），抑制链路延迟造成的过冲。
                const double e_eff =
                    dist_to_final - brake_stop_margin_ - speed_now * brake_latency_;
                const double speed = std::clamp(approach_gain_ * e_eff, 0.0, cruise_speed_);
                if (dist_to_final > 1e-6) {
                    vx_map_cmd = speed * gdx / dist_to_final;
                    vy_map_cmd = speed * gdy / dist_to_final;
                }
            } else {
                // 巡航段：沿“最近点”的本地路径切线前馈 + 横向纠偏。
                // 关键（防切弯）：切线取最近点方向而非前瞻点方向。否则拐角前前瞻点已越过
                // 拐角，前馈会提前指向拐角后的方向 → 车提前内切、撞上拐角内侧。
                std::size_t closest_idx = 0;
                findClosestPathIndex(current_x, current_y, closest_idx);
                const double local_yaw = computePathYawAtIndex(closest_idx);
                const double t_x = std::cos(local_yaw);
                const double t_y = std::sin(local_yaw);
                const double along = ex_closest * t_x + ey_closest * t_y;
                const double e_perp_x = ex_closest - along * t_x;
                const double e_perp_y = ey_closest - along * t_y;
                // 过弯限速：直线保持全速，弯道按前方累计转角自动压速
                const double fwd_speed = computeCornerSpeedLimit(closest_idx);
                vx_map_cmd = fwd_speed * t_x + kp_cross_ * e_perp_x;
                vy_map_cmd = fwd_speed * t_y + kp_cross_ * e_perp_y;
            }
        } else {
            // 兼容旧的纯前瞻点 PID 控制律（横向纠偏权重为 0，弯道会内切）。
            integral_x_ += ex_closest * dt;
            integral_y_ += ey_closest * dt;
            integral_x_ = std::clamp(integral_x_, -0.5, 0.5);
            integral_y_ = std::clamp(integral_y_, -0.5, 0.5);

            const double deriv_ex = (ex_lookahead - prev_ex_) / dt;
            const double deriv_ey = (ey_lookahead - prev_ey_) / dt;
            const double px = ex_lookahead;
            const double py = ey_lookahead;

            vx_map_cmd = kp_xy_ * px + ki_xy_ * integral_x_ + kd_xy_ * deriv_ex;
            vy_map_cmd = kp_xy_ * py + ki_xy_ * integral_y_ + kd_xy_ * deriv_ey;

            prev_ex_ = px;
            prev_ey_ = py;
        }

        // 6. 航向控制,下位机只需要给出角度即可
        double target_yaw = filterPathYaw(path_yaw, dt);

        // 7. 【陀螺模式修复】直接发送map坐标系速度，坐标变换在handler_node用实时航向角完成
        // 不再在这里做坐标变换，因为TF有延迟，陀螺旋转时会导致方向错误
        // 直接使用map坐标系速度
        double vx_base_raw = vx_map_cmd;
        double vy_base_raw = vy_map_cmd;

        // 8. 应用速度和加速度限制
        double vx_out = std::clamp(vx_base_raw, -max_vx_, max_vx_);
        double vy_out = std::clamp(vy_base_raw, -max_vy_, max_vy_);
        applyFluctuateApproachSpeedLimit(vx_out, vy_out);
        applyNarrowPassageSpeedLimit(vx_out, vy_out);

        // 加速度限制
        const double max_dv = cmd_accel_limit_linear_ * dt;
        // const double max_dw = cmd_accel_limit_angular_ * dt; // 没有使用 max_dw
        vx_out = std::clamp(vx_out, last_cmd_vx_ - max_dv, last_cmd_vx_ + max_dv);
        vy_out = std::clamp(vy_out, last_cmd_vy_ - max_dv, last_cmd_vy_ + max_dv);

        // 9. 发布指令并更新仿真（现在发送的是map坐标系速度）
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx_out;
        cmd.linear.y = vy_out;

        // angular都是特殊设置,作为当前角度和目标角度的容器
        cmd.angular.x = current_yaw;
        cmd.angular.z = target_yaw;
        cmd_pub_->publish(cmd);

        last_cmd_vx_ = cmd.linear.x;
        last_cmd_vy_ = cmd.linear.y;
        last_cmd_wz_ = cmd.angular.z;

        if (simulate_) {
            simulator_.integrateBodyCommand({ cmd.linear.x, cmd.linear.y, SIM_OMEGA }, dt);
            publishSimulatedTransform(now);
        }
    }

    void applyFluctuateApproachSpeedLimit(double& vx, double& vy) {
        const double distance = distance_to_fluctuate_region_;
        if (current_region_ == fluctuate_region_value_ ||
            distance < 0.0 ||
            distance <= fluctuate_release_distance_ ||
            !std::isfinite(distance) ||
            fluctuate_decel_distance_ <= 0.0)
        {
            return;
        }

        const double speed = std::hypot(vx, vy);
        if (speed < 1e-6) {
            return;
        }

        double speed_limit = speed;
        if (distance <= fluctuate_prepare_distance_) {
            speed_limit = fluctuate_min_approach_speed_;
        } else if (distance < fluctuate_decel_distance_) {
            const double ratio =
                (distance - fluctuate_prepare_distance_) /
                std::max(1e-6, fluctuate_decel_distance_ - fluctuate_prepare_distance_);
            speed_limit =
                fluctuate_min_approach_speed_ +
                std::clamp(ratio, 0.0, 1.0) *
                    (fluctuate_prepare_max_speed_ - fluctuate_min_approach_speed_);
        }

        speed_limit = std::clamp(speed_limit, 0.0, speed);
        if (speed_limit < speed) {
            const double scale = speed_limit / speed;
            vx *= scale;
            vy *= scale;
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                500,
                "Slowing before fluctuate: dist=%.2f speed %.2f -> %.2f",
                distance,
                speed,
                speed_limit
            );
        }
    }

    // Escape timer callback: handles detection and motion during escape
    void onEscapeTimer() {
        const rclcpp::Time now = get_clock()->now();

        // 1. 获取机器人当前位姿
        double current_x, current_y, current_yaw;
        if (!getCurrentPose(current_x, current_y, current_yaw)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Could not get current robot pose."
            );
            if (in_escape_mode_)
                publishZeroTwist();
            return;
        }

        // 2. ESCAPE 模式检测与进入条件
        int8_t current_cost = -1;
        const bool have_current_cost =
            has_costmap_ && getCostAt(current_x, current_y, current_cost);
        const bool at_target_cost = have_current_cost && current_cost >= 0
            && static_cast<int>(current_cost) <= escape_target_cost_threshold_;

        bool enter_blocked = false;
        if (have_current_cost) {
            if (current_cost < 0)
                enter_blocked = escape_treat_unknown_as_lethal_;
            else
                enter_blocked = static_cast<int>(current_cost) >= escape_enter_cost_threshold_;
            // RCLCPP_INFO_THROTTLE(
            //     get_logger(),
            //     *get_clock(),
            //     2000,
            //     "Current cost: %d, enter_blocked: %d",
            //     current_cost,
            //     enter_blocked
            // );
        }

        if (!in_escape_mode_ && enter_blocked) {
            if (!blocked_timing_active_) {
                blocked_timing_active_ = true;
                blocked_since_ = now;
                RCLCPP_INFO(
                    get_logger(),
                    "Blocked detected (cost %d), waiting %.1fs before escape...",
                    static_cast<int>(current_cost),
                    escape_enter_delay_s_
                );
            }
            const double blocked_duration = (now - blocked_since_).seconds();
            if (blocked_duration >= escape_enter_delay_s_) {
                int cgx = 0, cgy = 0;
                if (worldToGrid(current_x, current_y, cgx, cgy)) {
                    int tgx = 0, tgy = 0;
                    if (findNearestFreeCell(cgx, cgy, tgx, tgy)) {
                        double tx = 0.0, ty = 0.0;
                        if (gridToWorld(tgx, tgy, tx, ty)) {
                            escape_target_x_m_ = tx;
                            escape_target_y_m_ = ty;
                            in_escape_mode_ = true;
                            blocked_timing_active_ = false;
                            RCLCPP_WARN(
                                get_logger(),
                                "Entering ESCAPE mode after %.1fs blocked. Target: (%.3f, %.3f) from cost %d",
                                blocked_duration,
                                tx,
                                ty,
                                static_cast<int>(current_cost)
                            );
                        }
                    }
                }
            } else {
                RCLCPP_INFO_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    500,
                    "Blocked for %.1fs / %.1fs before escape triggers",
                    blocked_duration,
                    escape_enter_delay_s_
                );
            }
        } else if (!in_escape_mode_ && !enter_blocked) {
            if (blocked_timing_active_) {
                RCLCPP_INFO(get_logger(), "Left blocked zone, resetting escape delay timer.");
                blocked_timing_active_ = false;
            }
        }

        // If not in escape mode, nothing to do here
        if (!in_escape_mode_) {
            return;
        }

        // 3. ESCAPE 运行逻辑与退出条件
        if (at_target_cost) {
            in_escape_mode_ = false;
            integral_x_ = integral_y_ = 0.0;
            has_prev_time_ = false;
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Exited ESCAPE mode (reached cost <= %d).",
                escape_target_cost_threshold_
            );
            return; // let normal control resume on next cycle
        }

        double dx = escape_target_x_m_ - current_x;
        double dy = escape_target_y_m_ - current_y;
        double dist = std::hypot(dx, dy);
        if (dist < escape_goal_tolerance_) {
            int cgx = 0, cgy = 0;
            if (worldToGrid(current_x, current_y, cgx, cgy)) {
                int tgx = 0, tgy = 0;
                if (findNearestFreeCell(cgx, cgy, tgx, tgy)
                    && gridToWorld(tgx, tgy, escape_target_x_m_, escape_target_y_m_))
                {
                    RCLCPP_INFO_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        2000,
                        "ESCAPE re-target to (%.3f, %.3f)",
                        escape_target_x_m_,
                        escape_target_y_m_
                    );
                    dx = escape_target_x_m_ - current_x;
                    dy = escape_target_y_m_ - current_y;
                    dist = std::hypot(dx, dy);
                } else {
                    publishZeroTwist();
                    return;
                }
            }
        }

        if (!has_prev_time_) {
            prev_time_ = now;
            has_prev_time_ = true;
            return;
        }
        double dt = (now - prev_time_).seconds();
        prev_time_ = now;
        if (dt <= 0.0)
            return;
        dt = std::min(dt, max_dt_);

        double vx_map_cmd = 0.0, vy_map_cmd = 0.0;
        if (dist > 1e-6) {
            vx_map_cmd = escape_speed_ * dx / dist;
            vy_map_cmd = escape_speed_ * dy / dist;
        }

        // 【陀螺模式修复】直接发送map坐标系速度，坐标变换在handler_node完成
        // const double cos_yaw = std::cos(current_yaw);
        // const double sin_yaw = std::sin(current_yaw);
        // const double ux_b    = cos_yaw * vx_map_cmd + sin_yaw * vy_map_cmd;
        // const double uy_b    = -sin_yaw * vx_map_cmd + cos_yaw * vy_map_cmd;
        const double ux_b = vx_map_cmd;
        const double uy_b = vy_map_cmd;

        const double cmd_vx_raw = std::clamp(ux_b, -max_vx_, max_vx_);
        const double cmd_vy_raw = std::clamp(uy_b, -max_vy_, max_vy_);

        const double max_dv = cmd_accel_limit_linear_ * dt;
        double vx_out = std::clamp(cmd_vx_raw, last_cmd_vx_ - max_dv, last_cmd_vx_ + max_dv);
        double vy_out = std::clamp(cmd_vy_raw, last_cmd_vy_ - max_dv, last_cmd_vy_ + max_dv);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx_out;
        cmd.linear.y = vy_out;
        cmd.angular.x = current_yaw;
        cmd.angular.z = std::atan2(dy, dx);
        cmd_pub_->publish(cmd);

        last_cmd_vx_ = cmd.linear.x;
        last_cmd_vy_ = cmd.linear.y;
        last_cmd_wz_ = cmd.angular.z;

        if (simulate_) {
            simulator_.integrateBodyCommand({ cmd.linear.x, cmd.linear.y, SIM_OMEGA }, dt);
            publishSimulatedTransform(now);
        }
    }

    // --- 辅助函数 ---

    bool getCurrentPose(double& x, double& y, double& yaw) {
        if (simulate_) {
            const auto sim_pose = simulator_.getPose();
            x = sim_pose.x;
            y = sim_pose.y;
            yaw = sim_pose.yaw;
            return true;
        }

        try {
            geometry_msgs::msg::TransformStamped tf_map_to_base =
                tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            x = tf_map_to_base.transform.translation.x;
            y = tf_map_to_base.transform.translation.y;

            tf2::Quaternion q;
            tf2::fromMsg(tf_map_to_base.transform.rotation, q);
            double roll, pitch;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            return true;

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "TF lookup failed: %s",
                ex.what()
            );
            return false;
        }
    }

    // ---- Escape helpers ----
    bool worldToGrid(double x, double y, int& gx, int& gy) const {
        if (!has_costmap_)
            return false;
        const auto& info = last_costmap_.info;
        const double rel_x = x - info.origin.position.x;
        const double rel_y = y - info.origin.position.y;
        if (info.resolution <= 0.0)
            return false;
        gx = static_cast<int>(std::floor(rel_x / info.resolution));
        gy = static_cast<int>(std::floor(rel_y / info.resolution));
        if (gx < 0 || gy < 0 || gx >= static_cast<int>(info.width)
            || gy >= static_cast<int>(info.height))
            return false;
        return true;
    }

    bool gridToWorld(int gx, int gy, double& x, double& y) const {
        if (!has_costmap_)
            return false;
        const auto& info = last_costmap_.info;
        if (gx < 0 || gy < 0 || gx >= static_cast<int>(info.width)
            || gy >= static_cast<int>(info.height))
            return false;
        x = info.origin.position.x + (static_cast<double>(gx) + 0.5) * info.resolution;
        y = info.origin.position.y + (static_cast<double>(gy) + 0.5) * info.resolution;
        return true;
    }

    bool getCostAt(double x, double y, int8_t& out_cost) const {
        int gx, gy;
        if (!worldToGrid(x, y, gx, gy))
            return false;
        const size_t idx =
            static_cast<size_t>(gy) * last_costmap_.info.width + static_cast<size_t>(gx);
        if (idx >= last_costmap_.data.size())
            return false;
        out_cost = last_costmap_.data[idx];
        return true;
    }

    bool findNearestFreeCell(int start_gx, int start_gy, int& out_gx, int& out_gy) const {
        if (!has_costmap_)
            return false;
        const auto& info = last_costmap_.info;
        const int width = static_cast<int>(info.width);
        const int height = static_cast<int>(info.height);
        if (width <= 0 || height <= 0)
            return false;

        const int total = width * height;
        std::vector<uint8_t> visited(static_cast<size_t>(total), 0);
        std::queue<std::pair<int, int>> q;
        std::queue<int> depth_q;
        auto idx_of = [width](int x, int y) { return y * width + x; };

        auto can_step = [&](int x, int y) {
            if (x < 0 || y < 0 || x >= width || y >= height)
                return false;
            const int idx = idx_of(x, y);
            const int8_t v = last_costmap_.data[static_cast<size_t>(idx)];
            if (v < 0)
                return !escape_treat_unknown_as_lethal_;
            return static_cast<int>(v) < escape_lethal_threshold_;
        };

        auto push_if_valid = [&](int x, int y, int d) {
            if (x < 0 || y < 0 || x >= width || y >= height)
                return;
            const int idx = idx_of(x, y);
            if (visited[static_cast<size_t>(idx)])
                return;
            visited[static_cast<size_t>(idx)] = 1;
            q.emplace(x, y);
            depth_q.emplace(d);
        };

        push_if_valid(start_gx, start_gy, 0);

        const int dx8[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
        const int dy8[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

        while (!q.empty()) {
            auto [cx, cy] = q.front();
            q.pop();
            int cur_d = depth_q.front();
            depth_q.pop();

            const int cidx = idx_of(cx, cy);
            if (cidx < 0 || cidx >= total)
                continue;
            const int8_t val = last_costmap_.data[static_cast<size_t>(cidx)];
            if (val >= 0 && static_cast<int>(val) <= escape_target_cost_threshold_) {
                out_gx = cx;
                out_gy = cy;
                return true;
            }
            if (cur_d >= escape_max_radius_cells_)
                continue;
            for (int k = 0; k < 8; ++k) {
                const int nx = cx + dx8[k];
                const int ny = cy + dy8[k];
                if (can_step(nx, ny))
                    push_if_valid(nx, ny, cur_d + 1);
            }
        }
        return false;
    }

    bool isBlockedForNarrow(double x, double y) const {
        int gx = 0;
        int gy = 0;
        if (!worldToGrid(x, y, gx, gy)) {
            return true;
        }

        const size_t idx =
            static_cast<size_t>(gy) * last_costmap_.info.width + static_cast<size_t>(gx);
        if (idx >= last_costmap_.data.size()) {
            return true;
        }

        const int8_t cost = last_costmap_.data[idx];
        if (cost < 0) {
            return narrow_treat_unknown_as_blocked_;
        }
        return static_cast<int>(cost) >= narrow_obstacle_cost_threshold_;
    }

    double scanFreeDistanceForNarrow(
        double x,
        double y,
        double nx,
        double ny,
        double direction
    ) const {
        const double step = std::max(0.01, narrow_scan_step_);
        const double max_dist = std::max(step, narrow_scan_max_side_distance_);
        for (double d = step; d <= max_dist; d += step) {
            const double sx = x + direction * nx * d;
            const double sy = y + direction * ny * d;
            if (isBlockedForNarrow(sx, sy)) {
                return d;
            }
        }
        return max_dist;
    }

    double measurePassageWidth(double x, double y, double path_yaw) const {
        if (isBlockedForNarrow(x, y)) {
            return 0.0;
        }
        const double nx = -std::sin(path_yaw);
        const double ny = std::cos(path_yaw);
        const double left = scanFreeDistanceForNarrow(x, y, nx, ny, 1.0);
        const double right = scanFreeDistanceForNarrow(x, y, nx, ny, -1.0);
        return left + right;
    }

    bool findClosestPathIndex(double rob_x, double rob_y, std::size_t& closest_idx) const {
        if (last_path_.poses.empty()) {
            return false;
        }

        closest_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < last_path_.poses.size(); ++i) {
            const auto& p = last_path_.poses[i].pose.position;
            const double dx = p.x - rob_x;
            const double dy = p.y - rob_y;
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }
        return true;
    }

    bool updateNarrowPassageState(double rob_x, double rob_y) {
        if (!narrow_detection_enabled_ || !has_costmap_ || last_path_.poses.size() < 2) {
            if (narrow_passage_active_) {
                narrow_passage_active_ = false;
                narrow_passage_width_m_ = -1.0;
                narrow_path_yaw_ = std::numeric_limits<double>::quiet_NaN();
                publishNarrowPassageState();
            }
            return narrow_passage_active_;
        }

        std::size_t closest_idx = 0;
        if (!findClosestPathIndex(rob_x, rob_y, closest_idx)) {
            return narrow_passage_active_;
        }

        const bool was_active = narrow_passage_active_;
        const double width_threshold = was_active ? narrow_exit_width_ : narrow_enter_width_;
        const double sample_step = std::max(0.02, narrow_sample_step_);
        const double lookahead = std::max(sample_step, narrow_lookahead_distance_);
        const double required_length = std::max(sample_step, narrow_min_length_);

        double min_width = std::numeric_limits<double>::infinity();
        double consecutive_narrow_length = 0.0;
        double next_sample_s = 0.0;
        double traveled_s = 0.0;
        bool detected = false;
        double detected_yaw = std::numeric_limits<double>::quiet_NaN();

        for (std::size_t i = closest_idx; i + 1 < last_path_.poses.size() && traveled_s <= lookahead;
             ++i)
        {
            const auto& p0 = last_path_.poses[i].pose.position;
            const auto& p1 = last_path_.poses[i + 1].pose.position;
            const double dx = p1.x - p0.x;
            const double dy = p1.y - p0.y;
            const double seg_len = std::hypot(dx, dy);
            if (seg_len < 1e-6) {
                continue;
            }

            const double seg_yaw = std::atan2(dy, dx);
            while (next_sample_s <= traveled_s + seg_len && next_sample_s <= lookahead) {
                const double ratio = std::clamp((next_sample_s - traveled_s) / seg_len, 0.0, 1.0);
                const double sx = p0.x + ratio * dx;
                const double sy = p0.y + ratio * dy;
                const double width = measurePassageWidth(sx, sy, seg_yaw);
                min_width = std::min(min_width, width);

                if (width < width_threshold) {
                    consecutive_narrow_length += sample_step;
                    if (!std::isfinite(detected_yaw)) {
                        detected_yaw = seg_yaw;
                    }
                    if (consecutive_narrow_length >= required_length) {
                        detected = true;
                        break;
                    }
                } else {
                    consecutive_narrow_length = 0.0;
                    detected_yaw = std::numeric_limits<double>::quiet_NaN();
                }

                next_sample_s += sample_step;
            }

            if (detected) {
                break;
            }
            traveled_s += seg_len;
        }

        narrow_passage_width_m_ =
            std::isfinite(min_width) ? min_width : -1.0;
        narrow_passage_active_ = detected;
        narrow_path_yaw_ = detected && std::isfinite(detected_yaw)
            ? detected_yaw
            : std::numeric_limits<double>::quiet_NaN();

        publishNarrowPassageState();

        if (narrow_passage_active_ != was_active) {
            RCLCPP_WARN(
                get_logger(),
                "%s narrow passage: min_width=%.2f m, yaw=%.2f rad, threshold=%.2f m",
                narrow_passage_active_ ? "Entering" : "Leaving",
                narrow_passage_width_m_,
                std::isfinite(narrow_path_yaw_) ? narrow_path_yaw_ : 0.0,
                width_threshold
            );
        } else {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Narrow passage state=%s min_width=%.2f m threshold=%.2f m",
                narrow_passage_active_ ? "true" : "false",
                narrow_passage_width_m_,
                width_threshold
            );
        }

        return narrow_passage_active_;
    }

    void publishNarrowPassageState() {
        if (narrow_passage_pub_) {
            std_msgs::msg::Bool msg;
            msg.data = narrow_passage_active_;
            narrow_passage_pub_->publish(msg);
        }
        if (narrow_passage_width_pub_) {
            std_msgs::msg::Float32 msg;
            msg.data = static_cast<float>(narrow_passage_width_m_);
            narrow_passage_width_pub_->publish(msg);
        }
    }

    void applyNarrowPassageSpeedLimit(double& vx, double& vy) {
        if (!narrow_passage_active_ || narrow_speed_limit_ <= 0.0) {
            return;
        }

        const double speed = std::hypot(vx, vy);
        if (speed <= narrow_speed_limit_ || speed < 1e-6) {
            return;
        }

        const double scale = narrow_speed_limit_ / speed;
        vx *= scale;
        vy *= scale;
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "Narrow passage speed limit: %.2f -> %.2f m/s",
            speed,
            narrow_speed_limit_
        );
    }

    bool findLookaheadPoint(
        double rob_x,
        double rob_y,
        geometry_msgs::msg::Point& pt_ahead,
        geometry_msgs::msg::Point& pt_closest,
        double& path_yaw,
        bool& is_last
    ) {
        if (last_path_.poses.empty())
            return false;

        is_last = false;

        // 找到路径上离机器人最近的点的索引
        size_t closest_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max();
        for (size_t i = 0; i < last_path_.poses.size(); ++i) {
            double dx = last_path_.poses[i].pose.position.x - rob_x;
            double dy = last_path_.poses[i].pose.position.y - rob_y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_idx = i;
                pt_closest = last_path_.poses[i].pose.position; // 输出最近点
            }
        }

        // *** 核心修改: 计算动态前瞻距离 ***
        const double current_lookahead_distance = computeDynamicLookaheadDistance(closest_idx);

        // 从最近点开始向前搜索，直到找到一个距离机器人大于前瞻距离的点
        for (size_t i = closest_idx; i < last_path_.poses.size(); ++i) {
            double dx = last_path_.poses[i].pose.position.x - rob_x;
            double dy = last_path_.poses[i].pose.position.y - rob_y;
            if (std::hypot(dx, dy) > current_lookahead_distance) {
                pt_ahead = last_path_.poses[i].pose.position;
                path_yaw = computePathYawAtIndex(i);
                return true;
            }
        }

        // 如果遍历完路径都找不到，说明机器人已经接近终点，直接返回路径的最后一个点
        pt_ahead = last_path_.poses.back().pose.position;
        path_yaw = computePathYawAtIndex(last_path_.poses.size() - 1);
        is_last = true;
        return true;
    }

    double computePathYawAtIndex(std::size_t index) const {
        if (last_path_.poses.size() < 2) {
            return 0.0;
        }

        index = std::min(index, last_path_.poses.size() - 1);

        for (std::size_t next = index + 1; next < last_path_.poses.size(); ++next) {
            const auto& p0 = last_path_.poses[index].pose.position;
            const auto& p1 = last_path_.poses[next].pose.position;
            const double dx = p1.x - p0.x;
            const double dy = p1.y - p0.y;
            if (std::hypot(dx, dy) > 1e-4) {
                return std::atan2(dy, dx);
            }
        }

        for (std::size_t prev = index; prev > 0; --prev) {
            const auto& p0 = last_path_.poses[prev - 1].pose.position;
            const auto& p1 = last_path_.poses[index].pose.position;
            const double dx = p1.x - p0.x;
            const double dy = p1.y - p0.y;
            if (std::hypot(dx, dy) > 1e-4) {
                return std::atan2(dy, dx);
            }
        }

        return 0.0;
    }

    double filterPathYaw(double raw_yaw, double dt) {
        raw_yaw = normalizeAngle(raw_yaw);
        if (!has_filtered_path_yaw_) {
            filtered_path_yaw_ = raw_yaw;
            has_filtered_path_yaw_ = true;
            return filtered_path_yaw_;
        }

        double yaw_error = normalizeAngle(raw_yaw - filtered_path_yaw_);
        if (path_yaw_max_rate_ > 0.0 && dt > 0.0) {
            const double max_step = path_yaw_max_rate_ * dt;
            yaw_error = std::clamp(yaw_error, -max_step, max_step);
        }

        if (path_yaw_filter_tau_ > 1e-6 && dt > 0.0) {
            const double alpha = std::clamp(dt / (path_yaw_filter_tau_ + dt), 0.0, 1.0);
            yaw_error *= alpha;
        }

        filtered_path_yaw_ = normalizeAngle(filtered_path_yaw_ + yaw_error);
        return filtered_path_yaw_;
    }

    // *** 新增函数: 根据路径曲率计算动态前瞻距离 ***
    double computeDynamicLookaheadDistance(std::size_t start_index) const {
        if (!enable_dynamic_lookahead_) {
            return lookahead_distance_; // 返回固定的前瞻距离
        }
        if (last_path_.poses.size() < 3) {
            return min_lookahead_distance_; // 路径太短，使用最小前瞻距离
        }

        double accumulated_length_m = 0.0;
        double sum_abs_curvature = 0.0;
        int curvature_samples = 0;

        std::size_t i = start_index;
        // 在一个窗口内计算平均曲率
        while (i + 2 < last_path_.poses.size() && accumulated_length_m < curvature_window_distance_)
        {
            const auto& p0 = last_path_.poses[i].pose.position;
            const auto& p1 = last_path_.poses[i + 1].pose.position;
            const auto& p2 = last_path_.poses[i + 2].pose.position;

            const double dx01 = p1.x - p0.x;
            const double dy01 = p1.y - p0.y;
            const double dx12 = p2.x - p1.x;
            const double dy12 = p2.y - p1.y;
            const double len01 = std::hypot(dx01, dy01);
            const double len12 = std::hypot(dx12, dy12);

            if (len01 > 1e-6 && len12 > 1e-6) {
                const double th0 = std::atan2(dy01, dx01);
                const double th1 = std::atan2(dy12, dx12);
                double dth = normalizeAngle(th1 - th0);
                const double seg_len = len12;
                const double kappa =
                    std::fabs(dth) / std::max(seg_len, 1e-6); // 曲率 = d_theta / d_length
                sum_abs_curvature += kappa;
                curvature_samples += 1;
            }
            accumulated_length_m += len12;
            i += 1;
        }

        const double mean_abs_curvature = (curvature_samples > 0)
            ? (sum_abs_curvature / static_cast<double>(curvature_samples))
            : 0.0;

        // 根据平均曲率线性插值计算前瞻距离
        // 曲率越大，前瞻距离越小
        double alpha = 0.0;
        if (mean_abs_curvature <= curvature_low_) {
            alpha = 0.0; // 低曲率，使用最大前瞻距离
        } else if (mean_abs_curvature >= curvature_high_) {
            alpha = 1.0; // 高曲率，使用最小前瞻距离
        } else {
            // 在中间区域线性插值
            alpha = (mean_abs_curvature - curvature_low_)
                / std::max(1e-9, (curvature_high_ - curvature_low_));
        }

        const double lookahead =
            max_lookahead_distance_ - alpha * (max_lookahead_distance_ - min_lookahead_distance_);

        return std::clamp(lookahead, min_lookahead_distance_, max_lookahead_distance_);
    }

    // 过弯限速：预瞄前方窗口内相对当前切线的最大转角 Φ，按甩出量模型
    // d ≈ v^2*sin(Φ/2)/a 反解安全速度 v=sqrt(a*ε/sin(Φ/2))。直线 Φ≈0 → 全速。
    double computeCornerSpeedLimit(std::size_t start_index) const {
        if (!enable_corner_slowdown_ || last_path_.poses.size() < 3) {
            return cruise_speed_;
        }
        const double yaw_start = computePathYawAtIndex(start_index);
        double accumulated = 0.0;
        double max_turn = 0.0;
        for (std::size_t i = start_index; i + 1 < last_path_.poses.size(); ++i) {
            const auto& p0 = last_path_.poses[i].pose.position;
            const auto& p1 = last_path_.poses[i + 1].pose.position;
            accumulated += std::hypot(p1.x - p0.x, p1.y - p0.y);
            if (accumulated > corner_lookahead_distance_) {
                break;
            }
            const double turn = std::fabs(normalizeAngle(computePathYawAtIndex(i) - yaw_start));
            max_turn = std::max(max_turn, turn);
        }

        const double s = std::sin(0.5 * max_turn);
        if (s < 1e-6) {
            return cruise_speed_; // 直线，不减速
        }
        const double a = std::max(cmd_accel_limit_linear_, 1e-3);
        const double v_safe = std::sqrt(a * corner_cut_tolerance_ / s);
        return std::clamp(v_safe, corner_min_speed_, cruise_speed_);
    }

    void publishZeroTwist() {
        geometry_msgs::msg::Twist zero_twist;
        cmd_pub_->publish(zero_twist);
        last_cmd_vx_ = 0.0;
        last_cmd_vy_ = 0.0;
        last_cmd_wz_ = 0.0;
    }

    void resetControllerState() {
        integral_x_ = 0.0;
        integral_y_ = 0.0;
        prev_ex_ = 0.0;
        prev_ey_ = 0.0;
        has_prev_time_ = false;
        has_prev_yaw_ = false;
        prev_yaw_ = 0.0;
        has_filtered_path_yaw_ = false;
        filtered_path_yaw_ = 0.0;
    }

    void publishSimulatedTransform(const rclcpp::Time& now) {
        const auto s = simulator_.getPose();
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = map_frame_;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform.translation.x = s.x;
        tf_msg.transform.translation.y = s.y;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, s.yaw);
        tf_msg.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(tf_msg);
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // ROS 接口
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fluctuate_distance_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr region_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr narrow_passage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr narrow_passage_width_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr escape_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 路径和状态
    nav_msgs::msg::Path last_path_;
    bool has_path_ { false };

    // 参数
    std::string map_frame_, base_frame_;
    double goal_tolerance_;
    double max_dt_;
    double kp_xy_, ki_xy_, kd_xy_, kp_yaw_;
    double max_vx_, max_vy_, max_wz_;
    double cmd_accel_limit_linear_, cmd_accel_limit_angular_;
    double fluctuate_decel_distance_ { 2.0 };
    double fluctuate_prepare_distance_ { 1.1 };
    double fluctuate_prepare_max_speed_ { 0.35 };
    double fluctuate_min_approach_speed_ { 0.22 };
    double fluctuate_release_distance_ { 0.05 };
    double distance_to_fluctuate_region_ { -1.0 };
    uint8_t fluctuate_region_value_ { 5 };
    uint8_t current_region_ { 1 };
    double prediction_time_factor_;

    // 新增: 动态前瞻距离参数
    bool enable_dynamic_lookahead_;
    double lookahead_distance_; // 固定值 (备用)
    double min_lookahead_distance_, max_lookahead_distance_;
    double curvature_window_distance_;
    double curvature_low_, curvature_high_;
    double path_yaw_filter_tau_ { 0.20 };
    double path_yaw_max_rate_ { 3.0 };
    bool has_filtered_path_yaw_ { false };
    double filtered_path_yaw_ { 0.0 };

    // 路径跟随律 (沿切线前馈 + 横向纠偏) 与终点位置闭环减速
    bool use_path_following_ { true };
    double cruise_speed_ { 1.0 };
    double kp_cross_ { 1.5 };
    double approach_distance_ { 0.7 };
    double approach_gain_ { 1.5 };
    double brake_latency_ { 0.1 };
    double brake_stop_margin_ { 0.15 };

    // 过弯限速：预瞄累计转角反解安全速度，直线全速、弯道压速
    bool enable_corner_slowdown_ { true };
    double corner_lookahead_distance_ { 1.0 };
    double corner_cut_tolerance_ { 0.15 };
    double corner_min_speed_ { 0.3 };

    // Narrow passage detection and driving mode
    bool narrow_detection_enabled_ { true };
    bool narrow_treat_unknown_as_blocked_ { true };
    bool narrow_passage_active_ { false };
    int narrow_obstacle_cost_threshold_ { 99 };
    double narrow_lookahead_distance_ { 1.5 };
    double narrow_sample_step_ { 0.10 };
    double narrow_scan_step_ { 0.03 };
    double narrow_scan_max_side_distance_ { 1.2 };
    double narrow_enter_width_ { 0.90 };
    double narrow_exit_width_ { 1.10 };
    double narrow_min_length_ { 0.40 };
    double narrow_speed_limit_ { 0.35 };
    double narrow_passage_width_m_ { -1.0 };
    double narrow_path_yaw_ { std::numeric_limits<double>::quiet_NaN() };

    // PID 控制器状态
    bool has_prev_time_ { false };
    rclcpp::Time prev_time_;
    double integral_x_ { 0.0 }, integral_y_ { 0.0 };
    double prev_ex_ { 0.0 }, prev_ey_ { 0.0 };
    double last_cmd_vx_ { 0.0 }, last_cmd_vy_ { 0.0 }, last_cmd_wz_ { 0.0 };

    // --- 新增: 用于航向角预测 ---
    bool has_prev_yaw_ { false };
    double prev_yaw_ { 0.0 };

    // 仿真
    bool simulate_ { false };
    gvc::Simulator2D simulator_ {};

    // Costmap and escape state
    nav_msgs::msg::OccupancyGrid last_costmap_;
    bool has_costmap_ { false };
    bool in_escape_mode_ { false };
    int escape_free_cost_value_ { 0 };
    int escape_target_cost_threshold_ { 0 };
    int escape_enter_cost_threshold_ { 1 };
    int escape_lethal_threshold_ { 100 };
    bool escape_treat_unknown_as_lethal_ { true };
    double escape_speed_ { 0.4 };
    double escape_goal_tolerance_ { 0.05 };
    int escape_max_radius_cells_ { 200 };
    double escape_enter_delay_s_ { 2.0 };
    bool blocked_timing_active_ { false };
    rclcpp::Time blocked_since_;
    double escape_target_x_m_ { 0.0 };
    double escape_target_y_m_ { 0.0 };
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplifiedControllerNode>());
    rclcpp::shutdown();
    return 0;
}
