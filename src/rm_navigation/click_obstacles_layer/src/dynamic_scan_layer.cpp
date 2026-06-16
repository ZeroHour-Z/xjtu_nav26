#include "click_obstacles_layer/dynamic_scan_layer.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace click_obstacles_layer {

DynamicScanLayer::DynamicScanLayer() {
    enabled_ = true;
}

void DynamicScanLayer::onInitialize() {
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("topic", rclcpp::ParameterValue(std::string("/traversability/scan")));
    declareParameter("obstacle_radius", rclcpp::ParameterValue(0.35));
    declareParameter("max_obstacle_age", rclcpp::ParameterValue(0.6));
    declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));
    declareParameter("min_range", rclcpp::ParameterValue(0.0));
    declareParameter("max_range", rclcpp::ParameterValue(0.0));

    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("DynamicScanLayer: node expired");
    }

    (void)node->get_parameter("enabled", enabled_);
    (void)node->get_parameter("topic", topic_);
    (void)node->get_parameter("obstacle_radius", obstacle_radius_);
    (void)node->get_parameter("max_obstacle_age", max_obstacle_age_);
    (void)node->get_parameter("transform_tolerance", transform_tolerance_);
    (void)node->get_parameter("min_range", min_range_);
    (void)node->get_parameter("max_range", max_range_);

    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&DynamicScanLayer::onScan, this, std::placeholders::_1)
    );

    current_ = true;
}

void DynamicScanLayer::activate() {}
void DynamicScanLayer::deactivate() {}

void DynamicScanLayer::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    clearing_points_ = points_;
    points_.clear();
}

void DynamicScanLayer::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!enabled_ || msg->header.frame_id.empty()) {
        return;
    }

    const std::string global_frame = layered_costmap_->getGlobalFrameID();
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_->lookupTransform(
            global_frame,
            msg->header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(transform_tolerance_)
        );
    } catch (const tf2::TransformException& ex) {
        auto node = node_.lock();
        if (node) {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                2000,
                "DynamicScanLayer failed to transform %s to %s: %s",
                msg->header.frame_id.c_str(),
                global_frame.c_str(),
                ex.what()
            );
        }
        return;
    }

    std::vector<DynamicScanPoint> points;
    points.reserve(msg->ranges.size());
    const double effective_min_range = min_range_ > 0.0 ? min_range_ : static_cast<double>(msg->range_min);
    const double effective_max_range = max_range_ > 0.0 ? max_range_ : static_cast<double>(msg->range_max);

    for (size_t index = 0; index < msg->ranges.size(); ++index) {
        const double range = static_cast<double>(msg->ranges[index]);
        if (!std::isfinite(range) || range < effective_min_range || range > effective_max_range) {
            continue;
        }

        const double angle = static_cast<double>(msg->angle_min) +
            static_cast<double>(index) * static_cast<double>(msg->angle_increment);

        geometry_msgs::msg::PointStamped source;
        source.header = msg->header;
        source.point.x = range * std::cos(angle);
        source.point.y = range * std::sin(angle);
        source.point.z = 0.0;

        geometry_msgs::msg::PointStamped target;
        tf2::doTransform(source, target, transform);
        points.push_back({ target.point.x, target.point.y });
    }

    auto node = node_.lock();
    const rclcpp::Time now = node ? node->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    {
        std::lock_guard<std::mutex> lk(mutex_);
        clearing_points_ = points_;
        points_ = std::move(points);
        last_scan_time_ = now;
    }
}

bool DynamicScanLayer::dataFreshLocked(const rclcpp::Time& now) const {
    if (points_.empty() || last_scan_time_.nanoseconds() == 0) {
        return false;
    }
    return (now - last_scan_time_).seconds() <= max_obstacle_age_;
}

void DynamicScanLayer::updateBounds(
    double /*origin_x*/,
    double /*origin_y*/,
    double /*origin_yaw*/,
    double* min_x,
    double* min_y,
    double* max_x,
    double* max_y
) {
    if (!enabled_) {
        return;
    }

    auto node = node_.lock();
    const rclcpp::Time now = node ? node->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    std::lock_guard<std::mutex> lk(mutex_);

    for (const auto& point: clearing_points_) {
        *min_x = std::min(*min_x, point.x - obstacle_radius_);
        *min_y = std::min(*min_y, point.y - obstacle_radius_);
        *max_x = std::max(*max_x, point.x + obstacle_radius_);
        *max_y = std::max(*max_y, point.y + obstacle_radius_);
    }

    const bool fresh = dataFreshLocked(now);
    if (!fresh && last_scan_time_.nanoseconds() == 0) {
        return;
    }

    for (const auto& point: points_) {
        *min_x = std::min(*min_x, point.x - obstacle_radius_);
        *min_y = std::min(*min_y, point.y - obstacle_radius_);
        *max_x = std::max(*max_x, point.x + obstacle_radius_);
        *max_y = std::max(*max_y, point.y + obstacle_radius_);
    }
}

void DynamicScanLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j
) {
    if (!enabled_) {
        return;
    }

    auto node = node_.lock();
    const rclcpp::Time now = node ? node->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    std::lock_guard<std::mutex> lk(mutex_);

    const bool fresh = dataFreshLocked(now);
    clearing_points_.clear();

    if (!fresh) {
        points_.clear();
        last_scan_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        return;
    }

    const double resolution = master_grid.getResolution();
    const int radius_cells = static_cast<int>(std::ceil(obstacle_radius_ / resolution));
    const int radius_cells_sq = radius_cells * radius_cells;

    for (const auto& point: points_) {
        unsigned int center_x = 0;
        unsigned int center_y = 0;
        if (!master_grid.worldToMap(point.x, point.y, center_x, center_y)) {
            continue;
        }

        int start_x = static_cast<int>(center_x) - radius_cells;
        int end_x = static_cast<int>(center_x) + radius_cells;
        int start_y = static_cast<int>(center_y) - radius_cells;
        int end_y = static_cast<int>(center_y) + radius_cells;

        start_x = std::max(start_x, min_i);
        start_y = std::max(start_y, min_j);
        end_x = std::min(end_x, max_i);
        end_y = std::min(end_y, max_j);

        for (int y = start_y; y < end_y; ++y) {
            for (int x = start_x; x < end_x; ++x) {
                const int dx = x - static_cast<int>(center_x);
                const int dy = y - static_cast<int>(center_y);
                if (dx * dx + dy * dy <= radius_cells_sq) {
                    master_grid.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
                }
            }
        }
    }
}

} // namespace click_obstacles_layer

PLUGINLIB_EXPORT_CLASS(click_obstacles_layer::DynamicScanLayer, nav2_costmap_2d::Layer)
