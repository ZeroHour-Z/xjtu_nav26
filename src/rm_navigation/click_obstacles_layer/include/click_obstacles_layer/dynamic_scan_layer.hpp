#pragma once

#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <vector>

namespace click_obstacles_layer {

struct DynamicScanPoint {
    double x;
    double y;
};

class DynamicScanLayer: public nav2_costmap_2d::Layer {
public:
    DynamicScanLayer();
    ~DynamicScanLayer() override = default;

    void onInitialize() override;
    void updateBounds(
        double origin_x,
        double origin_y,
        double origin_yaw,
        double* min_x,
        double* min_y,
        double* max_x,
        double* max_y
    ) override;
    void
    updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
        override;
    void reset() override;

    bool isClearable() override {
        return true;
    }

    void activate() override;
    void deactivate() override;

private:
    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    bool dataFreshLocked(const rclcpp::Time& now) const;

    bool enabled_ { true };
    std::string topic_ { "/traversability/scan" };
    double obstacle_radius_ { 0.35 };
    double max_obstacle_age_ { 0.6 };
    double transform_tolerance_ { 0.2 };
    double min_range_ { 0.0 };
    double max_range_ { 0.0 };

    std::mutex mutex_;
    std::vector<DynamicScanPoint> points_;
    std::vector<DynamicScanPoint> clearing_points_;
    rclcpp::Time last_scan_time_ { 0, 0, RCL_ROS_TIME };

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

} // namespace click_obstacles_layer
