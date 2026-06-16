#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace click_obstacles_layer {

struct ForbiddenRegion {
    std::string name;
    std::vector<std::pair<double, double>> polygon;
    double center_y { 0.0 };
    double min_x { 0.0 };
    double min_y { 0.0 };
    double max_x { 0.0 };
    double max_y { 0.0 };
    bool blocked { false };
};

class DirectionalForbiddenLayer: public nav2_costmap_2d::Layer {
public:
    DirectionalForbiddenLayer();
    ~DirectionalForbiddenLayer() override = default;

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
        return false;
    }

    void activate() override;
    void deactivate() override;

private:
    void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onPlan(const nav_msgs::msg::Path::SharedPtr msg);
    bool loadRegion(const std::string& param_name, const std::string& name);
    bool pathHasForbiddenTraversal(
        const nav_msgs::msg::Path& path,
        const ForbiddenRegion& region
    ) const;
    static bool isPointInPolygon(
        double x,
        double y,
        const std::vector<std::pair<double, double>>& polygon
    );

    bool enabled_ { true };
    double min_region_delta_y_ { 0.10 };
    std::string goal_topic_ { "/goal_pose" };
    std::string plan_topic_ { "/plan" };

    mutable std::mutex mutex_;
    std::vector<ForbiddenRegion> regions_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
};

} // namespace click_obstacles_layer
