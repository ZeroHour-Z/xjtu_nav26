#include "click_obstacles_layer/click_obstacles_layer.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace click_obstacles_layer {

ClickObstaclesLayer::ClickObstaclesLayer() {
    enabled_ = true;
}

void ClickObstaclesLayer::onInitialize() {
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("obstacle_radius", rclcpp::ParameterValue(0.20));

    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("ClickObstaclesLayer: node expired");
    }

    // Read initial parameters. nav2's declareParameter() registers them as
    // "<layer_name>.<param>", so they must be read back with the name_ prefix
    // (otherwise the YAML values are silently ignored).
    (void)node->get_parameter(name_ + ".enabled", enabled_);
    (void)node->get_parameter(name_ + ".obstacle_radius", obstacle_radius_);

    sub_clicked_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        10,
        std::bind(&ClickObstaclesLayer::onClickedPoint, this, std::placeholders::_1)
    );
    sub_clear_ = node->create_subscription<std_msgs::msg::Empty>(
        "/rviz_click_obstacles/clear",
        10,
        std::bind(&ClickObstaclesLayer::onClear, this, std::placeholders::_1)
    );
    sub_radius_ = node->create_subscription<std_msgs::msg::Float32>(
        "/rviz_click_obstacles/set_radius",
        10,
        std::bind(&ClickObstaclesLayer::onSetRadius, this, std::placeholders::_1)
    );
    sub_enable_ = node->create_subscription<std_msgs::msg::Bool>(
        "/rviz_click_obstacles/enable",
        10,
        std::bind(&ClickObstaclesLayer::onEnable, this, std::placeholders::_1)
    );

    current_ = true;
}

void ClickObstaclesLayer::activate() {}
void ClickObstaclesLayer::deactivate() {}

void ClickObstaclesLayer::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    obstacles_.clear();
}

void ClickObstaclesLayer::onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (!enabled_)
        return;
    ObstaclePoint p { msg->point.x, msg->point.y };
    {
        std::lock_guard<std::mutex> lk(mutex_);
        obstacles_.push_back(p);
    }
}

void ClickObstaclesLayer::onClear(const std_msgs::msg::Empty::SharedPtr) {
    std::lock_guard<std::mutex> lk(mutex_);
    obstacles_.clear();
}

void ClickObstaclesLayer::onSetRadius(const std_msgs::msg::Float32::SharedPtr msg) {
    obstacle_radius_ = static_cast<double>(msg->data);
}

void ClickObstaclesLayer::onEnable(const std_msgs::msg::Bool::SharedPtr msg) {
    enabled_ = msg->data;
}

void ClickObstaclesLayer::updateBounds(
    double /*origin_x*/,
    double /*origin_y*/,
    double /*origin_yaw*/,
    double* min_x,
    double* min_y,
    double* max_x,
    double* max_y
) {
    if (!enabled_)
        return;
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto& p: obstacles_) {
        *min_x = std::min(*min_x, p.x - obstacle_radius_);
        *min_y = std::min(*min_y, p.y - obstacle_radius_);
        *max_x = std::max(*max_x, p.x + obstacle_radius_);
        *max_y = std::max(*max_y, p.y + obstacle_radius_);
    }
}

void ClickObstaclesLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j
) {
    if (!enabled_)
        return;
    std::lock_guard<std::mutex> lk(mutex_);

    const double res = master_grid.getResolution();
    const int r_cells = static_cast<int>(std::ceil(obstacle_radius_ / res));
    for (const auto& p: obstacles_) {
        unsigned int cx, cy;
        if (!master_grid.worldToMap(p.x, p.y, cx, cy))
            continue;

        int i0 = static_cast<int>(cx) - r_cells;
        int i1 = static_cast<int>(cx) + r_cells;
        int j0 = static_cast<int>(cy) - r_cells;
        int j1 = static_cast<int>(cy) + r_cells;

        i0 = std::max(i0, min_i);
        j0 = std::max(j0, min_j);
        i1 = std::min(i1, max_i);
        j1 = std::min(j1, max_j);

        const int r2 = r_cells * r_cells;
        for (int j = j0; j < j1; ++j) {
            for (int i = i0; i < i1; ++i) {
                int dx = i - static_cast<int>(cx);
                int dy = j - static_cast<int>(cy);
                if (dx * dx + dy * dy <= r2) {
                    master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
                }
            }
        }
    }
}

} // namespace click_obstacles_layer

PLUGINLIB_EXPORT_CLASS(click_obstacles_layer::ClickObstaclesLayer, nav2_costmap_2d::Layer)