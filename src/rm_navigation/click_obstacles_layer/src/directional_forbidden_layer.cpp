#include "click_obstacles_layer/directional_forbidden_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

namespace click_obstacles_layer {

DirectionalForbiddenLayer::DirectionalForbiddenLayer() {
    enabled_ = true;
}

void DirectionalForbiddenLayer::onInitialize() {
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("goal_topic", rclcpp::ParameterValue(std::string("/goal_pose")));
    declareParameter("plan_topic", rclcpp::ParameterValue(std::string("/plan")));
    declareParameter("min_region_delta_y", rclcpp::ParameterValue(0.10));
    declareParameter("fluctuate_region_1", rclcpp::ParameterValue(std::vector<double> {}));
    declareParameter("fluctuate_region_3", rclcpp::ParameterValue(std::vector<double> {}));

    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("DirectionalForbiddenLayer: node expired");
    }

    (void)node->get_parameter(name_ + ".enabled", enabled_);
    (void)node->get_parameter(name_ + ".goal_topic", goal_topic_);
    (void)node->get_parameter(name_ + ".plan_topic", plan_topic_);
    (void)node->get_parameter(name_ + ".min_region_delta_y", min_region_delta_y_);

    regions_.clear();
    (void)loadRegion("fluctuate_region_1", "fluctuate_region_1");
    (void)loadRegion("fluctuate_region_3", "fluctuate_region_3");

    goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_,
        10,
        std::bind(&DirectionalForbiddenLayer::onGoal, this, std::placeholders::_1)
    );
    plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
        plan_topic_,
        10,
        std::bind(&DirectionalForbiddenLayer::onPlan, this, std::placeholders::_1)
    );

    current_ = true;
}

void DirectionalForbiddenLayer::activate() {}
void DirectionalForbiddenLayer::deactivate() {}
void DirectionalForbiddenLayer::reset() {}

void DirectionalForbiddenLayer::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    (void)msg;
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto& region: regions_) {
        region.blocked = false;
    }
}

void DirectionalForbiddenLayer::onPlan(const nav_msgs::msg::Path::SharedPtr msg) {
    auto node = node_.lock();
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto& region: regions_) {
        if (region.blocked || !pathHasForbiddenTraversal(*msg, region)) {
            continue;
        }

        region.blocked = true;
        if (node) {
            RCLCPP_WARN(
                node->get_logger(),
                "DirectionalForbiddenLayer: %s blocked because /plan traverses it with increasing |y|",
                region.name.c_str()
            );
        }
    }
}

bool DirectionalForbiddenLayer::loadRegion(const std::string& param_name, const std::string& name) {
    auto node = node_.lock();
    if (!node) {
        return false;
    }

    std::vector<double> data;
    (void)node->get_parameter(name_ + "." + param_name, data);
    if (data.size() < 6 || data.size() % 2 != 0) {
        RCLCPP_WARN(
            node->get_logger(),
            "DirectionalForbiddenLayer: invalid %s, need even size >= 6",
            param_name.c_str()
        );
        return false;
    }

    ForbiddenRegion region;
    region.name = name;
    region.min_x = std::numeric_limits<double>::max();
    region.min_y = std::numeric_limits<double>::max();
    region.max_x = std::numeric_limits<double>::lowest();
    region.max_y = std::numeric_limits<double>::lowest();

    double sum_y = 0.0;
    for (size_t i = 0; i < data.size(); i += 2) {
        const double x = data[i];
        const double y = data[i + 1];
        region.polygon.push_back({ x, y });
        sum_y += y;
        region.min_x = std::min(region.min_x, x);
        region.min_y = std::min(region.min_y, y);
        region.max_x = std::max(region.max_x, x);
        region.max_y = std::max(region.max_y, y);
    }
    region.center_y = sum_y / static_cast<double>(region.polygon.size());
    regions_.push_back(region);

    RCLCPP_INFO(
        node->get_logger(),
        "DirectionalForbiddenLayer: loaded %s with %zu vertices, center_y=%.2f",
        name.c_str(),
        region.polygon.size(),
        region.center_y
    );
    return true;
}

bool DirectionalForbiddenLayer::pathHasForbiddenTraversal(
    const nav_msgs::msg::Path& path,
    const ForbiddenRegion& region
) const {
    bool found = false;
    double first_y = 0.0;
    double last_y = 0.0;

    for (const auto& pose: path.poses) {
        const double x = pose.pose.position.x;
        const double y = pose.pose.position.y;
        if (!isPointInPolygon(x, y, region.polygon)) {
            continue;
        }

        if (!found) {
            first_y = y;
            found = true;
        }
        last_y = y;
    }

    if (!found) {
        return false;
    }

    const double side = (region.center_y >= 0.0) ? 1.0 : -1.0;
    return side * (last_y - first_y) > min_region_delta_y_;
}

void DirectionalForbiddenLayer::updateBounds(
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

    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto& region: regions_) {
        if (!region.blocked) {
            continue;
        }
        *min_x = std::min(*min_x, region.min_x);
        *min_y = std::min(*min_y, region.min_y);
        *max_x = std::max(*max_x, region.max_x);
        *max_y = std::max(*max_y, region.max_y);
    }
}

void DirectionalForbiddenLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j
) {
    if (!enabled_) {
        return;
    }

    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto& region: regions_) {
        if (!region.blocked) {
            continue;
        }

        unsigned int min_mx = 0, min_my = 0, max_mx = 0, max_my = 0;
        if (!master_grid.worldToMap(region.min_x, region.min_y, min_mx, min_my) ||
            !master_grid.worldToMap(region.max_x, region.max_y, max_mx, max_my)) {
            continue;
        }

        const int i0 = std::max(min_i, static_cast<int>(std::min(min_mx, max_mx)));
        const int j0 = std::max(min_j, static_cast<int>(std::min(min_my, max_my)));
        const int i1 = std::min(max_i, static_cast<int>(std::max(min_mx, max_mx)) + 1);
        const int j1 = std::min(max_j, static_cast<int>(std::max(min_my, max_my)) + 1);

        for (int j = j0; j < j1; ++j) {
            for (int i = i0; i < i1; ++i) {
                double wx = 0.0, wy = 0.0;
                master_grid.mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
                if (isPointInPolygon(wx, wy, region.polygon)) {
                    master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
                }
            }
        }
    }
}

bool DirectionalForbiddenLayer::isPointInPolygon(
    double x,
    double y,
    const std::vector<std::pair<double, double>>& polygon
) {
    bool inside = false;
    const size_t n = polygon.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = polygon[i].first;
        const double yi = polygon[i].second;
        const double xj = polygon[j].first;
        const double yj = polygon[j].second;
        if (((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

} // namespace click_obstacles_layer

PLUGINLIB_EXPORT_CLASS(
    click_obstacles_layer::DirectionalForbiddenLayer,
    nav2_costmap_2d::Layer
)
