#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class PcdPublisherNode: public rclcpp::Node {
public:
    PcdPublisherNode(): rclcpp::Node("pcd_publisher") {
        this->declare_parameter<std::string>("map", "");
        this->declare_parameter<std::string>("frame_id", "map3d");
        this->declare_parameter<double>("rate", 5.0);
        this->declare_parameter<bool>("filter_invalid_points", true);
        this->declare_parameter<double>("max_abs_coord", 100000.0);
        // Voxel downsampling of the published cloud. RViz lags when rendering the
        // full dense map, so we publish a downsampled copy. global_localization
        // re-voxelizes /map3d to map_voxel_size (0.2 m by default) before ICP, so a
        // leaf <= that does not degrade localization. Set <= 0 to disable.
        this->declare_parameter<double>("voxel_leaf_size", 0.1);

        rclcpp::QoS qos(1);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.transient_local();
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map3d", qos);

        std::string path = this->get_parameter("map").as_string();
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid PCD path: (empty)");
        } else {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load PCD: %s", path.c_str());
            } else {
                cloud_ = downsampleCloud(sanitizeCloud(cloud));
                RCLCPP_INFO(
                    this->get_logger(),
                    "Loaded PCD: %s with %zu published points",
                    path.c_str(),
                    cloud_->points.size()
                );
            }
        }

        double rate = this->get_parameter("rate").as_double();
        if (rate <= 0.0)
            rate = 1.0;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&PcdPublisherNode::onTimer, this)
        );
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    sanitizeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) {
        bool filter_invalid = this->get_parameter("filter_invalid_points").as_bool();
        double max_abs_coord = this->get_parameter("max_abs_coord").as_double();

        if (!filter_invalid) {
            return input;
        }

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        output->points.reserve(input->points.size());

        size_t removed_non_finite = 0;
        size_t removed_out_of_range = 0;

        for (const auto& pt: input->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                ++removed_non_finite;
                continue;
            }
            if (std::abs(pt.x) > max_abs_coord || std::abs(pt.y) > max_abs_coord
                || std::abs(pt.z) > max_abs_coord)
            {
                ++removed_out_of_range;
                continue;
            }
            output->points.push_back(pt);
        }

        output->width = static_cast<uint32_t>(output->points.size());
        output->height = 1;
        output->is_dense = true;

        const size_t total_removed = removed_non_finite + removed_out_of_range;
        if (total_removed > 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Filtered invalid map points: removed %zu (non-finite=%zu, out-of-range=%zu, max_abs_coord=%.2f), kept %zu",
                total_removed,
                removed_non_finite,
                removed_out_of_range,
                max_abs_coord,
                output->points.size()
            );
        }

        return output;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) {
        double leaf = this->get_parameter("voxel_leaf_size").as_double();
        if (leaf <= 0.0 || input->points.empty()) {
            return input;
        }

        auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(input);
        vg.setLeafSize(
            static_cast<float>(leaf),
            static_cast<float>(leaf),
            static_cast<float>(leaf)
        );
        vg.filter(*output);
        output->is_dense = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Voxel-downsampled map for publishing: leaf=%.3f m, %zu -> %zu points",
            leaf,
            input->points.size(),
            output->points.size()
        );

        return output;
    }

    void onTimer() {
        if (!cloud_) {
            // publish empty
            sensor_msgs::msg::PointCloud2 msg;
            msg.header.frame_id = this->get_parameter("frame_id").as_string();
            msg.header.stamp = this->now();
            publisher_->publish(msg);
            return;
        }
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud_, msg);
        msg.header.frame_id = this->get_parameter("frame_id").as_string();
        msg.header.stamp = this->now();
        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcdPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}