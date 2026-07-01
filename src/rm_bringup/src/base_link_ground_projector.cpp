#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

class BaseLinkGroundProjector: public rclcpp::Node {
public:
    BaseLinkGroundProjector():
        Node("base_link_ground_projector"),
        tf_broadcaster_(*this) {
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
        sensor_frame_ = declare_parameter<std::string>("sensor_frame", "body");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        ground_z_ = declare_parameter<double>("ground_z", 0.0);

        sensor_to_base_x_ = declare_parameter<double>("sensor_to_base_x", 0.0);
        sensor_to_base_y_ = declare_parameter<double>("sensor_to_base_y", 0.1578);
        sensor_to_base_z_ = declare_parameter<double>("sensor_to_base_z", -0.2932);
        sensor_to_base_roll_ =
            declare_parameter<double>("sensor_to_base_roll", -1.7592918860102842);
        sensor_to_base_pitch_ =
            declare_parameter<double>("sensor_to_base_pitch", -0.2617993877991494);
        sensor_to_base_yaw_ = declare_parameter<double>("sensor_to_base_yaw", 0.0);

        sensor_to_base_tf_ = makeSensorToBaseTransform();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&BaseLinkGroundProjector::onOdom, this, std::placeholders::_1)
        );
    }

private:
    tf2::Transform makeSensorToBaseTransform() const {
        tf2::Quaternion q_sensor_base;
        q_sensor_base.setRPY(sensor_to_base_roll_, sensor_to_base_pitch_, sensor_to_base_yaw_);
        q_sensor_base.normalize();

        tf2::Transform sensor_to_base_tf(q_sensor_base);
        sensor_to_base_tf.setOrigin(
            tf2::Vector3(sensor_to_base_x_, sensor_to_base_y_, sensor_to_base_z_)
        );
        return sensor_to_base_tf;
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Transform odom_to_sensor_tf;
        tf2::fromMsg(msg->pose.pose, odom_to_sensor_tf);

        const tf2::Transform odom_to_raw_base_tf = odom_to_sensor_tf * sensor_to_base_tf_;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(odom_to_raw_base_tf.getRotation()).getRPY(roll, pitch, yaw);

        tf2::Quaternion yaw_only;
        yaw_only.setRPY(0.0, 0.0, yaw);
        yaw_only.normalize();

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform.translation.x = odom_to_raw_base_tf.getOrigin().x();
        tf_msg.transform.translation.y = odom_to_raw_base_tf.getOrigin().y();
        tf_msg.transform.translation.z = ground_z_;
        tf_msg.transform.rotation = tf2::toMsg(yaw_only);
        tf_broadcaster_.sendTransform(tf_msg);
    }

    std::string odom_frame_;
    std::string odom_topic_;
    std::string sensor_frame_;
    std::string base_frame_;
    double ground_z_ { 0.0 };
    double sensor_to_base_x_ { 0.0 };
    double sensor_to_base_y_ { 0.1578 };
    double sensor_to_base_z_ { -0.2932 };
    double sensor_to_base_roll_ { -1.7592918860102842 };
    double sensor_to_base_pitch_ { -0.2617993877991494 };
    double sensor_to_base_yaw_ { 0.0 };

    tf2::Transform sensor_to_base_tf_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseLinkGroundProjector>());
    rclcpp::shutdown();
    return 0;
}
