#include <memory>
#include <string>
#include "adm_tf/adm_tf_listener_node.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h> // Include for quaternion operations
#include <tf2/LinearMath/Matrix3x3.h> // Include for matrix operations

AdmTFListenerNode::AdmTFListenerNode(std::string name) : Node(name)
{
    // Initialize the TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Listener created!!");

    // Create a timer to periodically check for transforms (every second)
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&AdmTFListenerNode::lookupTransform, this));
}

void AdmTFListenerNode::lookupTransform()
{
    geometry_msgs::msg::TransformStamped transform;
    try {
        // Lookup the transform from "base_link" to each AU RADAR
        std::vector<std::string> radar_links = {
            "RADAR_FRONT",
            "RADAR_FRONT_RIGHT",
            "RADAR_FRONT_LEFT",
            "RADAR_REAR_RIGHT",
            "RADAR_REAR_LEFT"
        };

        for (const auto & radar : radar_links) {
            transform = tf_buffer_->lookupTransform("base_link", radar, tf2::TimePointZero);

            // Print the transform translation
            RCLCPP_INFO(this->get_logger(), "Transform received for %s: translation (x: %f, y: %f, z: %f)",
                        radar.c_str(),
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z);

            // Print the transform rotation (as quaternion)
            RCLCPP_INFO(this->get_logger(), "Rotation (qx: %f, qy: %f, qz: %f, qw: %f)",
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w);

            TransformToRPY(transform);
        }
    }
    catch (const tf2::TransformException & ex) {
        // Warn if the transform is not available
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        RCLCPP_INFO(this->get_logger(), "Broadcasting transform from %s to %s", transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
    }
}



void AdmTFListenerNode::TransformToRPY(geometry_msgs::msg::TransformStamped transform) {
    // Assuming transform is of type geometry_msgs::msg::TransformStamped
    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;

    // Create a Quaternion object
    tf2::Quaternion quaternion(qx, qy, qz, qw);

    // Convert quaternion to roll, pitch, yaw
    tf2::Matrix3x3 m(quaternion);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Now roll, pitch, and yaw hold the converted values
    // You can print or use them as needed
    std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
}
