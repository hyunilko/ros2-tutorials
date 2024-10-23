#ifndef __SIMPLE_TF_LISTENER_NODE_HPP__
#define __SIMPLE_TF_LISTENER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class AdmTFListenerNode : public rclcpp::Node {

public:
    AdmTFListenerNode(std::string name = "adm_tf_listener");

private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Add the timer as a member variable
    rclcpp::TimerBase::SharedPtr timer_;

    // Function to lookup transform
    void lookupTransform();
    void TransformToRPY(geometry_msgs::msg::TransformStamped transform);
};

#endif // __SIMPLE_TF_LISTENER_NODE_HPP__
