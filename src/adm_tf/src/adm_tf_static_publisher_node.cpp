#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "adm_tf/adm_tf_static_publisher_node.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "urdf/model.h"  // For loading URDF
#include <ament_index_cpp/get_package_share_directory.hpp>

AdmTFStaticPublisherNode::AdmTFStaticPublisherNode(std::string name) : Node(name)
{
    // QoS settings for the broadcaster
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
        .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
        .keep_last(10)
        .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        .avoid_ros_namespace_conventions(false);

    // Create the broadcaster
    _broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this, qos_profile);

    // Load URDF and broadcast transforms
    std::string urdf_path = ament_index_cpp::get_package_share_directory("adm_tf") + "/config/morning_car.urdf";    
    loadURDFAndBroadcastTransforms(urdf_path);

    // Create a timer to periodically send the transform (every second)
    _timer = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&AdmTFStaticPublisherNode::sendTransform, this));

    RCLCPP_INFO(this->get_logger(), "Static Publisher created and broadcasting transforms periodically!!");
}

void AdmTFStaticPublisherNode::loadURDFAndBroadcastTransforms(const std::string & urdf_path)
{
    urdf::Model model;
    if (!model.initFile(urdf_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load URDF file: %s", urdf_path.c_str());
        return;
    }

    for (const auto & joint : model.joints_) {
        if (joint.second->type == urdf::Joint::FIXED) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.frame_id = "base_link"; // Set to the correct parent link
            transform.child_frame_id = joint.second->child_link_name; // Set child link properly

            // Check if child_frame_id is valid
            if (transform.child_frame_id.empty() || transform.header.frame_id.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Invalid frame_id or child_frame_id for joint: %s", joint.second->name.c_str());
                continue; // Skip this iteration if the IDs are not set
            }

            transform.header.stamp = this->now();

            RCLCPP_INFO(this->get_logger(), "Transform sent: [%s] -> [%s], timestamp: %f",
                        transform.header.frame_id.c_str(),
                        transform.child_frame_id.c_str(),
                        rclcpp::Time(transform.header.stamp).seconds());

            // Set translation and rotation
            transform.transform.translation.x = joint.second->parent_to_joint_origin_transform.position.x;
            transform.transform.translation.y = joint.second->parent_to_joint_origin_transform.position.y;
            transform.transform.translation.z = joint.second->parent_to_joint_origin_transform.position.z;

            transform.transform.rotation.x = joint.second->parent_to_joint_origin_transform.rotation.x;
            transform.transform.rotation.y = joint.second->parent_to_joint_origin_transform.rotation.y;
            transform.transform.rotation.z = joint.second->parent_to_joint_origin_transform.rotation.z;
            transform.transform.rotation.w = joint.second->parent_to_joint_origin_transform.rotation.w;

            // Broadcast the transform
            _broadcaster->sendTransform(transform);
        }
    }

}


void AdmTFStaticPublisherNode::sendTransform()
{
    std::string urdf_path = ament_index_cpp::get_package_share_directory("adm_tf") + "/config/morning_car.urdf";
    loadURDFAndBroadcastTransforms(urdf_path);

    // RCLCPP_INFO(this->get_logger(), "Transform sent: [%s] -> [%s], timestamp: %f",
    //             transform.header.frame_id.c_str(),
    //             transform.child_frame_id.c_str(),
    //             rclcpp::Time(this->now()).seconds());

    // Broadcast the static transform
    _broadcaster->sendTransform(transform);
}
