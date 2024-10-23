#include "rclcpp/rclcpp.hpp"
#include "adm_tf/adm_tf_listener_node.hpp"

int main(int argc, char ** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a shared pointer to the AdmTFListenerNode
  auto node = std::make_shared<AdmTFListenerNode>();

  // Spin the node to process callbacks
  rclcpp::spin(node);

  // Shutdown ROS 2 when finished
  rclcpp::shutdown();
  
  return 0;
}
