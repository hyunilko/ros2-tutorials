#include "rclcpp/rclcpp.hpp"

#include "adm_tf/adm_tf_static_publisher_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<AdmTFStaticPublisherNode> node = std::make_shared<AdmTFStaticPublisherNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


