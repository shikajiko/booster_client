#include "booster_client/node/rpc_client_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rpc_client_node");
  auto rpc_client_node = std::make_shared<booster_client::RpcClientNode>(node);
  (void)rpc_client_node;

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
