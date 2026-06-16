#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "booster_client_interface/srv/set_mode.hpp"
#include "booster_client_interface/srv/set_upper_control.hpp"
#include "booster_interface/srv/rpc_service.hpp"
#include "booster_joint_interface/srv/prepare_transition.hpp"
#include "booster_model/mode_enum.hpp"
#include "booster_interface/msg/robot_states_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"

namespace booster_client
{

class RpcClientNode
{
public:
  explicit RpcClientNode(const rclcpp::Node::SharedPtr & node);

private:
  using RpcService = booster_interface::srv::RpcService;
  using ModeSwitchService = booster_client_interface::srv::SetMode;
  using UpperControlService = booster_client_interface::srv::SetUpperControl;
  using JointPrepareService = booster_joint_interface::srv::PrepareTransition;
  using RobotMode = booster_model::mode::RobotMode;
  using RobotState = booster_interface::msg::RobotStatesMsg;
  using UpperControl = booster_model::mode::UpperControl;

  void handle_mode_switch_request(
    std::shared_ptr<rclcpp::Service<ModeSwitchService>> service_handle,
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<ModeSwitchService::Request> req);

  void handle_upper_control_request(
    std::shared_ptr<rclcpp::Service<UpperControlService>> service_handle,
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<UpperControlService::Request> req);
  
  void update_robot_state()

  rclcpp::Node::SharedPtr node;
  rclcpp::Client<RpcService>::SharedPtr b1_client;
  rclcpp::Client<JointPrepareService>::SharedPtr joint_client;
  rclcpp::Service<ModeSwitchService>::SharedPtr mode_switch_service;
  rclcpp::Service<UpperControlService>::SharedPtr upper_control_service;
  rclcpp::Subscriber<RobotState>::SharedPtr robot_state_subscriber;

  RobotMode current_mode;

};

}  // namespace booster_client
