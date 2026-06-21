#include "booster_client/node/rpc_client_node.hpp"
#include "booster_interface/message_utils.hpp"

#include <chrono>
#include <future>
#include <thread>

namespace booster_client
{

RpcClientNode::RpcClientNode(const rclcpp::Node::SharedPtr & node)
: node(node)
{
  b1_client = node->create_client<RpcService>("booster_rpc_service");
  joint_client = node->create_client<JointPrepareService>("prep_transition_service");

  mode_switch_service = node->create_service<ModeSwitchService>(
    "client/set_mode",
    [this](
      std::shared_ptr<rclcpp::Service<ModeSwitchService>> service_handle,
      std::shared_ptr<rmw_request_id_t> request_header,
      std::shared_ptr<ModeSwitchService::Request> req) {
      handle_mode_switch_request(service_handle, request_header, req);
    });

  upper_control_service = node->create_service<UpperControlService>(
    "client/set_upper_control",
    [this](
      std::shared_ptr<rclcpp::Service<UpperControlService>> service_handle,
      std::shared_ptr<rmw_request_id_t> request_header,
      std::shared_ptr<UpperControlService::Request> req) {
      handle_upper_control_request(service_handle, request_header, req);
    });

  robot_state_subscriber = node->create_subscription<RobotState>(
    "/robot_states",
    10,
    [this](const RobotState::SharedPtr msg)
    {
      this->update_robot_state(msg);
    });
}

void RpcClientNode::handle_mode_switch_request(
  std::shared_ptr<rclcpp::Service<ModeSwitchService>> service_handle,
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<ModeSwitchService::Request> req)
{
  auto send_response =
    [service_handle, request_header](bool success)
    {
      auto res = std::make_shared<ModeSwitchService::Response>();
      res->success = success;
      service_handle->send_response(*request_header, *res);
    };

  if (!joint_client->service_is_ready() || !b1_client->service_is_ready()) {
    RCLCPP_WARN(
      node->get_logger(),
      "joint service or b1 rpc service is not ready, cannot switch mode");

    send_response(false);
    return;
  }

  const uint8_t target_mode = req->mode;

  if (need_prep_first(target_mode)) {
    execute_prep_transition(target_mode, send_response);
  } else {
    execute_target_transition(target_mode, send_response);
  }
}

void RpcClientNode::handle_upper_control_request(
  std::shared_ptr<rclcpp::Service<UpperControlService>> service_handle,
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<UpperControlService::Request> req)
{
  auto send_response =
    [service_handle, request_header](bool success)
    {
      auto res = std::make_shared<UpperControlService::Response>();
      res->success = success;
      service_handle->send_response(*request_header, *res);
    };

  if (!joint_client->service_is_ready() || !b1_client->service_is_ready()) {
    RCLCPP_WARN(
      node->get_logger(),
      "joint service or b1 rpc service is not ready, cannot %s upper body custom control",
      req->enable ? "enable" : "disable");

    send_response(false);
    return;
  }

  execute_upper_control_transition(req->enable, send_response);
}

void RpcClientNode::execute_target_transition(
  uint8_t target_mode,
  std::function<void(bool)> send_response)
{
  TransitionCommand cmd;
  cmd.transition = TransitionCommand::TRANSITION_MODE_SWITCH;
  cmd.target_mode = target_mode;

  send_joint_request(
    cmd,
    [this, target_mode, send_response](JointPrepareService::Response::SharedPtr joint_res)
    {
      if (!joint_res->success) {
        RCLCPP_WARN(
          node->get_logger(),
          "Joint prepare for mode switch failed");

        send_response(false);
        return;
      }

      std::this_thread::sleep_for(
        std::chrono::duration<float>(joint_res->delay_second));

      send_mode_request(
        target_mode,
        [this, send_response](bool success)
        {
          if (!success) {
            RCLCPP_WARN(
              node->get_logger(),
              "Mode switch rpc failed");

            send_response(false);
            return;
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          send_response(true);
        });
    });
}

void RpcClientNode::execute_prep_transition(
  uint8_t target_mode,
  std::function<void(bool)> send_response)
{
  TransitionCommand cmd;
  cmd.transition = TransitionCommand::TRANSITION_MODE_SWITCH;
  cmd.target_mode = target_mode;

  send_joint_request(
    cmd,
    [this, target_mode, send_response](JointPrepareService::Response::SharedPtr joint_res)
    {
      if (!joint_res->success) {
        RCLCPP_WARN(
          node->get_logger(),
          "Joint prepare for prep mode failed");

        send_response(false);
        return;
      }

      std::this_thread::sleep_for(
        std::chrono::duration<float>(joint_res->delay_second));

      send_mode_request(
        TransitionCommand::MODE_STAND,
        [this, target_mode, send_response](bool success)
        {
          if (!success) {
            RCLCPP_WARN(
              node->get_logger(),
              "Prep mode rpc failed");

            send_response(false);
            return;
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          execute_target_transition(target_mode, send_response);
        });
    });
}

void RpcClientNode::send_joint_request(
  TransitionCommand command,
  std::function<void(JointPrepareService::Response::SharedPtr)> cb)
{
  auto req = std::make_shared<JointPrepareService::Request>();
  req->command = std::move(command);

  joint_client->async_send_request(
    req,
    [cb](rclcpp::Client<JointPrepareService>::SharedFuture future)
    {
      cb(future.get());
    });
}

void RpcClientNode::send_mode_request(
  uint8_t mode,
  std::function<void(bool)> cb)
{
  auto req = std::make_shared<RpcService::Request>();

  req->msg = booster_interface::CreateMsg<
    booster::robot::b1::LocoApiId::kChangeMode,
    booster::robot::b1::ChangeModeParameter>(
      static_cast<booster::robot::RobotMode>(mode));

  b1_client->async_send_request(
    req,
    [cb](rclcpp::Client<RpcService>::SharedFuture future)
    {
      auto res = future.get();
      cb(res->msg.status == 0);
    });
}

void RpcClientNode::execute_upper_control_transition(
  bool enable,
  std::function<void(bool)> send_response)
{
  TransitionCommand cmd;
  cmd.transition = TransitionCommand::TRANSITION_UPPER_BODY_CONTROL;
  cmd.upper_body_enable = enable;

  send_joint_request(
    cmd,
    [this, enable, send_response](JointPrepareService::Response::SharedPtr joint_res)
    {
      if (!joint_res->success) {
        RCLCPP_WARN(
          node->get_logger(),
          "Joint prepare for upper body custom control failed");

        send_response(false);
        return;
      }

      std::this_thread::sleep_for(
        std::chrono::duration<float>(joint_res->delay_second));

      send_upper_control_request(
        enable,
        [this, enable, send_response](bool success)
        {
          if (!success) {
            RCLCPP_WARN(
              node->get_logger(),
              "Upper body custom control %s failed",
              enable ? "enable" : "disable");

            send_response(false);
            return;
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(2000));

          RCLCPP_INFO(
            node->get_logger(),
            "Upper body custom control %s accepted",
            enable ? "enable" : "disable");

          send_response(true);
        });
    });
}

void RpcClientNode::send_upper_control_request(
  bool enable,
  std::function<void(bool)> cb)
{
  auto req = std::make_shared<RpcService::Request>();

  req->msg = booster_interface::CreateMsg<
    booster::robot::b1::LocoApiId::kUpperBodyCustomControl,
    booster::robot::b1::UpperBodyCustomControlParameter>(enable);

  b1_client->async_send_request(
    req,
    [cb](rclcpp::Client<RpcService>::SharedFuture future)
    {
      auto res = future.get();
      cb(res->msg.status == 0);
    });
}

void RpcClientNode::update_robot_state(RobotState::SharedPtr msg)
{
  current_mode = static_cast<RobotMode>(msg->current_mode);
}

bool RpcClientNode::need_prep_first(uint8_t target_mode)
{
  const uint8_t current_transition_mode = to_transition_mode(current_mode);

  if ((current_transition_mode == TransitionCommand::MODE_CUSTOM &&
       target_mode == TransitionCommand::MODE_WALK) ||
      (current_transition_mode == TransitionCommand::MODE_WALK &&
       target_mode == TransitionCommand::MODE_CUSTOM))
  {
    return true;
  }

  return false;
}

uint8_t RpcClientNode::to_transition_mode(RobotMode mode)
{
  switch (mode) {
    case RobotMode::DAMP:
      return TransitionCommand::MODE_DAMPING;
    case RobotMode::PREP:
      return TransitionCommand::MODE_STAND;
    case RobotMode::WALK:
      return TransitionCommand::MODE_WALK;
    case RobotMode::CUSTOM:
      return TransitionCommand::MODE_CUSTOM;
    default:
      return TransitionCommand::MODE_DAMPING;
  }
}

}  // namespace booster_client