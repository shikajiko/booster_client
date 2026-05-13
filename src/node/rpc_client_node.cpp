#include "booster_client/node/rpc_client_node.hpp"

#include "booster_interface/booster_interface/message_utils.hpp"

#include <future>

namespace booster_client
{

RpcClientNode::RpcClientNode(const rclcpp::Node::SharedPtr & node)
{
  b1_client = node->create_client<RpcService>("booster_rpc_service");
  joint_client = node->create_client<JointPrepareService>("prepare_control_transition");
  mode_switch_service = node->create_service<ModeSwitchService>(
    "set_mode",
    [this](
      std::shared_ptr<rclcpp::Service<ModeSwitchService>> service_handle,
      std::shared_ptr<rmw_request_id_t> request_header,
      std::shared_ptr<ModeSwitchService::Request> req) {
      handle_mode_switch_request(service_handle, request_header, req);
    });
  upper_control_service = node->create_service<UpperControlService>(
    "set_upper_control",
    [this](
      std::shared_ptr<rclcpp::Service<UpperControlService>> service_handle,
      std::shared_ptr<rmw_request_id_t> request_header,
      std::shared_ptr<UpperControlService::Request> req) {
      handle_upper_control_request(service_handle, request_header, req);
    });
}

void RpcClientNode::handle_mode_switch_request(
  std::shared_ptr<rclcpp::Service<ModeSwitchService>> service_handle,
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<ModeSwitchService::Request> req)
{
  auto send_response =
    [service_handle, request_header](bool success) {
      auto res = std::make_shared<ModeSwitchService::Response>();
      res->success = success;
      service_handle->send_response(*request_header, *res);
    };

  if (!joint_client->service_is_ready() || !b1_client->service_is_ready()) {
    send_response(false);
    return;
  }

  auto joint_req = std::make_shared<JointPrepareService::Request>();
  joint_req->command.transition =
    booster_joint_interface::msg::PrepareControlTransitionCommand::TRANSITION_MODE_SWITCH;
  joint_req->command.target_mode = req->mode;

  joint_client->async_send_request(
    joint_req,
    [this, mode = req->mode, send_response](
      rclcpp::Client<JointPrepareService>::SharedFuture future) {
      auto joint_res = future.get();
      if (!joint_res->success) {
        send_response(false);
        return;
      }

      auto rpc_req = std::make_shared<RpcService::Request>();
      rpc_req->msg = booster_interface::CreateMsg<
        booster::robot::b1::LocoApiId::kChangeMode,
        booster::robot::b1::ChangeModeParameter>(
        static_cast<booster::robot::RobotMode>(mode));

      b1_client->async_send_request(
        rpc_req,
        [send_response](rclcpp::Client<RpcService>::SharedFuture future) {
          auto rpc_res = future.get();
          send_response(rpc_res->msg.status == 0);
        });
    });
}

void RpcClientNode::handle_upper_control_request(
  std::shared_ptr<rclcpp::Service<UpperControlService>> service_handle,
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<UpperControlService::Request> req)
{
  auto send_response =
    [service_handle, request_header](bool success) {
      auto res = std::make_shared<UpperControlService::Response>();
      res->success = success;
      service_handle->send_response(*request_header, *res);
    };

  if (!joint_client->service_is_ready() || !b1_client->service_is_ready()) {
    send_response(false);
    return;
  }

  auto joint_req = std::make_shared<JointPrepareService::Request>();
  joint_req->command.transition =
    booster_joint_interface::msg::PrepareControlTransitionCommand::TRANSITION_UPPER_BODY_CONTROL;
  joint_req->command.upper_body_enable = req->enable;

  joint_client->async_send_request(
    joint_req,
    [this, enable = req->enable, send_response](
      rclcpp::Client<JointPrepareService>::SharedFuture future) {
      auto joint_res = future.get();
      if (!joint_res->success) {
        send_response(false);
        return;
      }

      auto rpc_req = std::make_shared<RpcService::Request>();
      rpc_req->msg = booster_interface::CreateMsg<
        booster::robot::b1::LocoApiId::kUpperBodyCustomControl,
        booster::robot::b1::UpperBodyCustomControlParameter>(enable);

      b1_client->async_send_request(
        rpc_req,
        [send_response](rclcpp::Client<RpcService>::SharedFuture future) {
          auto rpc_res = future.get();
          send_response(rpc_res->msg.status == 0);
        });
    });
}

}  // namespace booster_client
