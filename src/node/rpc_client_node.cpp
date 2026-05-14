  #include "booster_client/node/rpc_client_node.hpp"
  #include "booster_interface/message_utils.hpp"
  #include <future>

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
      RCLCPP_WARN(
        node->get_logger(),
        "joint service or b1 rpc service is not ready, cannot switch mode");

      send_response(false);
      return;
    }

    auto joint_req = std::make_shared<JointPrepareService::Request>();
    joint_req->command.transition =
      booster_joint_interface::msg::TransitionCommand::TRANSITION_MODE_SWITCH;
    joint_req->command.target_mode = req->mode;

    joint_client->async_send_request(
      joint_req,
      [this, mode = req->mode, send_response](
        rclcpp::Client<JointPrepareService>::SharedFuture future) {
        auto joint_res = future.get();

        if (!joint_res->success) {
          RCLCPP_WARN(
            node->get_logger(),
            "Joint prepare for mode switch failed, cannot switch mode");

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
          [this, send_response](rclcpp::Client<RpcService>::SharedFuture future) {
            auto rpc_res = future.get();
            const bool success = rpc_res->msg.status == 0;

            if (!success) {
              RCLCPP_WARN(node->get_logger(), "Mode switch rpc failed");
            }

            send_response(success);
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
      RCLCPP_WARN(
        node->get_logger(),
        "joint service or b1 rpc service is not ready, cannot %s upper body custom control",
        req->enable ? "enable" : "disable");

      send_response(false);
      return;
    }

    auto joint_req = std::make_shared<JointPrepareService::Request>();
    joint_req->command.transition =
      booster_joint_interface::msg::TransitionCommand::TRANSITION_UPPER_BODY_CONTROL;
    joint_req->command.upper_body_enable = req->enable;

    joint_client->async_send_request(
      joint_req,
      [this, enable = req->enable, send_response](
        rclcpp::Client<JointPrepareService>::SharedFuture future) {
        auto joint_res = future.get();

        if (!joint_res->success) {
          RCLCPP_WARN(
            node->get_logger(),
            "Joint prepare for upper body custom control failed, cannot %s upper body custom control",
            enable ? "enable" : "disable");

          send_response(false);
          return;
        }

        auto rpc_req = std::make_shared<RpcService::Request>();
        rpc_req->msg = booster_interface::CreateMsg<
          booster::robot::b1::LocoApiId::kUpperBodyCustomControl,
          booster::robot::b1::UpperBodyCustomControlParameter>(enable);

        b1_client->async_send_request(
          rpc_req,
          [this, enable, send_response](rclcpp::Client<RpcService>::SharedFuture future) {
            auto rpc_res = future.get();
            const bool success = rpc_res->msg.status == 0;

            if (success) {
              RCLCPP_WARN(
                node->get_logger(),
                "Upper body custom control %s accepted",
                enable ? "enable" : "disable");
            } else {
              RCLCPP_WARN(
                node->get_logger(),
                "Upper body custom control %s failed",
                enable ? "enable" : "disable");
            }

            send_response(success);
          });
      });
  }

  }  // namespace booster_client
