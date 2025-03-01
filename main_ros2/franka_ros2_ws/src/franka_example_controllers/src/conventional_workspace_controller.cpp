// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_example_controllers/conventional_workspace_controller.hpp>

#include <exception>
#include <string>
#include <unistd.h>

#include <memory>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace franka_example_controllers
{

    CallbackReturn ConventionalWorkspaceController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        arm_id_ = get_node()->get_parameter("arm_id").as_string();

        RCLCPP_INFO(get_node()->get_logger(), "Configuring MPC controller for %s arm", arm_id_.c_str());

        // subscription_ = get_node()->create_subscription<mpc_interfaces::msg::ControlArray>(
        //     "topic", 10, std::bind(&ConventionalWorkspaceController::topic_callback, this, _1));

        // RCLCPP_INFO(get_node()->get_logger(), "Subscribed to topic 'topic'");

        std::string node_name = get_node()->get_name();
        std::string service_prefix = "/" + node_name + "/";

        start_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "start_mpc_service",
            std::bind(&ConventionalWorkspaceController::start_mpc, this, std::placeholders::_1, std::placeholders::_2));

        reset_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "reset_mpc_service",
            std::bind(&ConventionalWorkspaceController::reset_mpc, this, std::placeholders::_1, std::placeholders::_2));

        stop_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "stop_mpc_service",
            std::bind(&ConventionalWorkspaceController::stop_mpc, this, std::placeholders::_1, std::placeholders::_2));

        traj_switch_service_ = get_node()->create_service<mpc_interfaces::srv::TrajectoryCommand>(
            service_prefix + "traj_switch_service",
            std::bind(&ConventionalWorkspaceController::traj_switch, this, std::placeholders::_1, std::placeholders::_2));

        workspace_controller_switch_service_ = get_node()->create_service<mpc_interfaces::srv::WorkspaceControllerTypeCommand>(
            service_prefix + "workspace_controller_switch_service",
            std::bind(&ConventionalWorkspaceController::controller_switch, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_node()->get_logger(), "Services created");

        return CallbackReturn::SUCCESS;
    }

    void ConventionalWorkspaceController::controller_switch(const std::shared_ptr<mpc_interfaces::srv::WorkspaceControllerTypeCommand::Request> request,
                                                         std::shared_ptr<mpc_interfaces::srv::WorkspaceControllerTypeCommand::Response> response)
    {
        ControllerType old_controller_type = controller.get_controller_type();
        if(old_controller_type == static_cast<ControllerType>(request->workspace_controller_type))
        {
            response->status = "Controller type " + std::to_string(request->workspace_controller_type) + " already selected";
            RCLCPP_INFO(get_node()->get_logger(), "Controller type %ld already selected", request->workspace_controller_type);
        }
        else
        {
            ControllerType workspace_controller_type = static_cast<ControllerType>(request->workspace_controller_type);
            controller.switchController(workspace_controller_type);
            
            controller.reset();
            response->status = "Controller type " + std::to_string(request->workspace_controller_type) + " selected";
            std::string status_message = "Switched to " + controller.get_classic_controller_string(workspace_controller_type) + " Controller";
            RCLCPP_INFO(get_node()->get_logger(), status_message.c_str());
            response->status = status_message;
        }
    }

} // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ConventionalWorkspaceController,
                       controller_interface::ControllerInterface)