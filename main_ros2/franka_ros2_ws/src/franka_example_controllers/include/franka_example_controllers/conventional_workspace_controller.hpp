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

#pragma once

// #define MY_LOG_LEVEL RCUTILS_LOG_SEVERITY_WARN
#define MY_LOG_LEVEL RCUTILS_LOG_SEVERITY_INFO

#define MAX_INVALID_COUNT 100

#include <string>

#include <controller_interface/controller_interface.hpp>
#include "franka_example_controllers/visibility_control.h"
#include <franka_example_controllers/common_ros_base_controller.hpp>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include "mpc_interfaces/msg/num.hpp"
#include "mpc_interfaces/msg/control_array.hpp"
#include "mpc_interfaces/srv/add_three_ints.hpp"
#include "mpc_interfaces/srv/simple_command.hpp"
#include "mpc_interfaces/srv/trajectory_command.hpp"
#include "mpc_interfaces/srv/workspace_controller_type_command.hpp"
#include <semaphore.h>

#include "json.hpp"
#include "TicToc.hpp"
#include "SignalFilter.hpp"
#include "SharedMemory.hpp"
#include "WorkspaceController.hpp"
#include "CasadiEKF.hpp"
#include "FullSystemTorqueMapper.hpp"
#include "TicToc.hpp"
#include "param_robot.h"
#include "casadi_types.h"
#include "trajectory_settings.hpp"
#include <Eigen/Dense>
#include <random>

#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>
#include <future> // Include the future and async library

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers
{
    /*
    This class implements a conventional workspace controller for the Franka robot.
    It inherits from the CommonROSBaseController and provides methods to configure, activate, and update the
    controller state. The controller uses a WorkspaceController object to compute the control inputs based on the
    current state of the robot and the desired trajectory. It also provides a service to switch between
    different workspace controller types.
    */
    class ConventionalWorkspaceController : public franka_example_controllers::CommonROSBaseController
    {
    public:
        public:
        ConventionalWorkspaceController()
         : CommonROSBaseController(), controller(WorkspaceController(urdf_filename, general_config_filename))
         {
            base_controller = &controller;
            init_base_controller();
         }

        FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    private:
        WorkspaceController controller;

        rclcpp::Service<mpc_interfaces::srv::WorkspaceControllerTypeCommand>::SharedPtr workspace_controller_switch_service_;

        void controller_switch(const std::shared_ptr<mpc_interfaces::srv::WorkspaceControllerTypeCommand::Request> request,
                        std::shared_ptr<mpc_interfaces::srv::WorkspaceControllerTypeCommand::Response> response);
    };

} // namespace franka_example_controllers