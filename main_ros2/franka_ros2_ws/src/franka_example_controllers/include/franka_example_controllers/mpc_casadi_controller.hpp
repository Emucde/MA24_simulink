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
#define CUSTOM_LIST 1

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
#include "mpc_interfaces/srv/casadi_mpc_type_command.hpp"
#include <semaphore.h>

#include "json.hpp"
#include "TicToc.hpp"
#include "SignalFilter.hpp"
#include "SharedMemory.hpp"
#include "CasadiController.hpp"
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
    /**
     * The gravity compensation controller only sends zero torques so that the robot does gravity
     * compensation
     */
    class ModelPredictiveControllerCasadi : public controller_interface::ControllerInterface, franka_example_controllers::CommonROSBaseController
    {
    public:
        ModelPredictiveControllerCasadi()
         : CommonROSBaseController(), controller(CasadiController(urdf_filename, casadi_mpc_config_filename, general_config_filename))
         {
            base_controller = &controller;
            init_base_controller();
         }

        FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
        CallbackReturn on_init() override;

        FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
        [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
            const override;

        FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
        [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
            const override;

        FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
        controller_interface::return_type update(const rclcpp::Time &time,
                                                 const rclcpp::Duration &period) override;

        void solve();
    private:
#ifdef CUSTOM_LIST
    const std::string casadi_mpc_config_filename = std::string(MASTERDIR) + "/config_settings/casadi_mpc_weights_fr3_no_hand_custom_list.json";
#else
    const std::string casadi_mpc_config_filename = std::string(MASTERDIR) + "/config_settings/casadi_mpc_weights_fr3_no_hand_simulink.json";
#endif
        bool use_planner = false;
        CasadiController controller;
        // CasadiController controller = CasadiController(urdf_filename, casadi_mpc_config_filename, general_config_filename);
        // FullSystemTorqueMapper* torque_mapper = controller.get_torque_mapper();
        // uint N_step = controller.get_N_step();
        // uint traj_len = controller.get_traj_data_real_len();

        // rclcpp::Subscription<mpc_interfaces::msg::ControlArray>::SharedPtr subscription_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr start_mpc_service_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr reset_mpc_service_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr stop_mpc_service_;
        rclcpp::Service<mpc_interfaces::srv::TrajectoryCommand>::SharedPtr traj_switch_service_;
        rclcpp::Service<mpc_interfaces::srv::CasadiMPCTypeCommand>::SharedPtr mpc_switch_service_;

        void open_shared_memories();
        void close_shared_memories();
        // void topic_callback(const mpc_interfaces::msg::ControlArray & msg);
        void start_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
                       std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response);
        void reset_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
                       std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response);
        void stop_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
                      std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response);
        void traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
                         std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Response> response);
        void mpc_switch(const std::shared_ptr<mpc_interfaces::srv::CasadiMPCTypeCommand::Request> request,
                        std::shared_ptr<mpc_interfaces::srv::CasadiMPCTypeCommand::Response> response);

    };

    // #ifdef SIMULATION_MODE
    // Eigen::VectorXd generateNoiseVector(int n, double Ts, double mean_noise_amplitude);
    // #endif
} // namespace franka_example_controllers