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

#define SIMULATION_MODE 1

#define N_DOF 7
#define MAX_INVALID_COUNT 100

#include <string>

#include <controller_interface/controller_interface.hpp>
#include "franka_example_controllers/visibility_control.h"

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include "mpc_interfaces/msg/num.hpp"
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
#include "TicToc.hpp"
#include "param_robot.h"
#include "casadi_types.h"
#include "trajectory_settings.hpp"
#include <Eigen/Dense>

#include <future> // Include the future and async library

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers
{

    struct shm_flags
    {
        int8_t start = 0;
        int8_t reset = 0;
        int8_t stop = 0;
        int8_t torques_valid = 0;
    };
    /**
     * The gravity compensation controller only sends zero torques so that the robot does gravity
     * compensation
     */
    class ModelPredictiveControllerCasadi : public controller_interface::ControllerInterface
    {
    public:
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

    private:
        std::string arm_id_;
        const int num_joints = N_DOF;
        double torques_prev[N_DOF] = {0}; // previous torques
        int invalid_counter = 0;          // counter for invalid data, if exceeds MAX_INVALID_COUNT, terminate the controller
        uint global_traj_count = 0;
        bool mpc_started = false;
        int8_t traj_select = 1; // default trajectory
        bool first_torque_read = false;
        const std::string urdf_filename = std::string(MASTERDIR) + "/urdf_creation/fr3_no_hand_7dof.urdf";
        const std::string casadi_mpc_config_filename = std::string(MASTERDIR) + "/config_settings/casadi_mpc_weights_fr3_no_hand.json";
        const std::string general_config_filename = std::string(MASTERDIR) + "/config_settings/general_settings.json";
        const std::string ekf_config_filename = std::string(MASTERDIR) + "/config_settings/ekf_settings.json";
        const std::string tcp_frame_name = "fr3_link8_tcp";
        bool use_gravity = false;
        bool use_planner = false;
        bool use_lowpass_filter = false;
        bool use_ekf = false;
        CasadiController controller = CasadiController(urdf_filename, casadi_mpc_config_filename, tcp_frame_name, use_gravity, use_planner);
        Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(num_joints);
        Eigen::VectorXd x_measured = Eigen::VectorXd::Zero(2*num_joints);
        double* x_filtered_ptr = x_measured.data();
        double* u_next = controller.get_u_next();
        //Eigen::Map<Eigen::VectorXd>(u_next, robot_config.nq_red);
        uint N_step = controller.get_N_step();
        CasadiEKF ekf = CasadiEKF(ekf_config_filename);
        SignalFilter lowpass_filter = SignalFilter(num_joints, Ts, 400, 400);

        double Ts = 0.001;
        double T1 = 1/400, A1 = std::exp(-Ts/T1), B1 = 1 - A1; // Lowpass Filter for q
        double T2 = 1/400, A2 = std::exp(-Ts/T2), B2 = 1 - A2; // Lowpass Filter for q_p
        std::future<Eigen::VectorXd> tau_full_future;
        ErrorFlag error_flag = ErrorFlag::NO_ERROR;

        robot_config_t robot_config = get_robot_config();
        casadi_uint traj_len = controller.get_traj_data_real_len();

        const std::vector<SharedMemoryInfo> shm_readwrite_infos = {
            {"data_from_simulink_start", sizeof(int8_t), 1},
            {"data_from_simulink_reset", sizeof(int8_t), 1},
            {"data_from_simulink_stop", sizeof(int8_t), 1},
            {"readonly_mode", sizeof(int8_t), 1},
            {"read_traj_length", sizeof(casadi_uint), 1},
            {"read_traj_data_full", 19 * sizeof(casadi_real), traj_len},
            {"read_frequency_full", sizeof(casadi_real), traj_len},
            {"read_state_data_full", sizeof(casadi_real) * robot_config.nx, traj_len},
            {"read_control_data_full", sizeof(casadi_real) * robot_config.nq, traj_len}};

        const std::vector<std::string> sem_readwrite_names = {
            "shm_changed_semaphore",
        };
        SharedMemory shm;

        double current_frequency = 0.0;
        TicToc timer_mpc_solver;

        // rclcpp::Subscription<mpc_interfaces::msg::Num>::SharedPtr subscription_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr start_mpc_service_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr reset_mpc_service_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr stop_mpc_service_;
        rclcpp::Service<mpc_interfaces::srv::TrajectoryCommand>::SharedPtr traj_switch_service_;
        rclcpp::Service<mpc_interfaces::srv::CasadiMPCTypeCommand>::SharedPtr mpc_switch_service_;

        void open_shared_memories();
        void close_shared_memories();
        // void topic_callback(const mpc_interfaces::msg::Num & msg);
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
        nlohmann::json read_config(std::string file_path);
    };
} // namespace franka_example_controllers
