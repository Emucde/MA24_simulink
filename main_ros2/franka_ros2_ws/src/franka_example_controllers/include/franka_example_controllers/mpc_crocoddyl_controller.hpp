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

#define MAX_INVALID_COUNT 100

#include <string>

#include <controller_interface/controller_interface.hpp>
#include "franka_example_controllers/visibility_control.h"

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
#include "CrocoddylController.hpp"
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
    class ModelPredictiveControllerCrocoddyl : public controller_interface::ControllerInterface
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

        void solve();
    private:
        std::string arm_id_;
        robot_config_t robot_config = get_robot_config();
        const int nq = robot_config.nq;
        const int nx = robot_config.nx;
        Eigen::VectorXd state = Eigen::VectorXd::Zero(nq * 2);
        int invalid_counter = 0;          // counter for invalid data, if exceeds MAX_INVALID_COUNT, terminate the controller
        uint global_traj_count = 0;
        bool mpc_started = false;
        int8_t traj_select = 1; // default trajectory
        const std::string urdf_filename = std::string(MASTERDIR) + "/urdf_creation/fr3_no_hand_7dof.urdf";
        const std::string crocoddyl_config_filename = std::string(MASTERDIR) + "/utils_python/mpc_weights_crocoddyl.json";
        const std::string general_config_filename = std::string(MASTERDIR) + "/config_settings/general_settings.json";
        const std::string ekf_config_filename = std::string(MASTERDIR) + "/config_settings/ekf_settings.json";
        const std::string tcp_frame_name = "fr3_link8_tcp";
        bool use_gravity = false;
        bool use_planner = false;
        bool use_lowpass_filter = false;
        bool use_ekf = false;
        int solver_steps = 1;
        int solver_step_counter = 0;
        #ifdef SIMULATION_MODE
        bool use_noise=false;
        double mean_noise_amplitude = 0;
        #endif
        CrocoddylController controller = CrocoddylController(urdf_filename, crocoddyl_config_filename, general_config_filename);
        Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);
        Eigen::VectorXd x_prev = Eigen::VectorXd::Zero(nx);
        Eigen::VectorXd x_measured = Eigen::VectorXd::Zero(nx);
        Eigen::VectorXd x_filtered = Eigen::VectorXd::Zero(nx);
        double* x_filtered_ekf_ptr = x_measured.data();
        double* x_filtered_lowpass_ptr = x_measured.data();
        std::vector< Eigen::VectorXd > &u_opt_full = controller.get_u_opt_full();
        uint N_step = controller.get_N_step();

        double Ts = 0.001;
        double T1 = 1/400, A1 = std::exp(-Ts/T1), B1 = 1 - A1; // Lowpass Filter for q
        double T2 = 1/400, A2 = std::exp(-Ts/T2), B2 = 1 - A2; // Lowpass Filter for q_p

        FullSystemTorqueMapper* torque_mapper = controller.get_torque_mapper();
        CasadiEKF ekf = CasadiEKF(ekf_config_filename);
        SignalFilter lowpass_filter = SignalFilter(nq, Ts, state.data(), 400, 400);

        ErrorFlag error_flag = ErrorFlag::NO_ERROR;

        casadi_uint traj_len = controller.get_traj_data_real_len();

        const std::vector<SharedMemoryInfo> shm_readwrite_infos = {
            {"data_from_simulink_start", sizeof(int8_t), 1},
            {"data_from_simulink_reset", sizeof(int8_t), 1},
            {"data_from_simulink_stop", sizeof(int8_t), 1},
            {"readonly_mode", sizeof(int8_t), 1},
            {"read_traj_length", sizeof(casadi_uint), 1},
            {"read_traj_data_full", 19 * sizeof(casadi_real), 20000},
            {"read_frequency_full", sizeof(casadi_real), 20000},
            {"read_state_data_full", sizeof(casadi_real) * robot_config.nx, 20000},
            {"read_control_data_full", sizeof(casadi_real) * robot_config.nq, 20000},
            {"data_from_python", sizeof(casadi_real) * robot_config.nq, 1}};

        const std::vector<std::string> sem_readwrite_names = {
            "shm_changed_semaphore",
        };
        SharedMemory shm;

        bool semaphore_initialized = false;
        sem_t solve_start_semaphore;

        double current_frequency = 0.0;
        TicToc timer_mpc_solver;
        TicToc timer_all;

        const Eigen::MatrixXd *current_trajectory;

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
        nlohmann::json read_config(std::string file_path);

        Eigen::VectorXd filter_x_measured();
        #ifdef SIMULATION_MODE
        Eigen::VectorXd generateNoiseVector(int n, double Ts, double mean_noise_amplitude);
        #endif
        void init_filter(double omega_c_q, double omega_c_dq);
        void init_controller();
        void reset_mpc_trajectory();

        void init_semaphores();
        void destroy_semaphores();
    };

    // #ifdef SIMULATION_MODE
    // Eigen::VectorXd generateNoiseVector(int n, double Ts, double mean_noise_amplitude);
    // #endif
} // namespace franka_example_controllers