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
#include "CasadiController.hpp"
#include "CrocoddylController.hpp"
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
    class CommonBaseController : public controller_interface::ControllerInterface
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

        const std::string urdf_filename = std::string(MASTERDIR) + "/urdf_creation/fr3_no_hand_7dof.urdf";
        const std::string casadi_mpc_config_filename = std::string(MASTERDIR) + "/config_settings/casadi_mpc_weights_fr3_no_hand_simulink.json";
        const std::string general_config_filename = std::string(MASTERDIR) + "/config_settings/general_settings.json";
        const std::string ekf_config_filename = std::string(MASTERDIR) + "/config_settings/ekf_settings.json";
        const std::string tcp_frame_name = "fr3_link8_tcp";

        ErrorFlag error_flag = ErrorFlag::NO_ERROR;

        bool controller_started = false;
        bool use_gravity = false;
        bool use_lowpass_filter = false;
        bool use_ekf = false;

#ifdef SIMULATION_MODE
        bool use_noise = false;
        double mean_noise_amplitude = 0;
#endif

        // CrocoddylController controller = CrocoddylController(urdf_filename, crocoddyl_config_filename, tcp_frame_name, use_gravity);
        // CasadiController controller = CasadiController(urdf_filename, casadi_mpc_config_filename, tcp_frame_name, use_gravity, use_planner);
        // WorkspaceController controller = WorkspaceController(urdf_filename, tcp_frame_name, use_gravity);

        class Controller
        {
        public:
            Controller(const std::string &general_config_file,
                       const std::string &urdf_path,
                       const std::string &tcp_frame_name,
                       bool use_gravity) : general_config_file(general_config_file),
                                           urdf_path(urdf_path),
                                           tcp_frame_name(tcp_frame_name),
                                           use_gravity(use_gravity) {}
                        
                        virtual void set_torque_mapper_config(const std::string &torque_mapper_config) = 0;
                        virtual void switch_controller(int control_type) = 0;
                        virtual void init_file_trajectory(int traj_select, double *state, double T_traj_start, double T_traj_dur, double T_traj_end) = 0;
                        virtual uint get_traj_data_real_len() const = 0;
                        virtual int get_N_step() const = 0;
                        virtual void update_mpc_weights() = 0;
                        virtual int get_traj_step() const = 0;
                        virtual const Eigen::MatrixXd* get_trajectory() const = 0;
                        virtual const Eigen::VectorXd get_traj_x0_red_init(int traj_select) const = 0;
                        virtual void reset(const double *x0_red_init) = 0;
                        virtual const Eigen::VectorXd get_traj_x0_init(int traj_select) const = 0;
                        virtual ErrorFlag get_error_flag() const = 0;
                        virtual void simulateModelRK4(double *state, double *tau_full, double Ts) = 0;
                        virtual void solveMPC(double *x_filtered) = 0;
                        virtual void set_traj_count(uint count) = 0;
                        virtual void switch_traj(int traj_select) = 0;
            virtual void custom_init(nlohmann::json general_config) = 0;

        protected:
            std::string general_config_file; // Shared across all controllers
            std::string urdf_path;
            std::string tcp_frame_name;
            bool use_gravity;
        };

        class ROSCasadiMPC : public Controller, public CasadiController
        {
        public:
            bool use_planner = false;
            std::string casadi_mpc_config_filename = std::string(MASTERDIR) + "/config_settings/casadi_mpc_weights_fr3_no_hand_simulink.json";
            ROSCasadiMPC(const std::string &general_config_file,
                         const std::string &urdf_path,
                         const std::string &tcp_frame_name,
                         bool use_gravity) : Controller(general_config_file, urdf_path, tcp_frame_name, use_gravity),
                         CasadiController(urdf_path, casadi_mpc_config_filename, tcp_frame_name, use_gravity, use_planner) {}
        };

        class ROSCrocoddylMPC : public Controller, public CrocoddylController
        {
        public:
            std::string crocoddyl_config_filename = std::string(MASTERDIR) + "/config_settings/crocoddyl_mpc_settings_fr3_no_hand_simulink.json";
            ROSCrocoddylMPC(const std::string &general_config_file,
                            const std::string &urdf_path,
                            const std::string &tcp_frame_name,
                            bool use_gravity) : Controller(general_config_file, urdf_path, tcp_frame_name, use_gravity),
                            CrocoddylController(urdf_path, crocoddyl_config_filename, tcp_frame_name, use_gravity) {}
        };

        class ROSConventionalController : public Controller, public WorkspaceController
        {
        public:
            ROSConventionalController(const std::string &general_config_file,
                                      const std::string &urdf_path,
                                      const std::string &tcp_frame_name,
                                      bool use_gravity) : Controller(general_config_file, urdf_path, tcp_frame_name, use_gravity),
                                      WorkspaceController(urdf_path, tcp_frame_name, use_gravity) {}
        };

        std::unique_ptr<Controller> controller = std::make_unique<ROSCasadiMPC>(general_config_filename, urdf_filename, tcp_frame_name, use_gravity);

        Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);
        Eigen::VectorXd x_prev = Eigen::VectorXd::Zero(nx);
        Eigen::VectorXd x_measured = Eigen::VectorXd::Zero(nx);
        Eigen::VectorXd x_filtered = Eigen::VectorXd::Zero(nx);
        double *x_filtered_ekf_ptr = x_measured.data();
        double *x_filtered_lowpass_ptr = x_measured.data();
        double *u_opt_full_ptr = controller.get_u_opt_full();

        int solver_steps = controller.get_traj_step();
        int solver_step_counter = 0;
        uint traj_len = controller.get_traj_data_real_len();
        uint global_traj_count = 0;
        int8_t traj_select = 1; // default trajectory
        const Eigen::MatrixXd *current_trajectory;

        FullSystemTorqueMapper *torque_mapper = controller.get_torque_mapper();

        CasadiEKF ekf = CasadiEKF(ekf_config_filename);
        SignalFilter lowpass_filter = SignalFilter(nq, Ts, state.data(), 400, 400);

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

        TicToc timer_mpc_solver;
        TicToc timer_all;

        void open_shared_memories();
        void close_shared_memories();
        nlohmann::json read_config(std::string file_path);
        Eigen::VectorXd filter_x_measured();
        void init_filter(double omega_c_q, double omega_c_dq);
        void init_controller();
        void reset_controller_trajectory();

#ifdef SIMULATION_MODE
        Eigen::VectorXd generateNoiseVector(int n, double Ts, double mean_noise_amplitude);
#endif

        // rclcpp::Subscription<mpc_interfaces::msg::ControlArray>::SharedPtr subscription_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr start_controller_service_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr reset_controller_service_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr stop_controller_service_;
        rclcpp::Service<mpc_interfaces::srv::TrajectoryCommand>::SharedPtr traj_switch_service_;
        rclcpp::Service<mpc_interfaces::srv::CasadiMPCTypeCommand>::SharedPtr mpc_switch_service_;

        // void topic_callback(const mpc_interfaces::msg::ControlArray & msg);
        void start_controller(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
                              std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response);
        void reset_controller(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
                              std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response);
        void stop_controller(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
                             std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response);
        void traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
                         std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Response> response);
        void mpc_switch(const std::shared_ptr<mpc_interfaces::srv::CasadiMPCTypeCommand::Request> request,
                        std::shared_ptr<mpc_interfaces::srv::CasadiMPCTypeCommand::Response> response);
    };
} // namespace franka_example_controllers