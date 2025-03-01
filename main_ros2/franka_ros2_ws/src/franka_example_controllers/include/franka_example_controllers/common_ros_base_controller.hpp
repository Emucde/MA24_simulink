#ifndef FRANKA_EXAMPLE_CONTROLLERS_COMMON_BASE_CONTROLLER_HPP
#define FRANKA_EXAMPLE_CONTROLLERS_COMMON_BASE_CONTROLLER_HPP

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

#include "CommonBaseController.hpp"
#include "TicToc.hpp"
#include "SignalFilter.hpp"
#include "SharedMemory.hpp"
#include "CasadiController.hpp"
#include "CasadiEKF.hpp"
#include "FullSystemTorqueMapper.hpp"
#include "TicToc.hpp"
#include "param_robot.h"
#include "trajectory_settings.hpp"
#include <Eigen/Dense>
#include <random>

// #define MY_LOG_LEVEL RCUTILS_LOG_SEVERITY_WARN
#define MY_LOG_LEVEL RCUTILS_LOG_SEVERITY_INFO

#define MAX_INVALID_COUNT 100

#define SIMULATION_MODE 1
// #define CUSTOM_LIST 1

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

    class CommonROSBaseController : public controller_interface::ControllerInterface
    {
    public:
        CommonROSBaseController() {}

        FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
        virtual controller_interface::return_type update(const rclcpp::Time &time,
                                                         const rclcpp::Duration &period) override;

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

        void solve();
        virtual void open_shared_memories();
        virtual void close_shared_memories();
    protected:
        std::string arm_id_;
        robot_config_t robot_config = get_robot_config();
        double Ts = robot_config.dt;
        const int nq = robot_config.nq;
        const int nx = robot_config.nx;
        Eigen::VectorXd state = Eigen::VectorXd::Zero(nq * 2);
        int invalid_counter = 0; // counter for invalid data, if exceeds MAX_INVALID_COUNT, terminate the controller
        uint global_traj_count = 0;
        bool controller_started = false;
        bool first_start = true;
        int8_t traj_select = 1; // default trajectory
        const std::string urdf_filename = std::string(MASTERDIR) + "/urdf_creation/fr3_no_hand_7dof.urdf";
        const std::string general_config_filename = std::string(MASTERDIR) + "/config_settings/general_settings.json";
        const std::string ekf_config_filename = std::string(MASTERDIR) + "/config_settings/ekf_settings.json";
        const std::string tcp_frame_name = "fr3_link8_tcp";
        bool use_gravity = false;
        bool use_lowpass_filter = false;
        bool use_ekf = false;
        int solver_steps = 1;
        int solver_step_counter = 0;
#ifdef SIMULATION_MODE
        bool use_noise = false;
        double mean_noise_amplitude = 0;
#endif

        Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);
        Eigen::VectorXd x_prev = Eigen::VectorXd::Zero(nx);
        Eigen::VectorXd x_measured = Eigen::VectorXd::Zero(nx);

        CommonBaseController *base_controller;
        FullSystemTorqueMapper *torque_mapper;
        uint N_step;
        uint traj_len;

        CasadiEKF ekf = CasadiEKF(ekf_config_filename);
        SignalFilter lowpass_filter = SignalFilter(nq, x_measured.data(), general_config_filename);

        ErrorFlag error_flag = ErrorFlag::NO_ERROR;

        const std::vector<SharedMemoryInfo> shm_readwrite_infos = {
            {"data_from_simulink_start", sizeof(int8_t), 1},
            {"data_from_simulink_reset", sizeof(int8_t), 1},
            {"data_from_simulink_stop", sizeof(int8_t), 1},
            {"readonly_mode", sizeof(int8_t), 1},
            {"start_cpp", sizeof(int8_t), 1},
            {"stop_cpp", sizeof(int8_t), 1},
            {"reset_cpp", sizeof(int8_t), 1},
            {"error_cpp", sizeof(int8_t), 1},
            {"valid_cpp", sizeof(int8_t), 1},
            {"read_traj_length", sizeof(uint), 1},
            {"read_traj_data_full", 19 * sizeof(double), 20000},
            {"read_frequency_full", sizeof(double), 20000},
            {"read_state_data_full", sizeof(double) * robot_config.nx, 20000},
            {"read_control_data", sizeof(double) * robot_config.nq, 1},
            {"cpp_control_data", sizeof(double) * robot_config.nq, 1},
            {"ros2_state_data", sizeof(double) * robot_config.nx, 1},
            {"read_control_data_full", sizeof(double) * robot_config.nq, 20000},
            {"data_from_python", sizeof(double) * robot_config.nq, 1}};

        const std::vector<std::string> sem_readwrite_names = {
            "shm_changed_semaphore",
            "ros2_semaphore"
        };
        SharedMemory shm;
        sem_t *ros2_semaphore;
        char* valid_cpp_shm;

        double current_frequency = 0.0;
        TicToc timer_solver;
        TicToc timer_all;

        const Eigen::MatrixXd *current_trajectory;

        void init_base_controller()
        {
            torque_mapper = base_controller->get_torque_mapper();
            N_step = base_controller->get_N_step();
            traj_len = base_controller->get_traj_data_real_len();
        }

        nlohmann::json read_config(std::string file_path);

        Eigen::VectorXd generateNoiseVector(int n, double Ts, double mean_noise_amplitude);
        void init_filter(Eigen::VectorXd &x_nq);
        void init_controller();
        void init_trajectory();
        void reset();

        // method to write valid_cpp_shm
        void write_valid_cpp_shm(int8_t flags)
        {
            memcpy(valid_cpp_shm, &flags, sizeof(int8_t));
        }

        //method to read valid_cpp_shm
        int8_t read_valid_cpp_shm()
        {
            int8_t flags;
            memcpy(&flags, valid_cpp_shm, sizeof(int8_t));
            return flags;
        }

        // rclcpp::Subscription<mpc_interfaces::msg::ControlArray>::SharedPtr subscription_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr start_mpc_service_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr reset_mpc_service_;
        rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr stop_mpc_service_;
        rclcpp::Service<mpc_interfaces::srv::TrajectoryCommand>::SharedPtr traj_switch_service_;

        // void topic_callback(const mpc_interfaces::msg::ControlArray & msg);
        virtual void start_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
            std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response);
        virtual void reset_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
            std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response);
        virtual void stop_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
            std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response);
        virtual void traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
            std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Response> response);

        class BaseFilter
        {
        public:
            BaseFilter(double* x_nq_in) : x_nq_in(x_nq_in) {}
            virtual void run_filter() = 0;
            virtual double* get_filtered_data_ptr()
            {
                return x_filtered_ptr;
            }
            virtual ~BaseFilter() = default;
        protected:
            double* x_nq_in;
            double* x_filtered_ptr;
        };
    
        class LowpassFilter : public BaseFilter
        {
        public:
            LowpassFilter(SignalFilter& lowpass_filter, double* x_nq_in) : BaseFilter(x_nq_in), lowpass_filter(lowpass_filter)
            {
                lowpass_filter.reset_state(x_nq_in);
                x_filtered_ptr = lowpass_filter.getFilteredOutputPtr();
            }
            void run_filter() override
            {
                lowpass_filter.run(x_nq_in);
            }
        protected:
            SignalFilter& lowpass_filter;
        };
    
        class EKF_Filter : public BaseFilter
        {
        public:
            EKF_Filter(CasadiEKF& ekf, double* x_nq_in, double* u_nq_in) : BaseFilter(x_nq_in), ekf(ekf), u_nq_in(u_nq_in)
            {
                ekf.initialize(x_nq_in);
                x_filtered_ptr = ekf.get_x_k_plus_ptr();
            }
            void run_filter() override
            {
                ekf.predict(u_nq_in, x_nq_in);
            }
        protected:
            CasadiEKF& ekf;
            double* u_nq_in;
        };
    
        class LPEKF_Filter : public BaseFilter
        {
        public:
            LPEKF_Filter(CasadiEKF& ekf, SignalFilter& lowpass_filter, double* x_nq_in, double* u_nq_in) : BaseFilter(x_nq_in),
                                                                                                        ekf(ekf),
                                                                                                        lowpass_filter(lowpass_filter),
                                                                                                        u_nq_in(u_nq_in)
            {
                lowpass_filter.reset_state(x_nq_in);
                ekf.initialize(x_nq_in);
                x_filtered_lowpass_ptr = lowpass_filter.getFilteredOutputPtr();
                x_filtered_ptr = ekf.get_x_k_plus_ptr();
            }
            void run_filter() override
            {
                lowpass_filter.run(x_nq_in);
                ekf.predict(u_nq_in, x_filtered_lowpass_ptr);
            }
        protected:
            CasadiEKF& ekf;
            SignalFilter& lowpass_filter;
            double* u_nq_in;
            double* x_filtered_lowpass_ptr;
        };
    
        class NoFilter : public BaseFilter
        {
        public:
            NoFilter(double* x_nq_in) : BaseFilter(x_nq_in) {
                x_filtered_ptr = x_nq_in;
            }
            void run_filter() override
            {
                // do nothing
            }
        };
    
        EKF_Filter base_ekf_filter = EKF_Filter(ekf, x_measured.data(), tau_full.data());
        LowpassFilter base_lowpass_filter = LowpassFilter(lowpass_filter, x_measured.data());
        LPEKF_Filter base_lpekf_filter = LPEKF_Filter(ekf, lowpass_filter, x_measured.data(), tau_full.data());
        NoFilter base_no_filter = NoFilter(x_measured.data());
        BaseFilter* base_filter = &base_no_filter;
        double* x_filtered_ptr = 0;
    };
}

#endif // FRANKA_EXAMPLE_CONTROLLERS_COMMON_BASE_CONTROLLER_HPP