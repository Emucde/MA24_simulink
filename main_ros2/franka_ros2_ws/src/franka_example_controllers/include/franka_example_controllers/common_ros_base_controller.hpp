#ifndef FRANKA_EXAMPLE_CONTROLLERS_COMMON_BASE_CONTROLLER_HPP
#define FRANKA_EXAMPLE_CONTROLLERS_COMMON_BASE_CONTROLLER_HPP

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

#define SIMULATION_MODE 1

namespace franka_example_controllers
{

    struct shm_flags
    {
        int8_t start = 0;
        int8_t reset = 0;
        int8_t stop = 0;
        int8_t torques_valid = 0;
    };

    class CommonROSBaseController
    {
    public:
        CommonROSBaseController() {}

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
        Eigen::VectorXd x_filtered = Eigen::VectorXd::Zero(nx);
        double *x_filtered_ekf_ptr = x_measured.data();
        double *x_filtered_lowpass_ptr = x_measured.data();

        CommonBaseController *base_controller;
        FullSystemTorqueMapper *torque_mapper;
        uint N_step;
        uint traj_len;

        CasadiEKF ekf = CasadiEKF(ekf_config_filename);
        SignalFilter lowpass_filter = SignalFilter(nq, Ts, state.data(), 400, 400);

        ErrorFlag error_flag = ErrorFlag::NO_ERROR;

        const std::vector<SharedMemoryInfo> shm_readwrite_infos = {
            {"data_from_simulink_start", sizeof(int8_t), 1},
            {"data_from_simulink_reset", sizeof(int8_t), 1},
            {"data_from_simulink_stop", sizeof(int8_t), 1},
            {"readonly_mode", sizeof(int8_t), 1},
            {"read_traj_length", sizeof(uint), 1},
            {"read_traj_data_full", 19 * sizeof(double), 20000},
            {"read_frequency_full", sizeof(double), 20000},
            {"read_state_data_full", sizeof(double) * robot_config.nx, 20000},
            {"read_control_data", sizeof(double) * robot_config.nq, 1},
            {"read_control_data_full", sizeof(double) * robot_config.nq, 20000},
            {"data_from_python", sizeof(double) * robot_config.nq, 1}};

        const std::vector<std::string> sem_readwrite_names = {
            "shm_changed_semaphore",
        };
        SharedMemory shm;

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

        Eigen::VectorXd filter_x_measured();
#ifdef SIMULATION_MODE
        Eigen::VectorXd generateNoiseVector(int n, double Ts, double mean_noise_amplitude);
#endif
        void init_filter(double omega_c_q, double omega_c_dq);
        void init_controller();
        void init_trajectory();
        void reset_trajectory();
    };
}

#endif // FRANKA_EXAMPLE_CONTROLLERS_COMMON_BASE_CONTROLLER_HPP