#ifndef SHAREDMEMORYCONTROLLER_HPP
#define SHAREDMEMORYCONTROLLER_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy

#include "FullSystemTorqueMapper.hpp"
#include "CasadiMPC.hpp"
#include "CasadiController.hpp"
#include "mpc_config_types.h"
#include "param_robot.h"
#include "casadi_types.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include "eigen_templates.hpp"
#include "TicToc.hpp"
#include "SignalFilter.hpp"
#include "SharedMemory.hpp"
#include "param_robot.h"
#include "trajectory_settings.hpp"
#include "WorkspaceController.hpp"
#include "CrocoddylController.hpp"
#include "CrocoddylMPCType.hpp"
#include "CasadiEKF.hpp"
#include "CommonBaseController.hpp"
#include "json.hpp"
#include <random>
#include <cmath>

class SharedMemoryController {
public:
    SharedMemoryController(const std::string &masterDir, bool use_custom_list);
    SharedMemoryController(const std::string& urdf_file, 
                        const std::string& crocoddyl_config_file, 
                        const std::string& ekf_config_file, 
                        const std::string& general_config_file, 
                        const std::string& casadi_mpc_config_file);

    // Destructor
    ~SharedMemoryController();

    enum class MainControllerType
    {
        Classic,
        Casadi,
        Crocoddyl,
        COUNT
    };


    // Method to update the configuration
    void update_config();

    // Method to init the filter
    void init_filter(Eigen::VectorXd &x_nq);

    // Method to initialize the trajectory
    void init_trajectory(Eigen::VectorXd &x_nq);

    // Method to init shm
    void init_shm();

    // Method to init python for data reading
    void init_python();

    void run_simulation();

private:
    std::string urdf_filename;
    std::string crocoddyl_config_filename;
    std::string ekf_config_filename;
    std::string general_config_filename;
    std::string casadi_mpc_config_filename;

    // Robot Settings
    robot_config_t robot_config;
    uint nq, nx, nq_red, nx_red;
    Eigen::VectorXi n_indices;
    Eigen::VectorXi n_x_indices;

    std::string tcp_frame_name;

    // Controller
    WorkspaceController classic_controller;
    CasadiController casadi_controller;
    CrocoddylController crocoddyl_controller;

    MainControllerType active_controller_type;
    CommonBaseController* controller;

    // General Settings
    double Ts;
    uint solver_steps;
    double current_frequency;

    // Trajectory Settings
    uint traj_len;
    double trajectory_selection;
    double T_traj_start;
    double T_traj_dur;
    double T_traj_end;
    double transient_traj_len;


    // Input and Output Variables
    Eigen::VectorXd x0_nq_init;
    Eigen::VectorXd x0_nq_red_init;
    Eigen::VectorXd x_measured;
    Eigen::VectorXd x_filtered;
    Eigen::VectorXd tau_full;
    double* x_filtered_ekf_ptr;
    double* x_filtered_lowpass_ptr;
    
    // Filter Settings
    bool use_lowpass_filter;
    double omega_c_q;
    double omega_c_dq;
    SignalFilter lowpass_filter;
    double* x_lowpass_filtered_ptr;

    bool use_ekf;
    CasadiEKF ekf;
    double* x_ekf_filtered_ptr;
    
    // Simulation Settings
    bool use_noise;
    double mean_noise_amplitude;
    ErrorFlag error_flag;

    // Time measurement    
    TicToc timer_mpc_solver;
    TicToc timer_total;

    // Shared Memory Infos
    SharedMemory shm;
    const std::vector<SharedMemoryInfo> shm_readwrite_infos = {
        {"data_from_simulink_start", sizeof(int8_t), 1},
        {"data_from_simulink_reset", sizeof(int8_t), 1},
        {"data_from_simulink_stop", sizeof(int8_t), 1},
        {"readonly_mode", sizeof(int8_t), 1},
        {"read_traj_length", sizeof(casadi_uint), 1},
        {"read_traj_data_full", 19 * sizeof(double), 20000},
        {"read_frequency_full", sizeof(double), 20000},
        {"read_state_data_full", sizeof(double) * robot_config.nx, 20000},
        {"read_control_data_full", sizeof(double) * robot_config.nq, 20000},
        {"read_control_data", sizeof(double) * robot_config.nq, 1}};

    const std::vector<std::string> sem_readwrite_names = {
        "shm_changed_semaphore",
    };

    // get controller type of crocoddyl controller
    MainControllerType get_controller_type(const std::string &controller_type_str);
    CrocoddylMPCType get_crocoddyl_controller_type(const std::string &controller_type_str);
    Eigen::VectorXd generateNoiseVector(int n, double Ts, double mean_noise_amplitude);
};

#endif // SHAREDMEMORYCONTROLLER_HPP