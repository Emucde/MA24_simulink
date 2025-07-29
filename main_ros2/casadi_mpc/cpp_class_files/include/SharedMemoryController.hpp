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

/*
This class is a special controller that implements a standalone Shared Memory Controller. It get's
its data from ROS or Simulink and writes the control data back to ROS or Simulink.
It can be also used for simulation purposes.
*/
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
    void init_default_config();
    void update_config();

    // Method to init the filter
    void init_filter(Eigen::VectorXd &x_nq);

    // Method to initialize the trajectory
    void init_trajectory(Eigen::VectorXd &x_nq);

    // Method to init shm
    void init_shm();

    // Method to init python for data reading
    void init_python();

    void reset_cpp_shm_flags();

    void run_simulation();
    void run_shm_mode();
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
    uint traj_step;
    double current_frequency;

    // Trajectory Settings
    uint traj_len;
    uint trajectory_selection;
    double T_traj_start;
    double T_traj_dur;
    double T_traj_end;
    double transient_traj_len;
    const double* act_data;
    uint traj_count;


    // Input and Output Variables
    Eigen::VectorXd x0_nq_init;
    Eigen::VectorXd x0_nq_red_init;
    Eigen::VectorXd x_measured;
    Eigen::VectorXd x_measured_red;
    Eigen::VectorXd tau_full;
    
    // Filter Settings
    bool use_lowpass_filter;
    double omega_c_q;
    double omega_c_dq;
    SignalFilter lowpass_filter;

    bool use_ekf;
    CasadiEKF ekf;
    
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
        {"start_cpp", sizeof(int8_t), 1},
        {"stop_cpp", sizeof(int8_t), 1},
        {"reset_cpp", sizeof(int8_t), 1},
        {"error_cpp", sizeof(int8_t), 1},
        {"valid_cpp", sizeof(int8_t), 1},
        {"traj_switch_cpp", sizeof(int8_t), 1},
        {"data_from_simulink_stop", sizeof(int8_t), 1},
        {"readonly_mode", sizeof(int8_t), 1},
        {"read_traj_length", sizeof(casadi_uint), 1},
        {"read_traj_data_full", 19 * sizeof(double), 20000},
        {"read_frequency_full", sizeof(double), 20000},
        {"read_state_data_full", sizeof(double) * robot_config.nx, 20000},
        {"read_control_data_full", sizeof(double) * robot_config.nq, 20000},
        {"read_control_data", sizeof(double) * robot_config.nq, 1},
        {"cpp_control_data", sizeof(double) * robot_config.nq, 1},
        {"ros2_state_data", sizeof(double) * robot_config.nx, 1}};

    const std::vector<std::string> sem_readwrite_names = {
        "shm_changed_semaphore",
        "ros2_semaphore"
    };

    // get controller type of crocoddyl controller
    MainControllerType get_controller_type(const std::string &controller_type_str);
    CrocoddylMPCType get_crocoddyl_controller_type(const std::string &controller_type_str);
    Eigen::VectorXd generateNoiseVector(int n, double Ts, double mean_noise_amplitude);

    void reset_semaphore();
    bool read_signals();
    void shm_mode_init();
    void print_status(int& print_counter, const Eigen::Map<Eigen::VectorXd>& q_k_ndof_eig);
    void check_for_errors();
    void update_shared_memory(const Eigen::Ref<const Eigen::VectorXd>& read_state);
    void check_signals();

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

    // shm mode data
    int8_t start_cpp, stop_cpp, reset_cpp, valid_cpp, error_flag_int8;
    bool run_flag, first_run, data_valid;
    sem_t *ros2_semaphore;
    char* valid_cpp_shm;

    class BaseFilter
    {
    public:
        BaseFilter(double* x_nq_in) : x_nq_in(x_nq_in) {}
        virtual void run_filter() = 0;
        virtual double* get_filtered_data_ptr() = 0;
        virtual ~BaseFilter() = default;
    protected:
        double* x_nq_in;
    };

    class LowpassFilter : public BaseFilter
    {
    public:
        LowpassFilter(SignalFilter& lowpass_filter, double* x_nq_in) : BaseFilter(x_nq_in), lowpass_filter(lowpass_filter)
        {
            lowpass_filter.reset_state(x_nq_in);
            x_filtered_lowpass_ptr = lowpass_filter.getFilteredOutputPtr();
        }
        void run_filter() override
        {
            lowpass_filter.run(x_nq_in);
        }
        double* get_filtered_data_ptr() override
        {
            return x_filtered_lowpass_ptr;
        }
    protected:
        SignalFilter& lowpass_filter;
        double* x_filtered_lowpass_ptr;
    };

    class EKF_Filter : public BaseFilter
    {
    public:
        EKF_Filter(CasadiEKF& ekf, double* x_nq_in, double* u_nq_in) : BaseFilter(x_nq_in), ekf(ekf), u_nq_in(u_nq_in)
        {
            ekf.initialize(x_nq_in);
            x_filtered_ekf_ptr = ekf.get_x_k_plus_ptr();
        }
        void run_filter() override
        {
            ekf.predict(u_nq_in, x_nq_in);
        }
        double* get_filtered_data_ptr() override
        {
            return x_filtered_ekf_ptr;
        }
    protected:
        CasadiEKF& ekf;
        double* u_nq_in;
        double* x_filtered_ekf_ptr;
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
            x_filtered_ekf_ptr = ekf.get_x_k_plus_ptr();
        }
        void run_filter() override
        {
            lowpass_filter.run(x_nq_in);
            ekf.predict(u_nq_in, x_filtered_lowpass_ptr);
        }
        double* get_filtered_data_ptr() override
        {
            return x_filtered_ekf_ptr;
        }
    protected:
        CasadiEKF& ekf;
        SignalFilter& lowpass_filter;
        double* u_nq_in;
        double* x_filtered_lowpass_ptr;
        double* x_filtered_ekf_ptr;
    };

    class NoFilter : public BaseFilter
    {
    public:
        NoFilter(double* x_nq_in) : BaseFilter(x_nq_in) {}
        void run_filter() override
        {
            // do nothing
        }
        double* get_filtered_data_ptr() override
        {
            return x_nq_in;
        }
    };

    EKF_Filter base_ekf_filter;
    LowpassFilter base_lowpass_filter;
    LPEKF_Filter base_lpekf_filter;
    NoFilter base_no_filter;
    BaseFilter* base_filter;
};

#endif // SHAREDMEMORYCONTROLLER_HPP