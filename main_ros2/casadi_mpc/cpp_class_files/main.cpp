#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy

#include "include/FullSystemTorqueMapper.hpp"
#include "include/CasadiMPC.hpp"
#include "include/CasadiController.hpp"
#include "mpc_config.h"
#include "param_robot.h"
#include "casadi_types.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include "include/eigen_templates.hpp"
#include "include/TicToc.hpp"
#include "include/SignalFilter.hpp"
#include "include/SharedMemory.hpp"
#include "param_robot.h"
#include "trajectory_settings.hpp"

casadi_real x_k_tst[12] = {0};

void test_fun1(const casadi_real* x_k_ndof, const casadi_uint *n_x_indices_ptr)
{
    casadi_uint nx_red = 12;
    
    double x_k[nx_red];
    for (casadi_uint i = 0; i < nx_red; i++)
    {
        x_k[i] = x_k_ndof[n_x_indices_ptr[i]];
    }
    //0.01s
    memcpy(x_k_tst, x_k, nx_red*sizeof(casadi_real));
}

void test_fun2(const casadi_real* x_k_ndof, const Eigen::VectorXi n_x_indices_eig)
{
    casadi_uint nx_red = 12;

    // Eigen::Map<Eigen::VectorXd>(x_k, nx_red) = x_k_ndof_eig(n_x_indices_eig);

    // double x_k[nx_red];
    // Eigen::Map<Eigen::VectorXd>(x_k, nx_red) = Eigen::Map<const Eigen::VectorXd>(x_k_ndof, nx)(n_x_indices_eig);
    // memcpy(x_k_tst, x_k, nx_red*sizeof(casadi_real));
    // 0.016s

    // Eigen::VectorXd x_k_ndof_tst = Eigen::Map<const Eigen::VectorXd>(x_k_ndof, nx);
    // memcpy(x_k_tst, x_k_ndof_tst(n_x_indices_eig).eval().data(), nx_red*sizeof(casadi_real));
    // 0.02s

    // Eigen::VectorXd x_k_ndof_tst = Eigen::Map<const Eigen::VectorXd>(x_k_ndof, nx);
    // Eigen::VectorXd selected_indices = x_k_ndof_tst(n_x_indices_eig);
    // memcpy(x_k_tst, selected_indices.data(), nx_red*sizeof(casadi_real));
    // 0.022s

    double x_k[nx_red];
    for (casadi_uint i = 0; i < nx_red; i++)
    {
        x_k[i] = x_k_ndof[n_x_indices_eig[i]];
    }
    memcpy(x_k_tst, x_k, nx_red*sizeof(casadi_real));
    //0.01s
}

int main()
{
    // Measure execution time
    TicToc timer_mpc_solver;
    TicToc timer_total;

    // Configuration flags
    bool use_gravity = false;
    const std::string urdf_filename = std::string(MASTERDIR) + "/urdf_creation/fr3_no_hand_7dof.urdf";
    const std::string tcp_frame_name = "fr3_link8_tcp";

    CasadiController controller(urdf_filename, tcp_frame_name, use_gravity);
    controller.setActiveMPC(MPCType::MPC8);
    
    SharedMemory shm;
    const std::vector<std::string> shm_readwrite_names = {
        "data_from_simulink_start",
        "data_from_simulink_reset",
        "data_from_simulink_stop",
        "readonly_mode",
        "read_traj_length",
        "read_traj_data",
        "read_frequency",
        "read_state_data",
        "read_control_data"
    };

    const std::vector<std::string> sem_readwrite_names = {
        "shm_changed_semaphore"
    };

    shm.open_readwrite_shms(shm_readwrite_names);
    shm.open_readwrite_sems(sem_readwrite_names);

    ErrorFlag error_flag = ErrorFlag::NO_ERROR;
    double Ts = 0.001;
    const casadi_uint nq = controller.nq;
    const casadi_uint nx = controller.nx;
    const casadi_uint nq_red = controller.nq_red;
    const casadi_uint nx_red = controller.nx_red;
    const casadi_uint *n_indices_ptr = controller.get_n_indices();
    const casadi_uint *n_x_indices_ptr = controller.get_n_x_indices();
    Eigen::VectorXi n_indices_eig = ConstIntVectorMap(n_indices_ptr, nq_red);
    Eigen::VectorXi n_x_indices_eig = ConstIntVectorMap(n_x_indices_ptr, nx_red);
    casadi_real x_k_ndof[nx] = {0};
    double current_frequency = 0.0;

    const casadi_uint traj_len = controller.get_traj_data_real_len();

    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);

    Eigen::Map<Eigen::VectorXd> q_k_ndof_eig(x_k_ndof, nq);
    Eigen::Map<Eigen::VectorXd> x_k_ndof_eig(x_k_ndof, nx);

    #define TRAJ_SELECT 1
    x_k_ndof_eig = Eigen::Map<const Eigen::VectorXd> (controller.get_traj_x0_init(TRAJ_SELECT), nx);
    q_k_ndof_eig(n_indices_eig) += Eigen::VectorXd::Constant(nq_red, 0.1);
    // q_k_ndof_eig += Eigen::VectorXd::Constant(nq, 0.1);

    // initialize the filter
    SignalFilter filter(nq, Ts, x_k_ndof, 400, 400); // int num_joints, double Ts, double *state, double omega_c_q, double omega_c_dq
    double* x_filtered_ptr = filter.getFilteredOutputPtr();
    double* x_measured_ptr = x_k_ndof;

    // initialize the trajectory
    controller.init_file_trajectory(TRAJ_SELECT, x_k_ndof, 0.0, 1.0, 2.0);

    const Eigen::MatrixXd* trajectory = controller.get_trajectory();
    int traj_count = 0;
    Eigen::VectorXi selected_rows(7);
    selected_rows << 0, 1, 2, 9, 10, 11, 12;  // Selecting p_d (0-2) and q_d (9-11)
    Eigen::VectorXd y_d = Eigen::VectorXd::Zero(7);
    
    // ParamPolyTrajectory param_target;
    // param_target.p_target = Eigen::Vector3d(0.5, 0.0, 0.6);
    // param_target.R_target = Eigen::Matrix3d::Identity();
    // param_target.T_horizon_max = 2.0
    // param_target.x_init = x_k_ndof_eig;
    // controller.init_trajectory_custom_target(param_target);

    casadi_uint transient_traj_len = controller.get_transient_traj_len();
    
    // Start measuring time
    timer_total.tic();

    // enable shm read mode:
    int8_t readonly_mode = 1, start = 1;

    shm.write("readonly_mode", &readonly_mode, sizeof(int8_t));
    shm.write("read_traj_length", &traj_len, sizeof(casadi_uint));
    shm.write("data_from_simulink_start", &start, sizeof(int8_t));
    
    // Write data to shm:
    current_frequency = 0;
    shm.write("read_state_data", x_k_ndof, nx * sizeof(casadi_real));
    shm.write("read_control_data", tau_full.data(), nq * sizeof(casadi_real));
    shm.write("read_traj_data", controller.get_act_traj_data(), 7 * sizeof(casadi_real));
    shm.write("read_frequency", &current_frequency, sizeof(double));
    shm.post_semaphore("shm_changed_semaphore");

    // Main loop for trajectory processing
    for (casadi_uint i = 0; i < traj_len; i++)
    {
        filter.run(x_measured_ptr); // updates data from x_filtered_ptr

        timer_mpc_solver.tic();
        // tau_full = controller.solveMPC(x_filtered_ptr);
        tau_full = Eigen::VectorXd::Zero(nq);
        error_flag = controller.get_error_flag();
        timer_mpc_solver.toc();

        y_d << (*trajectory)(selected_rows, traj_count++);

        if (i % 100 == 0)
        {
            std::cout << "q_k: " << q_k_ndof_eig.transpose();
            std::cout << "Full torque: " << tau_full.transpose();
            timer_mpc_solver.print_frequency("\t");
            std::cout << std::endl;
        }
        if (i == transient_traj_len)
        {
            std::cout << "Switching to trajectory from data" << std::endl;
        }

        // simulate the model
        controller.simulateModel(x_k_ndof, tau_full.data(), Ts);
        current_frequency = timer_mpc_solver.get_frequency();

        // Write data to shm:
        shm.write("read_state_data", x_k_ndof, nx * sizeof(casadi_real));
        shm.write("read_control_data", tau_full.data(), nq * sizeof(casadi_real));
        // shm.write("read_traj_data", controller.get_act_traj_data(), 7 * sizeof(casadi_real));
        shm.write("read_traj_data", y_d.data(), 7 * sizeof(casadi_real));
        shm.write("read_frequency", &current_frequency, sizeof(double));
        shm.post_semaphore("shm_changed_semaphore");

        if (error_flag != ErrorFlag::NO_ERROR)
        {
            std::cerr << "Error flag: " << static_cast<int>(error_flag) << std::endl;
            break;
        }
    }

    // Measure and print execution time
    timer_mpc_solver.print_time("Total controller.solveMPC time: ");
    std::cout << std::endl;

    timer_total.toc();
    timer_total.print_time("Total execution time: ");
    std::cout << std::endl;

    shm.write("data_from_simulink_reset", &start, sizeof(int8_t));
    shm.post_semaphore("shm_changed_semaphore");
    shm.close_shared_memories();
    shm.close_semaphores();
    return 0;
}