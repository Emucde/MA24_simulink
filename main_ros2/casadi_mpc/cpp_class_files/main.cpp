#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy

#include "include/FullSystemTorqueMapper.hpp"
#include "include/CasadiMPC.hpp"
#include "include/CasadiController.hpp"
#include "mpc_config_types.h"
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
#include "WorkspaceController.hpp"
#include "CrocoddylController.hpp"

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
    bool use_mpc = false; // Set to true to use MPC, false to use classic controller

    // Measure execution time
    TicToc timer_mpc_solver;
    TicToc timer_total;

    // Configuration flags
    bool use_gravity = false;
    // MASTERDIR defined in CMakeLists.txt (=$masterdir)
    const std::string urdf_filename = std::string(MASTERDIR) + "/urdf_creation/fr3_no_hand_7dof.urdf";
    const std::string crocoddyl_config_filename = std::string(MASTERDIR) + "/utils_python/mpc_weights_crocoddyl.json";
    const std::string tcp_frame_name = "fr3_link8_tcp";
    
    robot_config_t robot_config = get_robot_config();

    ErrorFlag error_flag = ErrorFlag::NO_ERROR;
    double Ts = 0.001;
    const casadi_uint nq = robot_config.nq;
    const casadi_uint nx = robot_config.nx;
    const casadi_uint nq_red = robot_config.nq_red;
    const casadi_uint nx_red = robot_config.nx_red;
    const casadi_uint *n_indices_ptr = robot_config.n_indices;
    const casadi_uint *n_x_indices_ptr = robot_config.n_x_indices;
    Eigen::VectorXi n_indices_eig = ConstIntVectorMap(n_indices_ptr, nq_red);
    Eigen::VectorXi n_x_indices_eig = ConstIntVectorMap(n_x_indices_ptr, nx_red);
    casadi_real x_k_ndof[nx] = {0};
    const casadi_real* x0_init;
    double current_frequency = 0.0;
    casadi_uint traj_len = 0;
    Eigen::Map<Eigen::VectorXd> q_k_ndof_eig(x_k_ndof, nq);
    Eigen::Map<Eigen::VectorXd> x_k_ndof_eig(x_k_ndof, nx);
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);

    WorkspaceController classic_controller(urdf_filename, tcp_frame_name, use_gravity);
    CasadiController mpc_controller(urdf_filename, tcp_frame_name, use_gravity);
    CrocoddylController crocoddyl_controller(urdf_filename, crocoddyl_config_filename, tcp_frame_name, use_gravity);
    mpc_controller.setActiveMPC(MPCType::MPC8);
    classic_controller.switchController(ControllerType::InverseDynamics);
    // initialize the trajectory
    
    // set singularity robustness mode
    Eigen::MatrixXd W_bar_N = Eigen::MatrixXd::Identity(nq_red, nq_red);
    Eigen::Map<const Eigen::VectorXd>sugihara_limb_vector(robot_config.sugihara_limb_vector, nq);
    W_bar_N.diagonal() << sugihara_limb_vector(n_indices_eig);
    // Eigen::Map<Eigen::VectorXd>(W_bar_N.diagonal().data(), nq_red) = Eigen::Map<const Eigen::VectorXd>(robot_config.sugihara_limb_vector, robot_config.nq)(n_indices);
    Eigen::MatrixXd W_E = Eigen::MatrixXd::Identity(nq_red, nq_red);

    // PD CONTROLLER
    Eigen::MatrixXd K_d_pd = Eigen::DiagonalMatrix<double, 6>(100, 100, 100, 20, 20, 20);

    // CT CONTROLLER
    Eigen::MatrixXd Kp1_ct = Eigen::DiagonalMatrix<double, 6>(100, 100, 100, 50, 50, 50);

    // ID CONTROLLER
    Eigen::MatrixXd K_d_id = Eigen::DiagonalMatrix<double, 6>(100, 100, 100, 20, 20, 20);
    Eigen::MatrixXd Kp1_id = Eigen::DiagonalMatrix<double, 6>(100, 100, 100, 50, 50, 50); // PD gains

    ControllerSettings ctrl_settings = {
        .pd_plus_settings = {
            .D_d = 2*K_d_pd.array().sqrt(),
            .K_d = K_d_pd
        },
        .ct_settings = {
            .Kd1 = (2*Kp1_ct).array().sqrt(),
            .Kp1 = Kp1_ct
        },
        .id_settings = {
            .Kd1 = (2*Kp1_id).array().sqrt(),
            .Kp1 = Kp1_id,
            .D_d = (2*K_d_id).array().sqrt(),
            .K_d = K_d_id
        },
        .regularization_settings = {
            .mode = RegularizationMode::Damping,
            .k = 1e-5, // RegularizationMode::Damping
            .W_bar_N = W_bar_N, // Sugihara Method, not implemented
            .W_E = W_E, // Sugihara Method, not implemented
            .eps = 1e-1, // ThresholdSmallSingularValues, TikhonovRegularization, RegularizationBySingularValues
            .eps_collinear = 1e-5, // RegularizationMode::SimpleCollinearity
            .lambda_min = 1e-5 // RegularizationMode::SteinboeckCollinearity
        }
    };

    classic_controller.set_controller_settings(ctrl_settings);
    
    #define TRAJ_SELECT 1
    if(use_mpc)
    {
        traj_len = mpc_controller.get_traj_data_real_len();
        x0_init = mpc_controller.get_traj_x0_init(TRAJ_SELECT);
    }
    else
    {
        traj_len = classic_controller.get_traj_data_real_len();
        x0_init = classic_controller.get_traj_x0_init(TRAJ_SELECT);
    }

    x_k_ndof_eig = Eigen::Map<const Eigen::VectorXd> (x0_init, nx);
    // q_k_ndof_eig(n_indices_eig) += Eigen::VectorXd::Constant(nq_red, 0.1);
    // q_k_ndof_eig += Eigen::VectorXd::Constant(nq, 0.1);

    // ParamPolyTrajectory param_target;
    // param_target.p_target = Eigen::Vector3d(0.5, 0.0, 0.6);
    // param_target.R_target = Eigen::Matrix3d::Identity();
    // param_target.T_horizon_max = 2.0
    // param_target.x_init = x_k_ndof_eig;
    // mpc_controller.init_trajectory_custom_target(param_target);

    if(use_mpc)
    {
        mpc_controller.init_file_trajectory(TRAJ_SELECT, x_k_ndof, 0.0, 1.0, 2.0);
    }
    else
    {
        classic_controller.init_file_trajectory(TRAJ_SELECT, x_k_ndof, 0.0, 0.0, 0.0);
    }

    // initialize the filter
    // SignalFilter filter(nq, Ts, x_k_ndof, 400, 400); // int num_joints, double Ts, double *state, double omega_c_q, double omega_c_dq
    // double* x_filtered_ptr = filter.getFilteredOutputPtr();
    // double* x_measured_ptr = x_k_ndof;
    const double* act_data;

    casadi_uint transient_traj_len = mpc_controller.get_transient_traj_len();

    // create shared memory with size
    const std::vector<SharedMemoryInfo> shm_readwrite_infos = {
        {"data_from_simulink_start", sizeof(int8_t), 1},
        {"data_from_simulink_reset", sizeof(int8_t), 1},
        {"data_from_simulink_stop", sizeof(int8_t), 1},
        {"readonly_mode", sizeof(int8_t), 1},
        {"read_traj_length", sizeof(casadi_uint), 1},
        {"read_traj_data_full", 19 * sizeof(casadi_real), traj_len},
        {"read_frequency_full", sizeof(casadi_real), traj_len},
        {"read_state_data_full", sizeof(casadi_real) * robot_config.nx, traj_len},
        {"read_control_data_full", sizeof(casadi_real) * robot_config.nq, traj_len}
    };

    const std::vector<std::string> sem_readwrite_names = {
        "shm_changed_semaphore",
    };

    SharedMemory shm;
    shm.open_readwrite_shms(shm_readwrite_infos);
    shm.open_readwrite_sems(sem_readwrite_names);

    // enable shm read mode:
    int8_t readonly_mode = 1, start = 1;

    shm.write("readonly_mode", &readonly_mode);
    shm.write("read_traj_length", &traj_len);
    shm.write("data_from_simulink_start", &start);
    
    // Start measuring time
    timer_total.tic();

    // Main loop for trajectory processing
    for (casadi_uint i = 0; i < traj_len; i++)
    {
        // filter.run(x_measured_ptr); // updates data from x_filtered_ptr

        if(use_mpc)
        {
            timer_mpc_solver.tic();
            tau_full = mpc_controller.solveMPC(x_k_ndof);
            timer_mpc_solver.toc();
        }
        else
        {
            timer_mpc_solver.tic();
            tau_full = classic_controller.update(x_k_ndof);
            timer_mpc_solver.toc();
        }


        if (i % 100 == 0)
        {
            timer_mpc_solver.print_frequency("\tOUT,i=" + std::to_string(i));
            std::cout << "q_k: " << q_k_ndof_eig.transpose() << std::endl;
            std::cout << "\tFull torque: " << tau_full.transpose() << std::endl;
        }
        if (i == transient_traj_len)
        {
            std::cout << "Switching to trajectory from data" << std::endl;
        }

        // simulate the model
        if(use_mpc)
        {
            act_data = mpc_controller.get_act_traj_data();
            error_flag = mpc_controller.get_error_flag();
        }
        else
        {
            act_data = classic_controller.get_act_traj_data();
            error_flag = classic_controller.get_error_flag();
        }
        current_frequency = timer_mpc_solver.get_frequency();

        // Write data to shm:
        shm.write("read_state_data_full", x_k_ndof, i);
        shm.write("read_control_data_full", tau_full.data(), i);
        shm.write("read_traj_data_full", act_data, i);
        shm.write("read_frequency_full", &current_frequency, i);
        shm.post_semaphore("shm_changed_semaphore");

        // short delay
        // std::this_thread::sleep_for(std::chrono::microseconds(500));
        
        if(use_mpc)
        {
            mpc_controller.simulateModelRK4(x_k_ndof, tau_full.data(), Ts);
        }
        else
        {
            classic_controller.simulateModelRK4(x_k_ndof, tau_full.data(), Ts);
        }

        if (error_flag != ErrorFlag::NO_ERROR)
        {
            std::cerr << "Error flag: " << static_cast<int>(error_flag) << std::endl;
            break;
        }
    }

    // Measure and print execution time
    timer_mpc_solver.print_time("Total mpc_controller.solveMPC time: ");
    std::cout << std::endl;

    timer_total.toc();
    timer_total.print_time("Total execution time: ");
    std::cout << std::endl;

    shm.write("data_from_simulink_reset", &start);
    shm.post_semaphore("shm_changed_semaphore");
    shm.close_shared_memories();
    shm.close_semaphores();
    return 0;
}