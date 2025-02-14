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
#include "CrocoddylMPCType.hpp"
#include "CasadiEKF.hpp"
#include "json.hpp"
#include <random>
#include <cmath>

casadi_real x_k_tst[12] = {0};

void test_fun2(const casadi_real *x_k_ndof, const Eigen::VectorXi n_x_indices_eig)
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
    memcpy(x_k_tst, x_k, nx_red * sizeof(casadi_real));
    // 0.01s
}

nlohmann::json read_config(std::string file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: Could not open JSON file." << std::endl;
        return {};
    }
    nlohmann::json jsonData;
    file >> jsonData; // Parse JSON file
    file.close();
    return jsonData;
}

// Function to generate an Eigen vector of white noise
Eigen::VectorXd generateNoiseVector(int n, double Ts, double mean_noise_amplitude) {
    // Calculate noise power
    double noise_power = 1 / (2*Ts) * (Ts / 2) * M_PI * std::pow(mean_noise_amplitude, 2);

    // Initializes random number generator for normal distribution
    std::random_device rd;
    std::mt19937 generator(rd()); // Mersenne Twister random number generator
    std::normal_distribution<double> distribution(0.0, std::sqrt(noise_power)); // Normal distribution

    // Create an Eigen vector to hold the noise
    Eigen::VectorXd white_noise(n);

    // Generate white noise
    for (int i = 0; i < n; ++i) {
        white_noise[i] = distribution(generator);
    }

    return white_noise; // Return the generated noise vector
}

enum class MainControllerType
{
    Classic,
    Casadi,
    Crocoddyl,
    COUNT
};

MainControllerType get_controller_type(const std::string &controller_type_str)
{
    if (controller_type_str == "Classic")
        return MainControllerType::Classic;
    else if (controller_type_str == "Casadi")
        return MainControllerType::Casadi;
    else if (controller_type_str == "Crocoddyl")
        return MainControllerType::Crocoddyl;
    else
    {
        throw std::invalid_argument("Invalid controller type");
        return MainControllerType::COUNT;
    }
}

// get controller type of classic controller
ControllerType get_classic_controller_type(const std::string &controller_type_str)
{
    if (controller_type_str == "PD")
        return ControllerType::PDPlus;
    else if (controller_type_str == "CT")
        return ControllerType::CT;
    else if (controller_type_str == "ID")
        return ControllerType::InverseDynamics;
    else
    {
        throw std::invalid_argument("Invalid controller type");
        return ControllerType::PDPlus;
    }
}

// get controller type of casadi controller


// get controller type of crocoddyl controller
CrocoddylMPCType get_crocoddyl_controller_type(const std::string &controller_type_str)
{
    if (controller_type_str == "DynMPC_v1")
        return CrocoddylMPCType::DynMPC_v1;
    // else if (controller_type_str == "DynMPC_v2")
    //     return CrocoddylMPCType::DynMPC_v2;
    // else if (controller_type_str == "DynMPC_v3")
    //     return CrocoddylMPCType::DynMPC_v3;
    else
    {
        throw std::invalid_argument("Invalid controller type");
        return CrocoddylMPCType::DynMPC_v1;
    }
}

int main()
{
    std::cout << "Starting main loop" << std::endl;
    // Measure execution time
    TicToc timer_mpc_solver;
    TicToc timer_total;

    // MASTERDIR defined in CMakeLists.txt (=$masterdir)
    const std::string urdf_filename = std::string(MASTERDIR) + "/urdf_creation/fr3_no_hand_7dof.urdf";
    const std::string crocoddyl_config_filename = std::string(MASTERDIR) + "/utils_python/mpc_weights_crocoddyl.json";
    const std::string ekf_config_filename = std::string(MASTERDIR) + "/config_settings/ekf_settings.json";
    const std::string general_config_filename = std::string(MASTERDIR) + "/config_settings/general_settings.json";
    const std::string tcp_frame_name = "fr3_link8_tcp";

    nlohmann::json general_config = read_config(general_config_filename);

    // Configuration flags
    bool use_gravity = general_config["use_gravity"];
    bool use_lowpass_filter = general_config["use_lowpass_filter"];
    bool use_ekf = general_config["use_ekf"];
    bool use_noise = general_config["use_noise"];

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
    const casadi_real *x0_init;
    double current_frequency = 0.0;
    casadi_uint traj_len = 0;
    double mean_noise_amplitude = 0.1e-3;
    Eigen::VectorXd x_measured = Eigen::VectorXd::Zero(nx);

    Eigen::Map<Eigen::VectorXd> q_k_ndof_eig(x_k_ndof, nq);
    Eigen::Map<Eigen::VectorXd> x_k_ndof_eig(x_k_ndof, nx);
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);

    WorkspaceController classic_controller(urdf_filename, tcp_frame_name, use_gravity);
    CasadiController casadi_controller(urdf_filename, tcp_frame_name, use_gravity);
    CrocoddylController crocoddyl_controller(urdf_filename, crocoddyl_config_filename, tcp_frame_name, use_gravity);
    
    MainControllerType controller_type = get_controller_type(general_config["default_controller"]);
    casadi_controller.setActiveMPC(string_to_casadi_mpctype(general_config["default_casadi_mpc"]));
    classic_controller.switchController(get_classic_controller_type(general_config["default_classic_controller"]));
    crocoddyl_controller.setActiveMPC(get_crocoddyl_controller_type(general_config["default_crocoddyl_mpc"]));
    // initialize the trajectory

    // set singularity robustness mode
    Eigen::MatrixXd W_bar_N = Eigen::MatrixXd::Identity(nq_red, nq_red);
    Eigen::Map<const Eigen::VectorXd> sugihara_limb_vector(robot_config.sugihara_limb_vector, nq);
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
            .D_d = 2 * K_d_pd.array().sqrt(),
            .K_d = K_d_pd},
        .ct_settings = {.Kd1 = (2 * Kp1_ct).array().sqrt(), .Kp1 = Kp1_ct},
        .id_settings = {.Kd1 = (2 * Kp1_id).array().sqrt(), .Kp1 = Kp1_id, .D_d = (2 * K_d_id).array().sqrt(), .K_d = K_d_id},
        .regularization_settings = {
            .mode = RegularizationMode::Damping,
            .k = 1e-5,             // RegularizationMode::Damping
            .W_bar_N = W_bar_N,    // Sugihara Method, not implemented
            .W_E = W_E,            // Sugihara Method, not implemented
            .eps = 1e-1,           // ThresholdSmallSingularValues, TikhonovRegularization, RegularizationBySingularValues
            .eps_collinear = 1e-5, // RegularizationMode::SimpleCollinearity
            .lambda_min = 1e-5     // RegularizationMode::SteinboeckCollinearity
        }};

    classic_controller.set_controller_settings(ctrl_settings);

#define TRAJ_SELECT 1
    if (controller_type == MainControllerType::Casadi)
    {
        traj_len = casadi_controller.get_traj_data_real_len();
        x0_init = casadi_controller.get_traj_x0_init(TRAJ_SELECT);
    }
    else if(controller_type == MainControllerType::Classic)
    {
        traj_len = classic_controller.get_traj_data_real_len();
        x0_init = classic_controller.get_traj_x0_init(TRAJ_SELECT);
    }
    else if(controller_type == MainControllerType::Crocoddyl)
    {
        traj_len = crocoddyl_controller.get_traj_data_real_len();
        x0_init = crocoddyl_controller.get_traj_x0_init(TRAJ_SELECT);
    }

    x_k_ndof_eig = Eigen::Map<const Eigen::VectorXd>(x0_init, nx);
    // q_k_ndof_eig(n_indices_eig) += Eigen::VectorXd::Constant(nq_red, 0.1);
    // q_k_ndof_eig += Eigen::VectorXd::Constant(nq, 0.1);

    // ParamPolyTrajectory param_target;
    // param_target.p_target = Eigen::Vector3d(0.5, 0.0, 0.6);
    // param_target.R_target = Eigen::Matrix3d::Identity();
    // param_target.T_horizon_max = 2.0
    // param_target.x_init = x_k_ndof_eig;
    // casadi_controller.init_trajectory_custom_target(param_target);

    casadi_uint transient_traj_len = 0;

    if (controller_type == MainControllerType::Casadi)
    {
        casadi_controller.init_file_trajectory(TRAJ_SELECT, x_k_ndof, 0.0, 0.0, 0.0);
        transient_traj_len = casadi_controller.get_transient_traj_len();
    }
    else if (controller_type == MainControllerType::Classic)
    {
        classic_controller.init_file_trajectory(TRAJ_SELECT, x_k_ndof, 0.0, 0.0, 0.0);
        transient_traj_len = classic_controller.get_transient_traj_len();
    }
    else if (controller_type == MainControllerType::Crocoddyl)
    {
        crocoddyl_controller.init_file_trajectory(TRAJ_SELECT, x_k_ndof, 0.0, 0.0, 0.0);
        transient_traj_len = crocoddyl_controller.get_transient_traj_len();
    }

    // initialize EKF
    CasadiEKF ekf(ekf_config_filename);
    ekf.initialize(x_k_ndof);
    double* x_filtered_ptr = ekf.get_x_k_plus_ptr();

    // initialize the filter
    SignalFilter filter(nq, Ts, x_k_ndof, 400, 400); // int num_joints, double Ts, double *state, double omega_c_q, double omega_c_dq
    double* x_filtered_ptr_2 = filter.getFilteredOutputPtr();
    const double *act_data;

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
        {"read_control_data_full", sizeof(casadi_real) * robot_config.nq, traj_len}};

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
        if(use_noise)
            x_measured << x_k_ndof_eig + generateNoiseVector(nx, Ts, mean_noise_amplitude);
        else
            x_measured << x_k_ndof_eig;
        
        if(use_ekf)
            ekf.predict(tau_full.data(), x_measured.data());
        else
            x_filtered_ptr = x_measured.data();
        
        if(use_lowpass_filter)
            filter.run(x_filtered_ptr); // updates data from x_filtered_ptr
        else
            x_filtered_ptr_2 = x_filtered_ptr;
        
        // x_filtered_ptr = x_measured.data();

        if ( controller_type == MainControllerType::Casadi )
        {
            timer_mpc_solver.tic();
            tau_full = casadi_controller.solveMPC(x_filtered_ptr_2);
            timer_mpc_solver.toc();
            act_data = casadi_controller.get_act_traj_data();
            error_flag = casadi_controller.get_error_flag();
        }
        else if ( controller_type == MainControllerType::Classic )
        {
            timer_mpc_solver.tic();
            tau_full = classic_controller.update(x_filtered_ptr_2);
            timer_mpc_solver.toc();
            act_data = classic_controller.get_act_traj_data();
            error_flag = classic_controller.get_error_flag();
        }
        else if ( controller_type == MainControllerType::Crocoddyl )
        {
            timer_mpc_solver.tic();
            tau_full = crocoddyl_controller.solveMPC(x_filtered_ptr_2);
            timer_mpc_solver.toc();
            act_data = crocoddyl_controller.get_act_traj_data();
            error_flag = crocoddyl_controller.get_error_flag();
        }

        // double* w = casadi_controller.get_w();
        // std::cout << "w:" << std::endl;
        // for(int i = 0; i < 1063; i++)
        // {
        //     std::cout << w[i] << " ";
        // }
        // std::cout << std::endl;
        // return 0;

        if (i % 100 == 0)
        {
            timer_mpc_solver.print_frequency("i=" + std::to_string(i));
            std::cout << std::endl;
            std::cout << "q_k: |" << q_k_ndof_eig.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "|", "|")) << "|" << std::endl;
            std::cout << "u_k: |" << tau_full.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "|", "|")) << "|"<< std::endl;
        }
        if (i == transient_traj_len)
        {
            std::cout << "Switching to trajectory from data" << std::endl;
        }

        current_frequency = timer_mpc_solver.get_frequency();

        // Write data to shm:
        shm.write("read_state_data_full", x_filtered_ptr_2, i);
        shm.write("read_control_data_full", tau_full.data(), i);
        shm.write("read_traj_data_full", act_data, i);
        shm.write("read_frequency_full", &current_frequency, i);
        shm.post_semaphore("shm_changed_semaphore");

        if ( controller_type == MainControllerType::Casadi )
        {
            casadi_controller.simulateModelRK4(x_k_ndof, tau_full.data(), Ts);
        }
        else if ( controller_type == MainControllerType::Classic )
        {
            classic_controller.simulateModelRK4(x_k_ndof, tau_full.data(), Ts);
        }
        else if ( controller_type == MainControllerType::Crocoddyl )
        {
            crocoddyl_controller.simulateModelRK4(x_k_ndof, tau_full.data(), Ts);
        }

        if (error_flag != ErrorFlag::NO_ERROR)
        {
            std::cerr << "Error flag: " << static_cast<int>(error_flag) << std::endl;
            break;
        }
    }

    // Measure and print execution time
    timer_mpc_solver.print_time("Total casadi_controller.solveMPC time: ");
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