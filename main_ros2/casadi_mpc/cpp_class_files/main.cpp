#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy
#include <chrono>  // for time measurement

#include "include/pinocchio_utils.hpp"
#include "include/FullSystemTorqueMapper.hpp"
#include "include/CasadiMPC.hpp"
#include "include/CasadiController.hpp"
#include "include/casadi_controller_types.hpp"
#include "param_robot.h"
#include "casadi_types.h"
#include <Eigen/Dense>
#include "include/eigen_templates.hpp"


// #define PLOT_DATA
#define TRANSIENT_TRAJ_TESTS

class TicToc {
public:
    TicToc() : start_time(), elapsed_time(0), elapsed_total_time(0), running(false) {}

    void tic(bool reset = false) {
        if (reset) {
            start_time = std::chrono::time_point<std::chrono::high_resolution_clock>();
            elapsed_total_time = 0;
            running = false;
        } else {
            start_time = std::chrono::high_resolution_clock::now();
            running = true;
        }
    }

    double toc() {
        if (!running) {
            std::cerr << "Error: Tic not started.\n";
            return 0.0;
        }

        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_time_act = current_time - start_time;
        elapsed_time = elapsed_time_act.count();
        elapsed_total_time += elapsed_time;
        start_time = current_time;
        return elapsed_time;
    }

    double get_time() {
        return elapsed_total_time;
    }

    std::string get_time_str(const std::string& additional_text = "time") {
        return "\033[92mElapsed " + additional_text + ": " + format_time(elapsed_total_time) + "\033[0m";
    }

    void print_time(const std::string& additional_text = "time") {
        std::string time_str = get_time_str(additional_text);
        std::cout << time_str;
    }

    void reset() {
        start_time = std::chrono::time_point<std::chrono::high_resolution_clock>();
        elapsed_total_time = 0;
        running = false;
    }

    double get_frequency() {
        if (elapsed_total_time == 0) {
            std::cerr << "Error: No elapsed time. Cannot calculate frequency.\n";
            return 0;
        }
        double frequency = 1.0 / elapsed_time; // Frequency in Hz
        return frequency;
    }

    void print_frequency(std::string additional_text = "Frequency: ") {
        if (elapsed_total_time == 0) {
            std::cerr << "Error: No elapsed time. Cannot calculate frequency.\n";
            return;
        }
        double frequency = get_frequency();
        std::cout << "\033[92m" << additional_text << ": " << format_frequency(frequency) << "\033[0m";
    }


    std::string format_frequency(double frequency, int precision = 2) {
        std::ostringstream oss;
        if (frequency < 1) {
            oss << std::fixed << std::setprecision(precision) << frequency << " Hz"; // Hertz
        } else if (frequency < 1e3) {
            oss << std::fixed << std::setprecision(precision) << frequency << " Hz"; // Hertz
        } else if (frequency < 1e6) {
            oss << std::fixed << std::setprecision(precision) << frequency / 1e3 << " kHz"; // kHz
        } else {
            oss << std::fixed << std::setprecision(precision) << frequency / 1e6 << " MHz"; // MHz
        }
        return oss.str();
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    double elapsed_time; // current elapsed time in seconds
    double elapsed_total_time; // total elapsed time in seconds
    bool running;

    std::string format_time(double elapsed_time, int precision = 2) {
        std::ostringstream oss;
        if (elapsed_time == 0) {
            return "0 s";
        } else if (elapsed_time < 1e-6) {
            oss << std::fixed << std::setprecision(precision) << elapsed_time * 1e9 << " ns";  // Nanoseconds
        } else if (elapsed_time < 1e-3) {
            oss << std::fixed << std::setprecision(precision) << elapsed_time * 1e6 << " µs";  // Microseconds
        } else if (elapsed_time < 1) {
            oss << std::fixed << std::setprecision(precision) << elapsed_time * 1e3 << " ms";  // Milliseconds
        } else if (elapsed_time < 60) {
            oss << std::fixed << std::setprecision(precision) << elapsed_time << " s";         // Seconds
        } else {
            int minutes = static_cast<int>(elapsed_time / 60);
            double seconds = elapsed_time - minutes * 60;
            oss << minutes << " min " << std::fixed << std::setprecision(precision) << seconds << " s";
        }
        
        return oss.str();
    }
};

#define TRAJ_SELECT 4
int main()
{
    // Configuration flags
    bool use_gravity = false;
    const std::string urdf_filename = "../../../urdf_creation/fr3_no_hand_7dof.urdf";
    const std::string tcp_frame_name = "fr3_link8_tcp";

    CasadiController controller(urdf_filename, tcp_frame_name, use_gravity);
    controller.setActiveMPC(MPCType::MPC01);
    controller.switch_traj(TRAJ_SELECT);

    const casadi_uint nq = controller.nq;
    const casadi_uint nx = controller.nx;
    const casadi_uint nq_red = controller.nq_red;
    const casadi_uint nx_red = controller.nx_red;
    const casadi_uint *n_indices_ptr = controller.get_n_indices();
    const casadi_uint *n_x_indices_ptr = controller.get_n_x_indices();
    const casadi_uint traj_data_real_len = controller.get_traj_data_len();
    const std::vector<casadi_real> x_ref_nq_vec = controller.get_x_ref_nq();
    ErrorFlag error_flag = ErrorFlag::NO_ERROR;

    casadi_real *x_k = controller.get_x_k();
    casadi_real x_k_ndof[nx];
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);

    Eigen::Map<Eigen::VectorXd> q_k_ndof_eig(x_k_ndof, nq);
    Eigen::Map<Eigen::VectorXd> x_k_ndof_eig(x_k_ndof, nx);
    Eigen::Map<Eigen::VectorXd> x_k_eig(x_k, nx_red);

    Eigen::VectorXi n_indices_eig   = ConstIntVectorMap(n_indices_ptr, nq_red);
    Eigen::VectorXi n_x_indices_eig = ConstIntVectorMap(n_x_indices_ptr, nx_red);

    // Measure execution time
    TicToc timer_mpc_solver;
    TicToc timer_total;

    x_k_ndof_eig(n_x_indices_eig) = x_k_eig;
    
#ifdef PLOT_DATA
    std::ofstream x_k_ndof_file("x_k_ndof_data.txt");
    std::ofstream tau_full_file("tau_full_data.txt");
#endif

#ifdef TRANSIENT_TRAJ_TESTS

    q_k_ndof_eig(n_indices_eig) += Eigen::VectorXd::Constant(nq_red, 0.1);

    controller.generate_transient_trajectory(x_k_ndof, 0.0, 1.0, 2.0);
    Eigen::MatrixXd trajectory = controller.get_transient_traj_data();
    casadi_uint transient_traj_len = controller.get_transient_traj_len();

    timer_total.tic();

    // // Output the trajectory results
    // for (int i = 0; i < trajectory.cols(); ++i)
    // {
    //     if (i % 10 == 0)
    //         std::cout << "Step " << i << ": " << trajectory.col(i).transpose() << std::endl;
    // }
#endif

    // Main loop for trajectory processing
    for (casadi_uint i = 0; i < traj_data_real_len + transient_traj_len; i++)
    {
        timer_mpc_solver.tic();
        tau_full = controller.solveMPC(x_k_ndof);
        error_flag = controller.get_error_flag();
        timer_mpc_solver.toc();

        if (error_flag != ErrorFlag::NO_ERROR)
        {
            std::cerr << "Error flag: " << static_cast<int>(error_flag) << std::endl;
            break;
        }

        if (i % 100 == 0)
        {
            // std::cout << "q_k: " << q_k_ndof_eig.transpose();
            std::cout << "Full torque: " << tau_full.transpose();
            timer_mpc_solver.print_frequency("\t");
            std::cout << std::endl;
        }
        if(i == transient_traj_len)
        {
            std::cout << "Switching to trajectory from data" << std::endl;
        }

#ifdef PLOT_DATA
        x_k_ndof_file << x_k_ndof_eig.transpose() << std::endl;
        tau_full_file << tau_full.transpose() << std::endl;
#endif

        // simulate the model
        controller.simulateModel(x_k_ndof, tau_full.data(), 1e-3);
    }

    // Measure and print execution time
    timer_mpc_solver.print_time("Total controller.solveMPC time: ");
    std::cout << std::endl;

    timer_total.toc();
    timer_total.print_time("Total execution time: ");
    std::cout << std::endl;

#ifdef PLOT_DATA
    x_k_ndof_file.close();
    tau_full_file.close();
#endif

    return 0;
}

/*
% Read the torque data
torque_data = dlmread('./main_ros2/read_bin_data/tau_full_data.txt');

% Read the state data
state_data = dlmread('./main_ros2/read_bin_data/x_k_ndof_data.txt');

% Create a figure with subplots
figure;

% Plot torque data
subplot(3,1,1);
plot(torque_data);
title('7-Dimensional Torque');
xlabel('Time step');
ylabel('Torque (Nm)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

% Plot joint angles
subplot(3,1,2);
plot(state_data(:,1:7));
title('Joint Angles');
xlabel('Time step');
ylabel('Angle (rad)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

% Plot joint velocities
subplot(3,1,3);
plot(state_data(:,8:14));
title('Joint Velocities');
xlabel('Time step');
ylabel('Velocity (rad/s)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

% Adjust the layout
sgtitle('Robot Joint Data');

*/