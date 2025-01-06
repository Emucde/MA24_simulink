#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy
#include <chrono>  // for time measurement

#include "include/pinocchio_utils.hpp"
#include "include/FullSystemTorqueMapper.hpp"
#include "include/CasadiMPC.hpp"
#include "include/CasadiController.hpp"
#include "param_robot.h"
#include "casadi_types.h"
#include <Eigen/Dense>

#define TRAJ_SELECT 1
// #define PLOT_DATA

void set_x_k_ndof(casadi_real *x_k_ndof_ptr, casadi_real *x_k, const std::vector<casadi_real> &x_ref_nq_vec, casadi_uint nx, const casadi_uint *n_x_indices)
{
    int cnt = 0;
    for (casadi_uint i = 0; i < nx; ++i)
    {
        if (i == n_x_indices[cnt])
        {
            x_k_ndof_ptr[i] = x_k[cnt];
            cnt++;
        }
        else
        {
            x_k_ndof_ptr[i] = x_ref_nq_vec[i];
        }
    }
}

int main()
{
    // Configuration flags
    bool use_gravity = false;
    const std::string urdf_filename = "../../urdf_creation/fr3_no_hand_7dof.urdf";

    CasadiController controller(urdf_filename, use_gravity);
    controller.setActiveMPC(MPCType::MPC8);
    controller.switch_traj(TRAJ_SELECT);

    const casadi_uint nq = controller.nq;
    const casadi_uint nx = controller.nx;
    const casadi_uint nq_red = controller.nq_red;
    const casadi_uint nx_red = controller.nx_red;
    const casadi_uint *n_x_indices_ptr = controller.get_n_x_indices();
    const casadi_uint traj_data_real_len = controller.get_traj_data_len();
    const std::vector<casadi_real> x_ref_nq_vec = controller.get_x_ref_nq();
    casadi_real *u_opt = controller.get_optimal_control();
    casadi_real *x_k = controller.get_x_k();
    casadi_real x_k_ndof[nx];
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);
    Eigen::VectorXd x_k_ndof_eig;

    // Measure execution time
    auto start = std::chrono::high_resolution_clock::now();

    set_x_k_ndof(x_k_ndof, x_k, x_ref_nq_vec, nx, n_x_indices_ptr);

#ifdef PLOT_DATA
    std::ofstream x_k_ndof_file("x_k_ndof_data.txt");
    std::ofstream tau_full_file("tau_full_data.txt");
#endif

    // Main loop for trajectory processing
    for (casadi_uint i = 0; i < traj_data_real_len; i++)
    {
        tau_full = controller.solveMPC(x_k_ndof);
        x_k_ndof_eig = Eigen::Map<Eigen::VectorXd>(x_k_ndof, nq);

        if (i % 100 == 0)
        {
            std::cout << "q_k: " << x_k_ndof_eig.transpose() << std::endl;
            // std::cout << "Full torque: " << tau_full.transpose() << std::endl;
        }

#ifdef PLOT_DATA
        x_k_ndof_file << x_k_ndof_eig.transpose() << std::endl;
        tau_full_file << tau_full.transpose() << std::endl;
#endif

        memcpy(x_k, u_opt + nq_red, nx_red * sizeof(casadi_real));
        set_x_k_ndof(x_k_ndof, x_k, x_ref_nq_vec, nx, n_x_indices_ptr);
    }
    // Measure and print execution time
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: " << (double)duration.count() / 1000000 << " s" << std::endl;

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