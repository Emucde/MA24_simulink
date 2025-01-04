#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy
#include <chrono>  // for time measurement

#include "include/pinocchio_utils.hpp"
#include "include/FullSystemTorqueMapper.hpp"
#include "include/CasadiMPC.hpp"
#include "casadi_types.h"
#include <Eigen/Dense>

#define TRAJ_SELECT 1
// #define PLOT_DATA

int main()
{
    // Configuration flags
    bool use_gravity = false;
    bool fr3_kin_model = true;

    // Initialize MPC object
    CasadiMPC MPC_obj("MPC8");

    // Retrieve necessary parameters from MPC object
    const casadi_uint nq = MPC_obj.nq;
    const casadi_uint nx = MPC_obj.nx;
    const casadi_uint nq_red = MPC_obj.nq_red;
    const casadi_uint nx_red = MPC_obj.nx_red;
    const casadi_uint *n_x_indices_ptr = MPC_obj.get_n_x_indices();
    const casadi_uint traj_data_real_len = MPC_obj.get_traj_data_len();
    casadi_real *u_opt = MPC_obj.get_optimal_control();
    casadi_real *x_k = MPC_obj.get_x0();

    // URDF file path
    const std::string urdf_filename = "../../urdf_creation/fr3_no_hand_7dof.urdf";

    // Initialize torque mapper and robot model
    FullSystemTorqueMapper torque_mapper(urdf_filename, MPC_obj, use_gravity);
    pinocchio::Model robot_model;
    pinocchio::Data robot_data;
    pinocchio_utils::initRobot(urdf_filename, robot_model, robot_data, use_gravity);

    // Eigen vectors for state and torque
    Eigen::VectorXd x_k_ndof_vec = Eigen::VectorXd::Zero(nx);
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);

    // Map indices and state vectors
    Eigen::Map<Eigen::VectorXi> n_x_indices(reinterpret_cast<int *>(const_cast<uint32_t *>(n_x_indices_ptr)), nx_red);
    const std::vector<casadi_real> x_ref_nq_vec = MPC_obj.get_x_ref_nq();
    // Eigen::Map<Eigen::VectorXd> x_k_ndof(const_cast<double *>(x_ref_nq_vec.data()), x_ref_nq_vec.size());
    Eigen::Map<const Eigen::VectorXd> x_ref_nq_map(x_ref_nq_vec.data(), x_ref_nq_vec.size());
    Eigen::Map<Eigen::VectorXd> x_k_map(x_k, nx_red); // Map the pointer x_k which is nx_red-dimensional

    Eigen::VectorXd x_k_ndof = x_ref_nq_map;
    std::vector<casadi_real *> x_k_ndof_ptr;

    // Populate the vector with pointers
    int cnt = 0;
    for (casadi_uint i = 0; i < nx; ++i)
    {
        if (i == n_x_indices[cnt])
        {
            x_k_ndof_ptr.push_back(&x_k[cnt]);
            cnt++;
        }
        else
        {
            x_k_ndof_ptr.push_back(const_cast<double *>(&x_ref_nq_vec[i]));
        }
    }
    Eigen::Map<Eigen::VectorXd> u_k(u_opt, nq_red); // persistent map to u_opt

// Open files to save data
#ifdef PLOT_DATA
    std::ofstream x_k_ndof_file("x_k_ndof_data.txt");
    std::ofstream tau_full_file("tau_full_data.txt");
#endif

    // Measure execution time
    auto start = std::chrono::high_resolution_clock::now();

    // Main loop for trajectory processing
    for (casadi_uint i = 0; i < traj_data_real_len; i++)
    {
        int flag = MPC_obj.solve(x_k);
        if (flag)
        {
            std::cerr << "Error in Casadi function call." << std::endl;
            break;
        }

        // Update state and calculate full torque
        memcpy(x_k, u_opt + nq_red, nx_red * sizeof(casadi_real));
        tau_full = torque_mapper.calc_full_torque(u_k, x_k_ndof, nq, fr3_kin_model);

// Save data to files
#ifdef PLOT_DATA
        // x_k_ndof_file << tau_full_file << tau_full.transpose() << std::endl;
        for (size_t i = 0; i < x_k_ndof_ptr.size(); ++i)
        {
            x_k_ndof_file << *x_k_ndof_ptr[i];
            if (i < x_k_ndof_ptr.size() - 1)
                x_k_ndof_file << "\t";
        }
        x_k_ndof_file << std::endl;
        tau_full_file << tau_full.transpose() << std::endl;
#endif

        if (i % 100 == 0)
        {
            // std::cout << "Full torque: " << tau_full.transpose() << std::endl;
            // std::cout << "x_k: " << x_k_map.transpose() << std::endl;
            for (size_t i = 0; i < x_k_ndof_ptr.size(); ++i)
            {
                std::cout << *x_k_ndof_ptr[i];
                if (i < x_k_ndof_ptr.size() - 1)
                    std::cout << " ";
            }
            std::cout << std::endl;
        }
    }

// Close files
#ifdef PLOT_DATA
    x_k_ndof_file.close();
    tau_full_file.close();
#endif

    // Measure and print execution time
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: " << (double)duration.count() / 1000000 << " s" << std::endl;

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