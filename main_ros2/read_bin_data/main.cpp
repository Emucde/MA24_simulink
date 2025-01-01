#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy

#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "CasadiMPC.hpp"

// includes for time measurement
#include <chrono>

#define TRAJ_SELECT 1

int main()
{
    casadi_real *u_opt_out = nullptr;
    int u_opt_len = 0, i = 0;

    int flag = 0;
    uint32_t traj_data_real_len;

    const std::string urdf_filename = "../../urdf_creation/fr3_no_hand_7dof.urdf";

    pinocchio::Model robot_model;

    // Load the URDF model
    pinocchio::urdf::buildModel(urdf_filename, robot_model);

    // Create data structure for robot_model
    pinocchio::Data robot_data(robot_model); // Create data structure for robot_model
    
    // Number of joint positions, states, inputs
    int nq = robot_model.nq;
    int nx = 2 * nq;
    int nu = nq; // fully actuated
    
    std::cout << "robot_model name: " << robot_model.name << std::endl;

    bool use_gravity = true;

    if (use_gravity) {
        // Set linear motion as gravity (0, 0, -9.81) and angular as (0, 0, 0)
        robot_model.gravity.linear() << 0.0, 0.0, -9.81; // Linear part
    } else {
        robot_model.gravity.linear() << 0.0, 0.0, 0.0; // Linear part should be zero
    }

    std::cout << robot_model.gravity.linear();

    CasadiMPC MPC8_obj = CasadiMPC("MPC8");

    // Get the length of the trajectory data
    traj_data_real_len = MPC8_obj.get_traj_data_len();

    // measure time
    auto start = std::chrono::high_resolution_clock::now();

    for (i = 0; i < 1; i++)
    {
        flag = MPC8_obj.solve();
        // // Get the optimal control
        // MPC8_obj.get_optimal_control(u_opt_out, u_opt_len);

        // // Print the optimal control
        // std::cout << "Optimal control: ";
        // for (int i = 0; i < u_opt_len; ++i) {
        //     std::cout << u_opt_out[i] << " ";
        // }
        // std::cout << std::endl;
    }

    // measure time
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: "
              << (double)duration.count() / 1000000 << " s" << std::endl;

    return 0;
}
