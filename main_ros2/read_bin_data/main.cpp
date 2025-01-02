#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy

// #include "include/pinocchio_utils.hpp" // Assuming this is in the same directory
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

    // auto [robot_model, robot_data] = initRobot(urdf_filename);

    CasadiMPC MPC8_obj = CasadiMPC("MPC8");

    // Get the length of the trajectory data
    traj_data_real_len = MPC8_obj.get_traj_data_len();

    // measure time
    auto start = std::chrono::high_resolution_clock::now();

    // tau_full = calculateNdofTorqueWithFeedforward(u_k, x_k_ndof, robot_model_full, robot_data_full, n_dof, n_indices, fr3_kin_model);

    // // Use PD control for fixed joints
    // Eigen::VectorXd q_ndof = x_k_ndof.head(n_dof);
    // Eigen::VectorXd q_p_ndof = x_k_ndof.tail(n_dof);

    // Eigen::VectorXd q_fixed = q_ndof(n_indices_fixed);
    // Eigen::VectorXd q_p_fixed = q_p_ndof(n_indices_fixed);

    // Eigen::VectorXd tau_fixed = applyPDControl(q_fixed, q_p_fixed, q_0_ref_fixed, K_d_fixed, D_d_fixed);

    // tau_full(n_indices_fixed) += tau_fixed;

    // xs[i] = x_k_ndof;
    // us[i] = tau_full;

    for (i = 0; i < 1; i++)
    {
        flag = MPC8_obj.solve();
        // Get the optimal control
        MPC8_obj.get_optimal_control(u_opt_out, u_opt_len);

        // Print the optimal control
        std::cout << "Optimal control: ";
        for (int i = 0; i < u_opt_len; ++i) {
            std::cout << u_opt_out[i] << " ";
        }
        std::cout << std::endl;
    }

    // measure time
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: "
              << (double)duration.count() / 1000000 << " s" << std::endl;

    return 0;
}
