#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy

#include "include/pinocchio_utils.hpp" // Assuming this is in the same directory
#include "include/CasadiMPC.hpp"
#include "casadi_types.h" // Include for casadi types
#include <Eigen/Dense>    // for Eigen vector

// includes for time measurement
#include <chrono>

#define TRAJ_SELECT 1

int main()
{
    bool use_gravity = false;
    bool fr3_kin_model = true;

    CasadiMPC MPC_obj = CasadiMPC("MPC8");

    int flag = 0;
    const casadi_uint nq = MPC_obj.nq, nx = MPC_obj.nx, nq_red = MPC_obj.nq_red, nx_red = MPC_obj.nx_red;
    const casadi_uint n_fixed = nq - nq_red;
    const casadi_uint *n_indices_ptr = MPC_obj.get_n_indices();
    const casadi_uint *n_x_indices_ptr = MPC_obj.get_n_x_indices();
    const casadi_uint *n_indices_fixed_ptr = MPC_obj.get_n_indices_fixed();
    const casadi_uint *n_x_indices_fixed_ptr = MPC_obj.get_n_x_indices_fixed();
    const casadi_uint traj_data_real_len = MPC_obj.get_traj_data_len();
    casadi_uint i = 0;
    casadi_real *u_opt = MPC_obj.get_optimal_control();
    casadi_real *x_k = MPC_obj.get_x0();

    const std::string urdf_filename = "../../urdf_creation/fr3_no_hand_7dof.urdf";
    pinocchio::Model robot_model;
    pinocchio::Data robot_data;

    pinocchio_utils::initRobot(urdf_filename, robot_model, robot_data, use_gravity);

    Eigen::VectorXd x_k_ndof_vec = Eigen::VectorXd::Zero(nx);
    Eigen::VectorXd u_k(nq_red);
    Eigen::VectorXd x_k_vec(nx_red);
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);

    Eigen::Map<Eigen::VectorXi> n_indices(reinterpret_cast<int *>(const_cast<uint32_t *>(n_indices_ptr)), nq_red);
    Eigen::Map<Eigen::VectorXi> n_x_indices(reinterpret_cast<int *>(const_cast<uint32_t *>(n_x_indices_ptr)), nx_red);
    Eigen::Map<Eigen::VectorXi> n_indices_fixed(reinterpret_cast<int *>(const_cast<uint32_t *>(n_indices_fixed_ptr)), n_fixed);

    // Use PD control for fixed joints
    const std::vector<casadi_real> x_ref_nq_vec = MPC_obj.get_x_ref_nq();
    Eigen::Map<Eigen::VectorXd> x_ref_nq(const_cast<double *>(x_ref_nq_vec.data()), x_ref_nq_vec.size());
    Eigen::Map<Eigen::VectorXd> x_k_ndof = x_ref_nq; // Testing: should come from measurement

    Eigen::VectorXd q_ref_nq = x_ref_nq.head(nq);

    Eigen::VectorXd q_ndof = x_k_ndof.head(nq);  // Testing: should come from measurement
    Eigen::VectorXd q_p_ndof = x_k_ndof.tail(nq); // Testing: should come from measurement

    Eigen::VectorXd q_ref_fixed = q_ref_nq(n_indices_fixed);
    Eigen::VectorXd q_fixed = q_ndof(n_indices_fixed);
    Eigen::VectorXd q_p_fixed = q_p_ndof(n_indices_fixed);

    // Define K_d and D_d
    Eigen::MatrixXd K_d(7, 7);
    K_d.setZero(); // Set all elements to zero
    K_d.diagonal() << 100, 200, 500, 200, 50, 50, 10; // Set the diagonal elements

    // Compute D_d as sqrt(2*K_d)
    Eigen::MatrixXd D_d = (2 * K_d).array().sqrt(); // Element-wise sqrt after scaling by 2

    // Create K_d_fixed and D_d_fixed based on indices_fixed
    Eigen::MatrixXd K_d_fixed(n_indices_fixed.size(), n_indices_fixed.size());
    Eigen::MatrixXd D_d_fixed(n_indices_fixed.size(), n_indices_fixed.size());

    for (int i = 0; i < n_indices_fixed.size(); ++i) {
        for (int j = 0; j < n_indices_fixed.size(); ++j) {
            K_d_fixed(i, j) = K_d(n_indices_fixed(i), n_indices_fixed(j));
            D_d_fixed(i, j) = D_d(n_indices_fixed(i), n_indices_fixed(j));
        }
    }

    Eigen::VectorXd tau_fixed = Eigen::VectorXd::Zero(n_fixed);

    // measure time
    auto start = std::chrono::high_resolution_clock::now();

    for (i = 0; i < 10000; i++)
    {
        flag = MPC_obj.solve(x_k);
        // Get the optimal control

        u_k = Eigen::Map<Eigen::VectorXd>(u_opt, nq_red); // Just map the data to Eigen vector

        memcpy(x_k, u_opt + nq_red, nx_red * sizeof(casadi_real));

        x_k_ndof(n_x_indices) = Eigen::Map<Eigen::VectorXd>(x_k, nx_red);

        // // Calculate the full torque
        tau_full = pinocchio_utils::calculateNdofTorqueWithFeedforward(u_k, x_k_ndof, robot_model, robot_data, nq, n_indices, fr3_kin_model);

        tau_fixed = pinocchio_utils::applyPDControl(q_fixed, q_p_fixed, q_ref_fixed, K_d_fixed, D_d_fixed);

        tau_full(n_indices_fixed) += tau_fixed;

        // Print the optimal control
        // std::cout << "Optimal control: ";
        // for (int i = 0; i < nq_red; ++i) {
        //     std::cout << u_opt_out[i] << " ";
        // }
        // std::cout << x_k[0] << " " << x_k[1] << " " << x_k[2] << " " << x_k[3] << " " << x_k[4] << " " << x_k[5] << " " << x_k[6] << std::endl;
        // std::cout << "Optimal control: " << u_k << std::endl;
        std::cout << "Full torque: " << tau_full << std::endl;
        std::cout << std::endl;
    }

    // measure time
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: "
              << (double)duration.count() / 1000000 << " s" << std::endl;

    return 0;
}
