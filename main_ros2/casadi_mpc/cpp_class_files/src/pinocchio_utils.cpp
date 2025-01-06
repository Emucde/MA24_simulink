#include "pinocchio_utils.hpp" // Assuming this is in the same directory

namespace pinocchio_utils {

// Function to calculate n-DOF torque with feedforward
Eigen::VectorXd calculateNdofTorqueWithFeedforward(const Eigen::VectorXd &u, 
    const Eigen::VectorXd &x_k_ndof, 
    const pinocchio::Model &robot_model_full, 
    pinocchio::Data &robot_data_full, 
    int n, 
    const Eigen::Map<Eigen::VectorXi> &n_indices, 
    bool kin_model) {
    
    Eigen::VectorXd q = x_k_ndof.head(n);
    Eigen::VectorXd q_p = x_k_ndof.segment(n, n);
    Eigen::VectorXd q_pp(n);
    
    if (kin_model) {
        q_pp.setZero();
        q_pp(n_indices) = u;  // Assuming n_indices is a vector of valid indices.
    } else {
        Eigen::VectorXd tau_red = u;

        Eigen::MatrixXd M = pinocchio::crba(robot_model_full, robot_data_full, q);
        Eigen::VectorXd C_rnea = pinocchio::rnea(robot_model_full, robot_data_full, q, q_p, Eigen::VectorXd::Zero(n));

        Eigen::MatrixXd M_red = M(n_indices, n_indices);
        Eigen::VectorXd C_rnea_tilde = C_rnea(n_indices);

        q_pp.segment(n_indices[0], n_indices.size()) = M_red.ldlt().solve(tau_red - C_rnea_tilde);
        
        q_pp.setZero(); // Set all to zero
        q_pp(n_indices) = q_pp.segment(n_indices[0], n_indices.size());
    }

    Eigen::VectorXd tau_full = pinocchio::rnea(robot_model_full, robot_data_full, q, q_p, q_pp);
    
    return tau_full;
}


// Function for PD control
Eigen::VectorXd applyPDControl(const Eigen::VectorXd &q_fixed, 
                               const Eigen::VectorXd &q_p_fixed,
                               const Eigen::VectorXd &q_0_ref_fixed, 
                               const Eigen::MatrixXd &K_d_fixed,
                               const Eigen::MatrixXd &D_d_fixed) {
    return -K_d_fixed * (q_fixed - q_0_ref_fixed) - D_d_fixed * q_p_fixed;
}

void initRobot(const std::string &urdf_filename, pinocchio::Model &robot_model, pinocchio::Data &robot_data, bool use_gravity)
{
    // Load the URDF file into the robot model
    pinocchio::urdf::buildModel(urdf_filename, robot_model);

    // Initialize the corresponding data structure
    robot_data = pinocchio::Data(robot_model);

    if (use_gravity) {
        robot_model.gravity.linear() << 0.0, 0.0, -9.81;
    } else {
        robot_model.gravity.linear() << 0.0, 0.0, 0.0;
    }

}

} // namespace pinocchio_utils