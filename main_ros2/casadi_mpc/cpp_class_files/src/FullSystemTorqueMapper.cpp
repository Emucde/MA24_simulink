#include "FullSystemTorqueMapper.hpp"
#include "eigen_templates.hpp"

FullSystemTorqueMapper::FullSystemTorqueMapper(const std::string &urdf_filename,
                                               robot_config_t &robot_config,
                                               bool use_gravity,
                                               bool is_kinematic_mpc)
    : urdf_filename(urdf_filename),
      robot_config(robot_config),
      is_kinematic_mpc(is_kinematic_mpc),
      nq(robot_config.nq),
      nx(robot_config.nx),
      nq_red(robot_config.nq_red),
      nx_red(robot_config.nx_red),
      nq_fixed(robot_config.nq_fixed),
      n_indices(ConstIntVectorMap(robot_config.n_indices, nq_red)),
      n_indices_fixed(ConstIntVectorMap(robot_config.n_indices_fixed, nq_fixed)),
      q_ref_nq(ConstDoubleVectorMap(robot_config.q_0_ref, nq)),
      q_ref_fixed(n_indices_fixed.size())
{
    // Initialize the robot model and data using the URDF
    initRobot(urdf_filename, robot_model_full, robot_data_full, use_gravity);

    // Initialize member matrices, biases, etc.
    K_d = Eigen::MatrixXd::Zero(nq, nq);
    D_d = Eigen::MatrixXd::Zero(nq, nq);
    tau_full = Eigen::VectorXd::Zero(nq);
    q_pp = Eigen::VectorXd::Zero(nq);

    // Default configurations
    config.K_d = Eigen::MatrixXd::Zero(nq, nq);
    config.D_d = Eigen::MatrixXd::Zero(nq, nq);

    // Set the diagonal elements of K_d
    config.K_d.diagonal() << 100, 200, 500, 200, 50, 50, 10;
    D_d = (2 * K_d).array().sqrt();
    config.torque_limit = 100.0;

    K_d_fixed = Eigen::MatrixXd::Zero(nq_fixed, nq_fixed);
    D_d_fixed = Eigen::MatrixXd::Zero(nq_fixed, nq_fixed);

    for (int i = 0; i < n_indices_fixed.size(); ++i)
    {
        for (int j = 0; j < n_indices_fixed.size(); ++j)
        {
            K_d_fixed(i, j) = K_d(n_indices_fixed(i), n_indices_fixed(j));
            D_d_fixed(i, j) = D_d(n_indices_fixed(i), n_indices_fixed(j));
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PUBLIC METHODS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////                                                                             /////
/////                ||||||   ||   ||  ||||    ||      ||    |||||                /////
/////                ||   ||  ||   ||  ||  ||  ||      ||  ||                     /////
/////                ||||||   ||   ||  ||||||  ||      ||  ||                     /////
/////                ||       ||   ||  ||  ||  ||      ||  ||                     /////
/////                ||        |||||   ||||    ||||||  ||    |||||                /////
/////                                                                             /////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

Eigen::VectorXd FullSystemTorqueMapper::calculateNdofTorqueWithFeedforward(
    const Eigen::VectorXd &u,
    const Eigen::VectorXd &q,
    const Eigen::VectorXd &q_p)
{
    if (is_kinematic_mpc)
    {
        // For kinematic MPC, just assign directly based on input
        q_pp(n_indices) = u;
    }
    else
    {
        // Define a reduced input vector corresponding to the identified indices
        Eigen::VectorXd tau_red = u;

        // Get the full inertia matrix and Coriolis forces
        Eigen::MatrixXd M = pinocchio::crba(robot_model_full, robot_data_full, q);
        Eigen::VectorXd C_rnea = pinocchio::rnea(robot_model_full, robot_data_full, q, q_p, Eigen::VectorXd::Zero(nq));

        // Slice the inertia matrix and Coriolis forces to our reduced indices
        Eigen::MatrixXd M_red = M(n_indices, n_indices);
        Eigen::VectorXd C_rnea_tilde = C_rnea(n_indices);

        // Compute the acceleration for the reduced system
        Eigen::VectorXd q_pp_red = M_red.ldlt().solve(tau_red - C_rnea_tilde);
        q_pp(n_indices) = q_pp_red;
    }

    // Use updated q_pp to calculate the resulting full torques
    tau_full = pinocchio::rnea(robot_model_full, robot_data_full, q, q_p, q_pp);

    return tau_full;
}

Eigen::VectorXd FullSystemTorqueMapper::applyPDControl(const Eigen::VectorXd &q_fixed,
                                                       const Eigen::VectorXd &q_p_fixed,
                                                       const Eigen::VectorXd &q_0_ref_fixed,
                                                       const Eigen::MatrixXd &K_d_fixed,
                                                       const Eigen::MatrixXd &D_d_fixed)
{
    return -K_d_fixed * (q_fixed - q_0_ref_fixed) - D_d_fixed * q_p_fixed;
}

Eigen::VectorXd FullSystemTorqueMapper::enforceTorqueLimits(const Eigen::VectorXd &tau)
{
    return tau.cwiseMin(Eigen::VectorXd::Constant(tau.size(), config.torque_limit))
        .cwiseMax(Eigen::VectorXd::Constant(tau.size(), -config.torque_limit));
}

Eigen::VectorXd FullSystemTorqueMapper::calc_full_torque(const Eigen::VectorXd &u, const Eigen::VectorXd &x_k_ndof)
{
    Eigen::VectorXd q = x_k_ndof.head(nq);
    Eigen::VectorXd q_p = x_k_ndof.segment(nq, nq);
    tau_full = calculateNdofTorqueWithFeedforward(u, q, q_p);
    tau_full(n_indices_fixed) += applyPDControl(q(n_indices_fixed), q_p(n_indices_fixed), q_ref_fixed, K_d_fixed, D_d_fixed);

    // tau_full = enforceTorqueLimits(tau_full); // Apply torque limits
    return tau_full;
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PRIVATE METHODS ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////                                                                             /////
/////          ||||||   ||||||   ||  ||    ||   ||||    ||||||||  ||||||          /////
/////          ||   ||  ||   ||  ||  ||    ||  ||   ||     ||     ||              /////
/////          ||||||   ||||||   ||   ||  ||   |||||||     ||     ||||||          /////
/////          ||       || ||    ||    ||||    ||   ||     ||     ||              /////
/////          ||       ||   ||  ||     ||     ||   ||     ||     ||||||          /////
/////                                                                             /////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void FullSystemTorqueMapper::initRobot(const std::string &urdf_filename, pinocchio::Model &robot_model, pinocchio::Data &robot_data, bool use_gravity)
{
    // Load the URDF file into the robot model
    pinocchio::urdf::buildModel(urdf_filename, robot_model);

    // Initialize the corresponding data structure
    robot_data = pinocchio::Data(robot_model);

    if (use_gravity)
    {
        robot_model.gravity.linear() << 0.0, 0.0, -9.81;
    }
    else
    {
        robot_model.gravity.linear() << 0.0, 0.0, 0.0;
    }
}