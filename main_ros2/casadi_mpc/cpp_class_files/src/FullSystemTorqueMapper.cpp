#include "FullSystemTorqueMapper.hpp"
#include "eigen_templates.hpp"

FullSystemTorqueMapper::FullSystemTorqueMapper(const std::string &urdf_filename,
                                               const std::string &tcp_frame_name,
                                               robot_config_t &robot_config,
                                               bool use_gravity,
                                               bool is_kinematic_mpc)
    : urdf_filename(urdf_filename),
      tcp_frame_name(tcp_frame_name),
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
      q_ref_fixed(Eigen::Map<const Eigen::VectorXd>(robot_config.q_0_ref, nq_fixed)(n_indices_fixed))
{
    // Initialize the function pointer based on the type of MPC
    setFeedforwardTorqueFunction(is_kinematic_mpc);

    // Initialize the robot model and data using the URDF
    initRobot(urdf_filename, tcp_frame_name, robot_model_full, robot_data_full, use_gravity);

    // Initialize member matrices, biases, etc.
    tau_full = Eigen::VectorXd::Zero(nq);
    q_pp = Eigen::VectorXd::Zero(nq);

    config.K_d = Eigen::MatrixXd::Zero(nq, nq); // Proportional gain matrix
    // Default configurations
    config.K_d.diagonal() << 100, 200, 500, 200, 50, 50, 10;
    config.D_d = (2 * config.K_d).array().sqrt();
    config.K_d_fixed = config.K_d(n_indices_fixed, n_indices_fixed);
    config.D_d_fixed = config.D_d(n_indices_fixed, n_indices_fixed);
    config.q_ref_nq = q_ref_nq;
    config.q_ref_nq_fixed = q_ref_fixed;

    config.torque_limit = 100.0;
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

// Implementation of kinematic torque mapping
Eigen::VectorXd FullSystemTorqueMapper::calcFeedforwardTorqueKinematic(
    const Eigen::VectorXd &u,
    const Eigen::VectorXd &q,
    const Eigen::VectorXd &q_p)
{
    // Assuming q_pp is properly defined elsewhere
    q_pp.setZero(); // Initialize q_pp if necessary
    q_pp(n_indices) = u; // Set specific index using input

    // Calculate the resulting torques
    return pinocchio::rnea(robot_model_full, robot_data_full, q, q_p, q_pp);
}

// Implementation of dynamic torque mapping
Eigen::VectorXd FullSystemTorqueMapper::calcFeedforwardTorqueDynamic(
    const Eigen::VectorXd &u,
    const Eigen::VectorXd &q,
    const Eigen::VectorXd &q_p)
{
    q_pp.setZero(); // Initialize q_pp
    Eigen::VectorXd tau_red = u; // Define reduced input vector

    // Calculate inertia matrix and Coriolis forces
    pinocchio::crba(robot_model_full, robot_data_full, q);
    robot_data_full.M.triangularView<Eigen::StrictlyLower>() = robot_data_full.M.transpose().triangularView<Eigen::StrictlyLower>();
    Eigen::MatrixXd M = robot_data_full.M;
    Eigen::VectorXd C_rnea = pinocchio::rnea(robot_model_full, robot_data_full, q, q_p, Eigen::VectorXd::Zero(nq));

    // Slice the inertia matrix and Coriolis forces
    Eigen::MatrixXd M_red = M(n_indices, n_indices);
    Eigen::VectorXd C_rnea_tilde = C_rnea(n_indices);

    // Compute the acceleration for the reduced system
    Eigen::VectorXd q_pp_red = M_red.ldlt().solve(tau_red - C_rnea_tilde);
    q_pp(n_indices) = q_pp_red; // Update full acceleration vector

    // Calculate the resulting full torques
    return pinocchio::rnea(robot_model_full, robot_data_full, q, q_p, q_pp);
}

void FullSystemTorqueMapper::setFeedforwardTorqueFunction(bool is_kinematic_mpc) {
    if (is_kinematic_mpc) {
        calcFeedforwardTorqueFunPtr = &FullSystemTorqueMapper::calcFeedforwardTorqueKinematic;
    } else {
        calcFeedforwardTorqueFunPtr = &FullSystemTorqueMapper::calcFeedforwardTorqueDynamic;
    }
}

Eigen::VectorXd FullSystemTorqueMapper::applyPDControl(const Eigen::VectorXd &q_fixed,
                                                       const Eigen::VectorXd &q_p_fixed)
{
    return -config.K_d_fixed * (q_fixed - config.q_ref_nq_fixed) - config.D_d_fixed * q_p_fixed;
}

Eigen::VectorXd FullSystemTorqueMapper::enforceTorqueLimits(const Eigen::VectorXd &tau)
{
    return tau.cwiseMin(Eigen::VectorXd::Constant(tau.size(), config.torque_limit))
        .cwiseMax(Eigen::VectorXd::Constant(tau.size(), -config.torque_limit));
}

Eigen::VectorXd FullSystemTorqueMapper::calc_full_torque(const Eigen::VectorXd &u, const Eigen::VectorXd &x_k_ndof)
{
    Eigen::Ref<const Eigen::VectorXd> q = x_k_ndof.head(nq);
    Eigen::Ref<const Eigen::VectorXd> q_p = x_k_ndof.tail(nq);
    tau_full = (this->*calcFeedforwardTorqueFunPtr)(u, q, q_p);
    tau_full(n_indices_fixed) += applyPDControl(q(n_indices_fixed), q_p(n_indices_fixed));

    // tau_full = enforceTorqueLimits(tau_full); // Apply torque limits
    return tau_full;
}

void FullSystemTorqueMapper::calcPose(const casadi_real* x, Eigen::Vector3d &p, Eigen::Matrix3d &R)
{
    Eigen::Map<const Eigen::VectorXd> q(x, nq);
    calcPose(q, p, R);
}

// Method for calculating a pose by a given state
void FullSystemTorqueMapper::calcPose(const Eigen::VectorXd &q, Eigen::Vector3d &p, Eigen::Matrix3d &R)
{
    pinocchio::forwardKinematics(robot_model_full, robot_data_full, q);
    pinocchio::updateFramePlacements(robot_model_full, robot_data_full);
    p = robot_data_full.oMf[tcp_frame_id].translation();
    R = robot_data_full.oMf[tcp_frame_id].rotation();
}

// Method for simulating the robot model
void FullSystemTorqueMapper::simulateModelEuler(Eigen::Map<Eigen::VectorXd> &x_k_ndof, Eigen::Map<const Eigen::VectorXd> &tau, double dt)
{
    Eigen::Ref<const Eigen::VectorXd> q = x_k_ndof.head(nq);
    Eigen::Ref<const Eigen::VectorXd> q_p = x_k_ndof.tail(nq);

    Eigen::VectorXd q_pp = pinocchio::aba(robot_model_full, robot_data_full, q, q_p, tau);

    x_k_ndof.head(nq) = q + q_p * dt; // q_next = q + q_p * dt
    x_k_ndof.tail(nq) = q_p + q_pp * dt; // q_p_next = q_p + q_pp * dt
}

void FullSystemTorqueMapper::simulateModelRK4(Eigen::Map<Eigen::VectorXd> &x_k_ndof, Eigen::Map<const Eigen::VectorXd> &tau, double dt)
{
    // Define lambda for computing f(x, u) = [q_p, q_pp] = [x2, u]
    auto f = [&](const Eigen::VectorXd& state) -> Eigen::VectorXd {
        // Extract positions and velocities from the state vector
        Eigen::Ref<const Eigen::VectorXd> q = state.head(nq);
        Eigen::Ref<const Eigen::VectorXd> q_p = state.tail(nq);

        // Compute joint accelerations using Pinocchio's ABA
        Eigen::VectorXd q_pp = pinocchio::aba(robot_model_full, robot_data_full, q, q_p, tau);

        // Combine derivatives: [q_p; q_pp]
        Eigen::VectorXd derivative(2 * nq);
        derivative.head(nq) = q_p;    // dq/dt = q_p
        derivative.tail(nq) = q_pp;  // dq_p/dt = q_pp

        return derivative;
    };
    x_k_ndof = RK4(x_k_ndof, dt, f);
}

Eigen::VectorXd FullSystemTorqueMapper::RK4(const Eigen::VectorXd& state, double dt, const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f) {
    // Compute k1
    Eigen::VectorXd k1 = f(state);

    // Compute k2
    Eigen::VectorXd k2 = f(state + 0.5 * dt * k1);

    // Compute k3
    Eigen::VectorXd k3 = f(state + 0.5 * dt * k2);

    // Compute k4
    Eigen::VectorXd k4 = f(state + dt * k3);

    // Update state using weighted sum of derivatives
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
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

void FullSystemTorqueMapper::initRobot(const std::string &urdf_filename,
                                       const std::string &tcp_frame_name,
                                       pinocchio::Model &robot_model,
                                       pinocchio::Data &robot_data,
                                       bool use_gravity)
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

    // Initialize the TCP frame Id
    tcp_frame_id = robot_model.getFrameId(tcp_frame_name);
}