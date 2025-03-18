#include "FullSystemTorqueMapper.hpp"
#include "eigen_templates.hpp"

FullSystemTorqueMapper::FullSystemTorqueMapper(const std::string &urdf_filename,
                                               robot_config_t &robot_config,
                                               const std::string &general_config_file)
    : urdf_filename(urdf_filename), general_config_filename(general_config_file),
      robot_config(robot_config),
      is_kinematic_mpc(false),
      nq(robot_config.nq), nx(robot_config.nx),
      nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
      nq_fixed(robot_config.nq_fixed),
      n_indices(ConstIntVectorMap(robot_config.n_indices, nq_red)),
      n_indices_fixed(ConstIntVectorMap(robot_config.n_indices_fixed, nq_fixed)),
      q_ref_nq(ConstDoubleVectorMap(robot_config.q_0_ref, nq)),
      q_ref_fixed(Eigen::Map<const Eigen::VectorXd>(robot_config.q_0_ref, nq_fixed)(n_indices_fixed)),
      J_psi(calc_reduced_mapping_matrix()), J_psi_T(J_psi.transpose()),
      A(calc_coercive_condition_matrix()),
      dt(robot_config.dt)
{
    // Initialize the function pointer based on the type of MPC
    setFeedforwardTorqueFunction(is_kinematic_mpc);

    update_config();

    // Initialize the robot model and data using the URDF
    initRobot();

    // Initialize member matrices, biases, etc.
    tau_full = Eigen::VectorXd::Zero(nq);
    q_pp = Eigen::VectorXd::Zero(nq);
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

void FullSystemTorqueMapper::update_config()
{
    nlohmann::json general_config = read_config<>(general_config_filename);

    use_gravity = get_config_value<bool>(general_config, "use_gravity");
    tcp_frame_name = get_config_value<std::string>(general_config, "tcp_frame_name");

    auto torque_mapper_settings = get_config_value<nlohmann::json>(general_config, "torque_mapper_settings");
    config.K_d = Eigen::VectorXd::Map(get_config_value<nlohmann::json>(torque_mapper_settings, "K_d").get<std::vector<double>>().data(), robot_config.nq).asDiagonal();
    config.D_d = Eigen::VectorXd::Map(get_config_value<nlohmann::json>(torque_mapper_settings, "D_d").get<std::vector<double>>().data(), robot_config.nq).asDiagonal();
    config.K_d_fixed = Eigen::VectorXd::Map(get_config_value<nlohmann::json>(torque_mapper_settings, "K_d_fixed").get<std::vector<double>>().data(), robot_config.nq_fixed).asDiagonal();
    config.D_d_fixed = Eigen::VectorXd::Map(get_config_value<nlohmann::json>(torque_mapper_settings, "D_d_fixed").get<std::vector<double>>().data(), robot_config.nq_fixed).asDiagonal();
    config.q_ref_nq = q_ref_nq;
    config.q_ref_nq_fixed = q_ref_fixed;
    config.torque_limit = get_config_value<double>(torque_mapper_settings, "torque_limit");
}

void FullSystemTorqueMapper::update_config(double dt_new)
{
    update_config();
    dt = dt_new;

    // adapt D gain of Torque mapper to dt:
    // Reduce eigenvalues!
    double scale = robot_config.dt / dt;
    config.D_d *= scale;
    config.D_d_fixed *= scale;
    config.K_d *= (scale * scale);
    config.K_d_fixed *= (scale * scale);
}

// Implementation of kinematic torque mapping
Eigen::VectorXd FullSystemTorqueMapper::calcFeedforwardTorqueKinematic(
    const Eigen::VectorXd &u,
    const Eigen::VectorXd &q,
    const Eigen::VectorXd &q_p)
{
    // Assuming q_pp is properly defined elsewhere
    q_pp.setZero();      // Initialize q_pp if necessary
    q_pp(n_indices) = u; // Set specific index using input

    pinocchio::crba(robot_model_full, robot_data_full, q);
    robot_data_full.M.triangularView<Eigen::StrictlyLower>() = robot_data_full.M.transpose().triangularView<Eigen::StrictlyLower>();
    Eigen::MatrixXd M = robot_data_full.M;
    M_fixed = M(n_indices_fixed, n_indices_fixed);

    // Calculate the resulting torques
    return pinocchio::rnea(robot_model_full, robot_data_full, q, q_p, q_pp);
}

// Implementation of dynamic torque mapping
Eigen::VectorXd FullSystemTorqueMapper::calcFeedforwardTorqueDynamic(
    const Eigen::VectorXd &u,
    const Eigen::VectorXd &q,
    const Eigen::VectorXd &q_p)
{
    q_pp.setZero();              // Initialize q_pp
    Eigen::VectorXd tau_red = u; // Define reduced input vector

    // Calculate inertia matrix and Coriolis forces
    pinocchio::crba(robot_model_full, robot_data_full, q);
    robot_data_full.M.triangularView<Eigen::StrictlyLower>() = robot_data_full.M.transpose().triangularView<Eigen::StrictlyLower>();
    Eigen::MatrixXd M = robot_data_full.M;
    Eigen::VectorXd C_rnea = pinocchio::rnea(robot_model_full, robot_data_full, q, q_p, Eigen::VectorXd::Zero(nq));

    // Calculate reduced inertia matrix and reduced Coriolis forces
    pinocchio::crba(robot_model_reduced, robot_data_reduced, q(n_indices));
    robot_data_reduced.M.triangularView<Eigen::StrictlyLower>() = robot_data_reduced.M.transpose().triangularView<Eigen::StrictlyLower>();
    Eigen::MatrixXd M_red = robot_data_reduced.M;
    Eigen::VectorXd C_rnea_red = pinocchio::rnea(robot_model_reduced, robot_data_reduced, q(n_indices), q_p(n_indices), Eigen::VectorXd::Zero(nq_red));

    // Slice the inertia matrix and Coriolis forces
    // Eigen::MatrixXd M_red = M(n_indices, n_indices);
    M_fixed = M(n_indices_fixed, n_indices_fixed);
    // Eigen::VectorXd C_rnea_red = C_rnea(n_indices);

    // Compute the acceleration for the reduced system
    Eigen::VectorXd q_pp_red = M_red.ldlt().solve(tau_red - C_rnea_red);
    q_pp(n_indices) = q_pp_red; // Update full acceleration vector

    // Calculate the resulting full torques
    return pinocchio::rnea(robot_model_full, robot_data_full, q, q_p, q_pp);
}

Eigen::VectorXd FullSystemTorqueMapper::calcFeedforwardTorqueDynamicAlternative(
    const Eigen::VectorXd &u,
    const Eigen::VectorXd &q,
    const Eigen::VectorXd &q_p)
{
    Eigen::VectorXd q_mod = q;
    Eigen::VectorXd q_p_mod = q_p;
    q_mod(n_indices_fixed) = q_ref_fixed;
    q_p_mod(n_indices_fixed) = Eigen::VectorXd::Zero(nq_fixed);

    // Calculate inertia matrix and Coriolis forces
    pinocchio::crba(robot_model_full, robot_data_full, q_mod);
    robot_data_full.M.triangularView<Eigen::StrictlyLower>() = robot_data_full.M.transpose().triangularView<Eigen::StrictlyLower>();
    Eigen::MatrixXd M = robot_data_full.M;
    Eigen::VectorXd C_rnea = pinocchio::rnea(robot_model_full, robot_data_full, q_mod, q_p_mod, Eigen::VectorXd::Zero(nq));

    // Calculate the inverse inertia Matrix (Cholesky is faster and more stable than FullPivLU from M.inverse())
    Eigen::MatrixXd M_inv = robot_data_full.M.llt().solve(Eigen::MatrixXd::Identity(nq, nq));

    Eigen::MatrixXd F_psi = u;
    // Eigen::MatrixXd F_phi = A*M_inv*C_rnea -config.D_d_fixed * A*q_p - config.K_d_fixed * A*(q - q_ref_nq);
    Eigen::MatrixXd F_phi = A*M_inv*C_rnea -config.D_d_fixed * q_p(n_indices_fixed) - config.K_d_fixed * (q(n_indices_fixed) - config.q_ref_nq_fixed);

    // solve
    //                       ( [ J_psi^T ] ) [ F_psi ]
    // tau_c = tau_full = inv( [         ] ) [       ] = K * F
    //                       ( [ A*M_inv ] ) [ F_phi ]

    // Build composite matrix K
    Eigen::MatrixXd K(nq, nq);
    K.topRows(nq_red) = J_psi.transpose();
    K.bottomRows(nq_fixed) = A * M_inv;

    // Build composite vector F
    Eigen::VectorXd F(nq);
    F.head(nq_red) = F_psi;
    F.tail(nq_fixed) = F_phi;

    // Solve using QR decomposition (most efficient for rectangular full-rank systems)
    Eigen::VectorXd tau_c = K.colPivHouseholderQr().solve(F);
    return tau_c; // tau_c = tau_full
}

Eigen::VectorXd FullSystemTorqueMapper::calcFeedforwardTorqueKinematicAlternative(
    const Eigen::VectorXd &u,
    const Eigen::VectorXd &q,
    const Eigen::VectorXd &q_p)
{
    // inefficient:
    // Eigen::MatrixXd tau_psi = J_psi_T * M * J_psi * u + J_psi_T * (C * J_psi * q_p + g);

    // better: use directly reduced system and ID
    Eigen::VectorXd tau_psi = pinocchio::rnea(robot_model_reduced, robot_data_reduced, q(n_indices), q_p(n_indices), u);
    return calcFeedforwardTorqueDynamicAlternative(tau_psi, q, q_p);
}

void FullSystemTorqueMapper::setFeedforwardTorqueFunction(bool is_kinematic_mpc)
{
    if (is_kinematic_mpc)
    {
        // calcFeedforwardTorqueFunPtr = &FullSystemTorqueMapper::calcFeedforwardTorqueKinematic;
        calcFeedforwardTorqueFunPtr = &FullSystemTorqueMapper::calcFeedforwardTorqueKinematicAlternative;
    }
    else
    {
        // calcFeedforwardTorqueFunPtr = &FullSystemTorqueMapper::calcFeedforwardTorqueDynamic;
        calcFeedforwardTorqueFunPtr = &FullSystemTorqueMapper::calcFeedforwardTorqueDynamicAlternative;
    }
}

Eigen::VectorXd FullSystemTorqueMapper::applyPDControl(const Eigen::VectorXd &q_fixed,
                                                       const Eigen::VectorXd &q_p_fixed)
{
    return M_fixed * (-config.K_d_fixed * (q_fixed - config.q_ref_nq_fixed) - config.D_d_fixed * q_p_fixed);
}

Eigen::VectorXd FullSystemTorqueMapper::enforceTorqueLimits(const Eigen::VectorXd &tau)
{
    return tau.cwiseMin(Eigen::VectorXd::Constant(tau.size(), config.torque_limit))
        .cwiseMax(Eigen::VectorXd::Constant(tau.size(), -config.torque_limit));
}

Eigen::VectorXd FullSystemTorqueMapper::calc_full_torque(const Eigen::VectorXd &u, const Eigen::VectorXd &x_k_ndof)
{
    Eigen::Map<const Eigen::VectorXd> q(x_k_ndof.head(nq).data(), nq);
    Eigen::Map<const Eigen::VectorXd> q_p(x_k_ndof.tail(nq).data(), nq);
    tau_full = (this->*calcFeedforwardTorqueFunPtr)(u, q, q_p);
    // tau_full(n_indices_fixed) += applyPDControl(q(n_indices_fixed), q_p(n_indices_fixed));

    // tau_full = enforceTorqueLimits(tau_full); // Apply torque limits
    return tau_full;
}

void FullSystemTorqueMapper::calcPose(const double *x, Eigen::Vector3d &p, Eigen::Matrix3d &R)
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

    x_k_ndof.head(nq) = q + q_p * dt;    // q_next = q + q_p * dt
    x_k_ndof.tail(nq) = q_p + q_pp * dt; // q_p_next = q_p + q_pp * dt
}

void FullSystemTorqueMapper::simulateModelRK4(Eigen::Map<Eigen::VectorXd> &x_k_ndof, Eigen::Map<const Eigen::VectorXd> &tau, double dt)
{
    // Define lambda for computing f(x, u) = [q_p, q_pp] = [x2, u]
    auto f = [&](const Eigen::VectorXd &state) -> Eigen::VectorXd
    {
        // Extract positions and velocities from the state vector
        Eigen::Ref<const Eigen::VectorXd> q = state.head(nq);
        Eigen::Ref<const Eigen::VectorXd> q_p = state.tail(nq);

        // Compute joint accelerations using Pinocchio's ABA
        Eigen::VectorXd q_pp = pinocchio::aba(robot_model_full, robot_data_full, q, q_p, tau);

        // Combine derivatives: [q_p; q_pp]
        Eigen::VectorXd derivative(2 * nq);
        derivative.head(nq) = q_p;  // dq/dt = q_p
        derivative.tail(nq) = q_pp; // dq_p/dt = q_pp

        return derivative;
    };
    x_k_ndof = RK4(x_k_ndof, dt, f);
}

Eigen::VectorXd FullSystemTorqueMapper::RK4(const Eigen::VectorXd &state, double dt, const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &f)
{
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

void FullSystemTorqueMapper::initRobot()
{
    // Load the URDF file into the robot model
    pinocchio::urdf::buildModel(urdf_filename, robot_model_full);

    //////////////////// Build reduced robot model
    //iterate all joints and lock the ones that are fixed
    // Joint 0 is universe joint, so we add 1 to indices
    std::vector<pinocchio::JointIndex> joints_to_lock;
    for (uint32_t i = 0; i < robot_config.nq_fixed; i++)
    {
        const std::string & joint_name = robot_model_full.names[robot_config.n_indices_fixed[i]+1];
        joints_to_lock.push_back(robot_model_full.getJointId(joint_name));
    }

    //////////////////// 

    robot_model_reduced = pinocchio::buildReducedModel(robot_model_full, joints_to_lock, q_ref_nq);

    if (use_gravity)
    {
        robot_model_full.gravity.linear() << 0.0, 0.0, -9.81;
        robot_model_reduced.gravity.linear() << 0.0, 0.0, -9.81;
    }
    else
    {
        robot_model_full.gravity.linear() << 0.0, 0.0, 0.0;
        robot_model_reduced.gravity.linear() << 0.0, 0.0, 0.0;
    }

    // Initialize the corresponding data structure
    robot_data_full = pinocchio::Data(robot_model_full);
    robot_data_reduced = pinocchio::Data(robot_model_reduced);

    // Initialize the TCP frame Id
    tcp_frame_id = robot_model_full.getFrameId(tcp_frame_name);
}

Eigen::MatrixXd FullSystemTorqueMapper::calc_reduced_mapping_matrix()
{
    Eigen::MatrixXd J_psi = Eigen::MatrixXd::Zero(nq, nq_red);
    for (uint i = 0; i < nq_red; i++)
    {
        J_psi(n_indices(i), i) = 1;
    }
    return J_psi;
}

Eigen::MatrixXd FullSystemTorqueMapper::calc_coercive_condition_matrix()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nq_fixed, nq);
    for (uint i = 0; i < nq_fixed; i++)
    {
        A(i, n_indices_fixed(i)) = 1;
    }
    return A;
}