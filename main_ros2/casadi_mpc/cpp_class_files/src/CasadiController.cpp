#include "CasadiController.hpp"

CasadiController::CasadiController(const std::string &urdf_path, const std::string &tcp_frame_name, bool use_gravity)
    : robot_config(get_robot_config()),
      nq(robot_config.nq), nx(robot_config.nx), nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
      n_x_indices(robot_config.n_x_indices),
      torque_mapper(urdf_path, tcp_frame_name, robot_config, use_gravity, true)
{
    // Initialize MPC objects
    for (int i = 0; i < static_cast<int>(MPCType::COUNT); ++i)
    {
        MPCType mpc = static_cast<MPCType>(i);
        if (mpc != MPCType::INVALID)
        { // Ensures we are within valid enum range
            std::string mpcName = mpcToString(mpc);
            casadi_mpcs.push_back(CasadiMPC(mpcName, robot_config)); // Initialize all MPCs
        }
    }

    setActiveMPC(MPCType::MPC8); // Set the default MPC
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

// Solve one MPC iteration and result a map to the optimal control
Eigen::VectorXd CasadiController::solveMPC(const casadi_real *const x_k_ndof_ptr)
{
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);

    // Copy the state to the MPC object
    int cnt = 0;
    for (casadi_uint i = 0; i < nx; ++i)
    {
        if (i == n_x_indices[cnt])
        {
            x_k_ptr[cnt] = x_k_ndof_ptr[i];
            cnt++;
        }
    }

    int flag = active_mpc->solve(); // uses internal the pointer x_k_ptr
    if (flag)
    {
        std::cerr << "Error in Casadi function call." << std::endl;
        return tau_full; // Return zero torque
    }

    Eigen::VectorXd u_k = Eigen::Map<Eigen::VectorXd>(u_k_ptr, nq_red);
    Eigen::VectorXd x_k_ndof = Eigen::Map<const Eigen::VectorXd>(x_k_ndof_ptr, nx);

    tau_full = torque_mapper.calc_full_torque(u_k, x_k_ndof);
    return tau_full;
}

// TODO:
// 1. aktuellen Zustand lesen
// 2. Pose von aktuellen Zustand berechnen: pe0 und R_init (pinocchio, homogene transformation)
// 3. Ersten Wert der Trajektorie lesen: pe0 und qe0(convert to R_target)
// 4. Trajektorie berechnen:
//      generate_trajectory(double dt, const Eigen::Vector4d &xe0, const Eigen::Vector4d &xeT,
//                              const Eigen::Matrix3d &R_init, const Eigen::Matrix3d &R_target,
//                              const std::map<std::string, double> &param_traj_poly)
// 5. Logik für Transiente trajektorie und umschalten zu der aus file.

// Method to generate a transient trajectory
Eigen::MatrixXd CasadiController::generate_transient_trajectory(const casadi_real *const x_k_ndof_ptr,
                                                                const std::map<std::string, double> &param_traj_poly)
{
    Eigen::Vector3d p_init, p_target;
    Eigen::Matrix3d R_init, R_target;

    // only map the first nq elements to get joint angles
    Eigen::Map<const Eigen::VectorXd> q_init_nq(x_k_ndof_ptr, nq);
    Eigen::Map<const Eigen::VectorXd> q_target_nq(get_x_ref_nq().data(), nq);

    std::cout << "q_init_nq: " << q_init_nq.transpose() << std::endl;
    std::cout << "q_init_nq: " << q_target_nq.transpose() << std::endl;

    // Calculate the pose from the state
    torque_mapper.calcPose(q_init_nq, p_init, R_init);

    // Get the target state from the trajectory file
    torque_mapper.calcPose(q_target_nq, p_target, R_target);

    std::cout << "q_init_nq: " << p_init.transpose() << std::endl;
    std::cout << "q_init_nq: " << p_target.transpose() << std::endl;

    // Generate the transient trajectory
    Eigen::MatrixXd traj_data = generate_trajectory(dt, p_init, p_target, R_init, R_target, param_traj_poly);

    return traj_data;
}

// set the active MPC
void CasadiController::setActiveMPC(MPCType mpc_type)
{
    if (mpc_type != MPCType::INVALID)
    {
        selected_mpc_type = mpc_type;
        active_mpc = &casadi_mpcs[static_cast<int>(selected_mpc_type)];
        torque_mapper.set_kinematic_mpc_flag(active_mpc->is_kinematic_mpc);
        x_k_ptr = active_mpc->get_x_k();
        u_k_ptr = active_mpc->get_optimal_control();
        dt = active_mpc->get_dt();
    }
    else
    {
        std::cerr << "Invalid MPC type." << std::endl;
    }
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

std::string CasadiController::mpcToString(MPCType mpc)
{
    switch (mpc)
    {
    case MPCType::MPC01:
        return "MPC01";
    case MPCType::MPC6:
        return "MPC6";
    case MPCType::MPC7:
        return "MPC7";
    case MPCType::MPC8:
        return "MPC8";
    case MPCType::MPC9:
        return "MPC9";
    case MPCType::MPC10:
        return "MPC10";
    case MPCType::MPC11:
        return "MPC11";
    case MPCType::MPC12:
        return "MPC12";
    case MPCType::MPC13:
        return "MPC13";
    case MPCType::MPC14:
        return "MPC14";
    default:
        return "INVALID";
    }
}

Eigen::Vector4d CasadiController::trajectory_poly(double t, const Eigen::Vector4d &y0, const Eigen::Vector4d &yT, double T)
{
    Eigen::Vector4d y_d;

    // y_d = y0 + (6.0 / (T * T * T * T * T) * t * t * t * t * t -
    //              15.0 / (T * T * T * T) * t * t * t * t +
    //              10.0 * t * t * t / (T * T * T)) * (yT - y0);

    double t_T = t / T;
    double t_T2 = t_T * t_T;
    double t_T3 = t_T2 * t_T;
    double t_T4 = t_T3 * t_T;
    double t_T5 = t_T4 * t_T;

    y_d = y0 + (6.0 * t_T5 - 15.0 * t_T4 + 10.0 * t_T3) * (yT - y0);
    return y_d;
}

Eigen::VectorXd CasadiController::create_poly_traj(const Eigen::Vector3d &yT, const Eigen::Vector3d &y0, double t,
                                                   const Eigen::Matrix3d &R_init, const Eigen::Vector3d &rot_ax,
                                                   double rot_alpha_scale, const std::map<std::string, double> &param_traj_poly)
{
    double T_start = param_traj_poly.at("T_start");
    double T_poly = param_traj_poly.at("T_poly");
    Eigen::Vector4d p_d;

    if (t - T_start < 0)
    {
        p_d << y0[0], y0[1], y0[2], 0.0;
    }
    else if (t - T_start > T_poly)
    {
        p_d << yT[0], yT[1], yT[2], rot_alpha_scale;
    }
    else
    {
        p_d = trajectory_poly(t - T_start,
                              Eigen::Vector4d(y0[0], y0[1], y0[2], 0.0),
                              Eigen::Vector4d(yT[0], yT[1], yT[2], rot_alpha_scale),
                              T_poly);
    }

    double alpha = p_d[3];

    Eigen::Matrix3d skew_ew;
    skew_ew << 0, -rot_ax[2], rot_ax[1],
        rot_ax[2], 0, -rot_ax[0],
        -rot_ax[1], rot_ax[0], 0;

    Eigen::Matrix3d R_act = (Eigen::Matrix3d::Identity() + sin(alpha) * skew_ew +
                             (1 - cos(alpha)) * skew_ew * skew_ew) *
                            R_init;

    Eigen::Quaterniond q_d(R_act);

    // Create the final vector to hold both position and quaternion
    Eigen::VectorXd result(7);
    result << p_d.head<3>(), q_d.coeffs(); // q_d.coeffs() returns the quaternion in the form {x, y, z, w}

    return result;
}

Eigen::MatrixXd CasadiController::generate_trajectory(double dt, const Eigen::Vector3d &xe0, const Eigen::Vector3d &xeT,
                                                      const Eigen::Matrix3d &R_init, const Eigen::Matrix3d &R_target,
                                                      const std::map<std::string, double> &param_traj_poly)
{
    double T_end = param_traj_poly.at("T_end");

    int N = static_cast<int>(T_end / dt);
    Eigen::MatrixXd traj_data(7, N);

    Eigen::Matrix3d RR = R_target * R_init.transpose();
    Eigen::Quaterniond rot_quat(RR);

    // Convert quaternion to rotation matrix, then get axis and angle
    Eigen::Vector3d rot_vec = rot_quat.vec(); // The vector part (x, y, z)
    double rot_rho = rot_quat.w();            // The scalar part (w)

    double rot_alpha_scale = 2 * acos(rot_rho); // Angle from the quaternion
    Eigen::Vector3d rot_ax;

    if (rot_alpha_scale == 0)
    {
        rot_ax = Eigen::Vector3d(0, 0, 0); // Random axis because rotation angle is 0
    }
    else
    {
        rot_ax = rot_vec / sin(rot_alpha_scale / 2); // Normalize the axis
    }

    if (rot_alpha_scale > M_PI)
    {
        rot_alpha_scale = 2 * M_PI - rot_alpha_scale;
        rot_ax = -rot_ax;
    }

    double current_time = 0.0;

    for (int i = 0; i < N; i++)
    {
        current_time = i * dt;
        Eigen::VectorXd x_d = create_poly_traj(xeT, xe0, current_time, R_init, rot_ax, rot_alpha_scale, param_traj_poly);
        traj_data.col(i) = x_d;
    }

    return traj_data;
}