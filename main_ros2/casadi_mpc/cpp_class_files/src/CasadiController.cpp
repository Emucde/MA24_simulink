#include "CasadiController.hpp"
#include "eigen_templates.hpp"

CasadiController::CasadiController(const std::string &urdf_path, const std::string &tcp_frame_name, bool use_gravity)
    : robot_config(get_robot_config()),
      nq(robot_config.nq), nx(robot_config.nx), nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
      n_indices(ConstIntVectorMap(robot_config.n_indices, nq_red)),
      n_x_indices(ConstIntVectorMap(robot_config.n_x_indices, nx_red)),
      torque_mapper(urdf_path, tcp_frame_name, robot_config, use_gravity, true)
{
    // Read the trajectory data
    traj_file = robot_config.traj_data_path;
    all_traj_data = readTrajectoryData(traj_file);

    int default_traj_select = 1;
    traj_rows = all_traj_data[default_traj_select-1].rows();
    traj_len = all_traj_data[default_traj_select-1].cols();
    traj_data.resize(traj_rows, traj_len);
    traj_data << all_traj_data[default_traj_select-1];

    // Real length of the singular trajectory data without additional samples for last prediction horizon
    traj_real_len = robot_config.traj_data_real_len; // transient trajectory length = 0

    // Default: no transient trajectory
    setTransientTrajParams(0.0, 0.0, 0.0);
    transient_traj_cnt = 0;
    transient_traj_len = 0;
    traj_rows = 7; // Default value

    // Initialize MPC objects
    for (int i = 0; i < static_cast<int>(MPCType::COUNT); ++i)
    {
        MPCType mpc = static_cast<MPCType>(i);
        if (mpc != MPCType::INVALID)
        { // Ensures we are within valid enum range
            std::string mpcName = mpcToString(mpc);
            casadi_mpcs.push_back(CasadiMPC(mpcName, robot_config, &traj_data, traj_real_len)); // Initialize all MPCs
        }
    }

    setActiveMPC(MPCType::MPC8); // Set the default MPC

    // Initialize the previous torque
    tau_full_prev = Eigen::VectorXd::Zero(nq);
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
    Eigen::Map<const Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);
    double x_k[nx_red];

    // Convert nx to nx_red state
    for (casadi_uint i = 0; i < nx_red; i++)
    {
        x_k[i] = x_k_ndof[n_x_indices[i]];
    }

    // Solve the MPC
    int flag = active_mpc->solve(x_k); // uses internal the pointer x_k_ptr
    if (flag)
    {
        std::cerr << "Error in Casadi function call." << std::endl;
        error_flag = ErrorFlag::CASADI_ERROR;
        tau_full_prev = tau_full; // zero torque
        return tau_full;          // Return zero torque
    }

    // Calculate the full torque (Feedforward + PD control for fixed joints)
    Eigen::Map<Eigen::VectorXd> u_k(u_k_ptr, nq_red);

    tau_full = torque_mapper.calc_full_torque(u_k, x_k_ndof);

    if (!tau_full.allFinite())
    {
        std::cout << "NaN in torque detected (tau = " << tau_full.transpose() << "). Output zero torque." << std::endl;
        tau_full.setZero(); // Set torque to zero
        error_flag = ErrorFlag::NAN_DETECTED;
    }
    else
    {
        // Check for a jump in torque
        Eigen::VectorXd delta_u = tau_full - tau_full_prev;

        // Conditions for jumps
        bool condition1 = (delta_u.array() > 0 && delta_u.array() > tau_max_jump).any();
        bool condition2 = (delta_u.array() < 0 && delta_u.array() < -tau_max_jump).any();

        if (condition1 || condition2)
        {
            std::cout << "Jump in torque detected (tau = " << tau_full.transpose() << "). Output zero torque." << std::endl;
            tau_full.setZero(); // Set torque to zero
            error_flag = ErrorFlag::JUMP_DETECTED;
        }
        else
        {
            tau_full_prev = tau_full; // Update previous torque
            error_flag = ErrorFlag::NO_ERROR;
        }
    }

    return tau_full;
}

void CasadiController::init_trajectory(casadi_uint traj_select, const casadi_real *const x_k_ndof_ptr,
                                       double T_start, double T_poly, double T_end)
{
    setTransientTrajParams(T_start, T_poly, T_end);
    init_trajectory(traj_select, x_k_ndof_ptr); // Call the other init_trajectory with transient trajectory
}

void CasadiController::init_trajectory(casadi_uint traj_select, const casadi_real *const x_k_ndof_ptr)
{
    double x_k[nx_red];

    if (x_k_ndof_ptr != nullptr)
    {
        generate_transient_trajectory(x_k_ndof_ptr); // Generate transient trajectory
    }
    else
    {
        transient_traj_data = Eigen::MatrixXd::Zero(traj_rows, 0); // No transient trajectory
    }

    // Check if the trajectory selection is valid
    if (traj_select < 1 || traj_select > all_traj_data.size())
    {
        std::cerr << "Invalid trajectory selection. Selecting Trajectory 1" << std::endl;
        traj_select = 1;
    }

    // Concatenate trajectories
    traj_data.resize(traj_rows, transient_traj_data.cols() + all_traj_data[traj_select - 1].cols());
    traj_data << transient_traj_data, all_traj_data[traj_select - 1];
    traj_len = traj_data.cols();
    traj_real_len = robot_config.traj_data_real_len + transient_traj_data.cols();

    std::cout << "Traj:" << traj_data.rows() << "x" << traj_data.cols() << std::endl;

    // Convert nx to nx_red state
    for (casadi_uint i = 0; i < nx_red; i++)
    {
        x_k[i] = x_k_ndof_ptr[n_x_indices[i]];
    }

    // send trajectory pointer to all MPCs
    for (int i = 0; i < static_cast<int>(MPCType::COUNT); ++i)
    {
        if (static_cast<MPCType>(i) != MPCType::INVALID)
        {
            casadi_mpcs[i].switch_traj(&traj_data, x_k, traj_real_len);
        }
    }

}

void CasadiController::init_trajectory(casadi_uint traj_select)
{
    init_trajectory(traj_select, nullptr); // Call the other init_trajectory with no transient trajectory
}

// Method to generate a transient trajectory
void CasadiController::generate_transient_trajectory(const casadi_real *const x_k_ndof_ptr,
                                                     double T_start, double T_poly, double T_end)
{
    setTransientTrajParams(T_start, T_poly, T_end);
    generate_transient_trajectory(x_k_ndof_ptr);
}

void CasadiController::generate_transient_trajectory(const casadi_real *const x_k_ndof_ptr)
{
    Eigen::Vector3d p_init, p_target;
    Eigen::Matrix3d R_init, R_target;

    // only map the first nq elements to get joint angles
    Eigen::Map<const Eigen::VectorXd> q_init_nq(x_k_ndof_ptr, nq);
    Eigen::Map<const Eigen::VectorXd> q_target_nq(robot_config.q_0_ref, nq);

#ifdef DEBUG
    std::cout << "q_init_nq: " << q_init_nq.transpose() << std::endl;
    std::cout << "q_init_nq: " << q_target_nq.transpose() << std::endl;
#endif

    // Calculate the pose from the state
    torque_mapper.calcPose(q_init_nq, p_init, R_init);

    // Get the target state from the trajectory file
    torque_mapper.calcPose(q_target_nq, p_target, R_target);

#ifdef DEBUG
    std::cout << "q_init_nq: " << p_init.transpose() << std::endl;
    std::cout << "q_init_nq: " << p_target.transpose() << std::endl;
#endif

    // Generate the transient trajectory
    transient_traj_data = generate_trajectory(dt, p_init, p_target, R_init, R_target, param_transient_traj_poly);
    transient_traj_len = transient_traj_data.cols();
    traj_rows = transient_traj_data.rows();
    transient_traj_cnt = 0;
}

// set the active MPC
void CasadiController::setActiveMPC(MPCType mpc_type)
{
    if (mpc_type != MPCType::INVALID)
    {
        selected_mpc_type = mpc_type;
        active_mpc = &casadi_mpcs[static_cast<int>(selected_mpc_type)];
        active_mpc_config = const_cast<mpc_config_t *>(active_mpc->get_mpc_config());
        active_mpc_input_config = &active_mpc_config->input_config;

        torque_mapper.set_kinematic_mpc_flag(active_mpc->is_kinematic_mpc);
        x_k_ptr = active_mpc->get_x_k();
        u_k_ptr = active_mpc->get_optimal_control();
        y_d_ptr = active_mpc->get_y_d();
        w_ptr = active_mpc->get_w();
        traj_data_per_horizon = active_mpc->get_traj_data_per_horizon_len();
        mpc_traj_indices = const_cast<casadi_uint *>(active_mpc->get_mpc_traj_indices());
        dt = active_mpc->get_dt();
    }
    else
    {
        std::cerr << "Invalid MPC type." << std::endl;
    }
}

// set the transient trajectory parameters
void CasadiController::setTransientTrajParams(double T_start, double T_poly, double T_end)
{
    if (T_start < 0)
    {
        throw std::invalid_argument("T_start must be greater than or equal to 0 (start time of the transient trajectory)");
    }
    if (T_start > T_poly)
    {
        throw std::invalid_argument("T_start must be less than or equal to T_poly (total time of the polynomial trajectory)");
    }
    if (T_poly > T_end)
    {
        throw std::invalid_argument("T_poly must be less than or equal to T_end (total time of the transient trajectory)");
    }

    param_transient_traj_poly["T_start"] = T_start;
    param_transient_traj_poly["T_poly"] = T_poly;
    param_transient_traj_poly["T_end"] = T_end;
}

// Method to simulate the robot model
void CasadiController::simulateModel(casadi_real *const x_k_ndof_ptr, const casadi_real *const tau_ptr, double dt)
{
    Eigen::Map<Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);
    Eigen::Map<const Eigen::VectorXd> tau(tau_ptr, nq);
    torque_mapper.simulateModel(x_k_ndof, tau, dt);
}

// Method to switch the trajectory
void CasadiController::switch_traj(casadi_uint traj_select, const casadi_real *const x_k_ndof)
{
    if (traj_select < 1 || traj_select > all_traj_data.size())
    {
        std::cerr << "Invalid trajectory selection. Selecting Trajectory 1" << std::endl;
        traj_select = 1;
    }
    init_trajectory(traj_select, x_k_ndof);
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
    result << p_d.head<3>(), q_d.w(), q_d.x(), q_d.y(), q_d.z(); // important: q_d.coeffs() returns the quaternion in the form {x, y, z, w}

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

std::vector<Eigen::MatrixXd> CasadiController::readTrajectoryData(const std::string &traj_file)
{
    std::ifstream file(traj_file, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Error opening file: " + traj_file);
    }

    // Read dimensions
    uint32_t rows, cols, traj_amount;
    file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
    file.read(reinterpret_cast<char *>(&cols), sizeof(cols));
    file.read(reinterpret_cast<char *>(&traj_amount), sizeof(traj_amount));

#ifdef DEBUG
    std::cout << "Rows: " << rows << ", Cols: " << cols << ", Trajectory Amount: " << traj_amount << std::endl;
#endif

    // Create a vector to hold all trajectories
    std::vector<Eigen::MatrixXd> trajectories(traj_amount, Eigen::MatrixXd(rows, cols));

    // Read the trajectory data
    for (size_t i = 0; i < traj_amount; ++i)
    {
        file.read(reinterpret_cast<char *>(trajectories[i].data()), rows * cols * sizeof(double));
        if (!file)
        {
            throw std::runtime_error("Error reading trajectory data from file: " + traj_file);
        }
    }

#ifdef DEBUG
    // Debug: Print first trajectory as an example
    std::cout << "First trajectory:\n"
              << trajectories[0] << std::endl;
#endif

    return trajectories;
}