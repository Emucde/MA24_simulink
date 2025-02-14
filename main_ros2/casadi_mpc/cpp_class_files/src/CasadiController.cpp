#include "CasadiController.hpp"
#include "eigen_templates.hpp"

CasadiController::CasadiController(const std::string &urdf_path, const std::string &tcp_frame_name, bool use_gravity)
    : robot_config(get_robot_config()),
      nq(robot_config.nq), nx(robot_config.nx), nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
      n_indices(ConstIntVectorMap(robot_config.n_indices, nq_red)),
      n_x_indices(ConstIntVectorMap(robot_config.n_x_indices, nx_red)),
      torque_mapper(urdf_path, tcp_frame_name, robot_config, use_gravity, true),
      trajectory_generator(torque_mapper, robot_config.dt)
{
    // Initialize MPC objects
    for (int i = 0; i < static_cast<int>(CasadiMPCType::COUNT); ++i)
    {

        CasadiMPCType mpc = static_cast<CasadiMPCType>(i);
        if (mpc != CasadiMPCType::INVALID)
        {                                                                                                                                             // Ensures we are within valid enum range
            casadi_mpcs.push_back(CasadiMPC(mpc, robot_config, trajectory_generator.get_traj_data(), trajectory_generator.get_traj_data_real_len())); // Initialize all MPCs
        }
    }

    setActiveMPC(CasadiMPCType::MPC8); // Set the default MPC
    switch_traj(1); // Set the default trajectory

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
            error_flag = ErrorFlag::JUMP_DETECTED;
            std::cout << "Jump in torque detected (tau = " << tau_full.transpose() << "). Output zero torque." << std::endl;
            tau_full.setZero(); // Set torque to zero
        }
        else
        {
            error_flag = ErrorFlag::NO_ERROR;
            tau_full_prev = tau_full; // Update previous torque
        }
    }

    return tau_full;
}

void CasadiController::init_file_trajectory(casadi_uint traj_select, const casadi_real *x_k_ndof_ptr,
                                            double T_start, double T_poly, double T_end)
{
    trajectory_generator.init_file_trajectory(traj_select, x_k_ndof_ptr, T_start, T_poly, T_end);
    update_trajectory_data(x_k_ndof_ptr);
}

void CasadiController::init_custom_trajectory(ParamPolyTrajectory param)
{
    trajectory_generator.init_custom_trajectory(param);
    update_trajectory_data(param.x_init.data());
}

void CasadiController::switch_traj(casadi_uint traj_select)
{
    trajectory_generator.switch_traj(traj_select);
    update_trajectory_data(trajectory_generator.get_traj_x0_init()->data());
}

void CasadiController::update_trajectory_data(const casadi_real *const x_k_ndof_ptr)
{
    // Convert nx to nx_red state
    double x_k[nx_red];
    for (casadi_uint i = 0; i < nx_red; i++)
    {
        x_k[i] = x_k_ndof_ptr[n_x_indices[i]];
    }

    // send trajectory pointer to all MPCs
    for (int i = 0; i < static_cast<int>(CasadiMPCType::COUNT); ++i)
    {
        if (static_cast<CasadiMPCType>(i) != CasadiMPCType::INVALID)
        {
            casadi_mpcs[i].switch_traj(trajectory_generator.get_traj_data(), x_k, trajectory_generator.get_traj_data_real_len());
        }
    }
}

// set the active MPC
void CasadiController::setActiveMPC(CasadiMPCType mpc_type)
{
    if (mpc_type != CasadiMPCType::INVALID)
    {
        selected_mpc_type = mpc_type;
        active_mpc = &casadi_mpcs[static_cast<int>(selected_mpc_type)];

        active_mpc_config = active_mpc->get_mpc_config();
        active_mpc_input_config = &active_mpc_config->in;

        torque_mapper.set_kinematic_mpc_flag(active_mpc->is_kinematic_mpc);
        u_k_ptr = active_mpc->get_optimal_control();

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

// Method to simulate the robot model
void CasadiController::simulateModelEuler(casadi_real *const x_k_ndof_ptr, const casadi_real *const tau_ptr, double dt)
{
    Eigen::Map<Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);
    Eigen::Map<const Eigen::VectorXd> tau(tau_ptr, nq);
    torque_mapper.simulateModelEuler(x_k_ndof, tau, dt);

    // Check for errors
    if (x_k_ndof.hasNaN())
    {
        error_flag = ErrorFlag::NAN_DETECTED;
        std::cerr << "NaN values detected in the joint vector!" << std::endl;
    }
}

// Method to simulate the robot model
void CasadiController::simulateModelRK4(casadi_real *const x_k_ndof_ptr, const casadi_real *const tau_ptr, double dt)
{
    Eigen::Map<Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);
    Eigen::Map<const Eigen::VectorXd> tau(tau_ptr, nq);
    torque_mapper.simulateModelRK4(x_k_ndof, tau, dt);

    // Check for errors
    if (x_k_ndof.hasNaN())
    {
        error_flag = ErrorFlag::NAN_DETECTED;
        std::cerr << "NaN values detected in the joint vector!" << std::endl;
    }
}

void CasadiController::reset()
{
    active_mpc->reset();
    tau_full_prev = Eigen::VectorXd::Zero(nq);
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