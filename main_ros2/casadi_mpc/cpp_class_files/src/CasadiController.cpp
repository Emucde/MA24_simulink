#include "CasadiController.hpp"

CasadiController::CasadiController(const std::string &urdf_path, bool use_gravity)
    : urdf_path(urdf_path),
      robot_config(get_robot_config()),
      nq(robot_config.nq), nx(robot_config.nx), nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
      n_x_indices(robot_config.n_x_indices),
      torque_mapper(urdf_path, robot_config, use_gravity, true)
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