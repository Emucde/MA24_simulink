#include "CrocoddylController.hpp"

CrocoddylController::CrocoddylController(const std::string &urdf_path,
                                         const std::string &crocoddyl_config_path,
                                         const std::string &general_config_file)
    : urdf_path(urdf_path), crocoddyl_config_path(crocoddyl_config_path), general_config_file(general_config_file),
      robot_config(get_robot_config()),
      n_indices(ConstIntVectorMap(robot_config.n_indices, robot_config.nq_red)),
      n_x_indices(ConstIntVectorMap(robot_config.n_x_indices, robot_config.nx_red)),
      nq(robot_config.nq), nx(robot_config.nx), nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
      robot_model(urdf_path, robot_config, general_config_file, true), // use reduced model
      torque_mapper(urdf_path, robot_config, general_config_file),
      trajectory_generator(torque_mapper, robot_config.dt)
{
    // Initialize MPC objects
    for (int i = 0; i < static_cast<int>(CrocoddylMPCType::COUNT); ++i)
    {
        CrocoddylMPCType mpc = static_cast<CrocoddylMPCType>(i);
        if (mpc != CrocoddylMPCType::INVALID)
        {                                                                                                          // Ensures we are within valid enum range
            crocoddyl_mpcs.push_back(CrocoddylMPC(mpc, robot_model, crocoddyl_config_path, trajectory_generator)); // Initialize all MPCs
        }
    }

    nlohmann::json general_config = read_config(general_config_file);

    setActiveMPC(CrocoddylMPCType::DynMPC_v1); // Set the default MPC
    
    switch_traj(get_config_value<uint>(general_config, "trajectory_selection"));

    // Initialize the previous torque
    tau_full_prev = Eigen::VectorXd::Zero(nq);
}

nlohmann::json CrocoddylController::read_config(std::string file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: Could not open JSON file." << std::endl;
        return {};
    }
    nlohmann::json jsonData;
    file >> jsonData; // Parse JSON file
    file.close();
    return jsonData;
}

void CrocoddylController::init_file_trajectory(uint traj_select, const double *x_k_ndof_ptr,
                                               double T_start, double T_poly, double T_end)
{
    trajectory_generator.init_file_trajectory(traj_select, x_k_ndof_ptr, T_start, T_poly, T_end);
    update_trajectory_data(x_k_ndof_ptr);
}

// Method for creating a custom trajectory with extra samples for the last prediction horizon
void CrocoddylController::init_custom_trajectory(ParamPolyTrajectory param)
{
    trajectory_generator.init_custom_trajectory(param);
    update_trajectory_data(param.x_init.data());
}

void CrocoddylController::switch_traj(uint traj_select)
{
    trajectory_generator.switch_traj(traj_select);
    update_trajectory_data(trajectory_generator.get_traj_x0_init()->data());
}

void CrocoddylController::update_trajectory_data(const double *x_k_ndof_ptr)
{
    Eigen::Map<const Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);
    Eigen::VectorXd x_k = Eigen::VectorXd::Zero(nx_red);
    x_k << x_k_ndof(n_x_indices);

    // send trajectory pointer to all MPCs
    for (int i = 0; i < static_cast<int>(CrocoddylMPCType::COUNT); ++i)
    {
        if (static_cast<CrocoddylMPCType>(i) != CrocoddylMPCType::INVALID)
        {
            crocoddyl_mpcs[i].switch_traj(x_k);
        }
    }
}

Eigen::VectorXd CrocoddylController::solveMPC(const double *const x_k_ndof_ptr)
{
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);
    Eigen::Map<const Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);
    Eigen::VectorXd x_k = x_k_ndof(n_x_indices);
    
    bool hasConverged = active_mpc->solve(x_k);
    if (!hasConverged)
    {
        std::cerr << "Error in Crocoddyl function call." << std::endl;
        error_flag = ErrorFlag::CROCODDYL_ERROR;
        tau_full_prev = tau_full; // zero torque
        reset(x_k.data());
        return tau_full;          // Return zero torque
    }

    u_k_ptr = active_mpc->get_optimal_control();
    Eigen::Map<Eigen::VectorXd> u_k(u_k_ptr, nq_red);

    tau_full = torque_mapper.calc_full_torque(u_k, x_k_ndof);

    error_check(tau_full);
    return tau_full;
}

void CrocoddylController::update_mpc_weights()
{
    for (int i = 0; i < static_cast<int>(CrocoddylMPCType::COUNT); ++i)
    {
        if (static_cast<CrocoddylMPCType>(i) != CrocoddylMPCType::INVALID)
        {
            crocoddyl_mpcs[i].init_config();
            crocoddyl_mpcs[i].create_mpc_solver();
        }
    }
}

void CrocoddylController::setActiveMPC(CrocoddylMPCType mpc_type)
{
    if (mpc_type != CrocoddylMPCType::INVALID)
    {
        selected_mpc_type = mpc_type;
        active_mpc = &crocoddyl_mpcs[static_cast<int>(selected_mpc_type)];
        torque_mapper.set_kinematic_mpc_flag(active_mpc->is_kinematic);
        u_k_ptr = active_mpc->get_optimal_control();
    }
    else
    {
        std::cerr << "Invalid MPC type." << std::endl;
    }
}

void CrocoddylController::simulateModelEuler(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt)
{
    Eigen::Map<Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);

    // Check for errors
    if (x_k_ndof.hasNaN())
    {
        error_flag = ErrorFlag::NAN_DETECTED;
        std::cerr << "NaN values detected in the joint vector!" << std::endl;
        return;
    }

    Eigen::Map<const Eigen::VectorXd> tau(tau_ptr, nq);
    torque_mapper.simulateModelEuler(x_k_ndof, tau, dt);
}


void CrocoddylController::simulateModelRK4(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt)
{
    Eigen::Map<Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);

    // Check for errors
    if (x_k_ndof.hasNaN())
    {
        error_flag = ErrorFlag::NAN_DETECTED;
        std::cerr << "NaN values detected in the joint vector!" << std::endl;
        return;
    }

    Eigen::Map<const Eigen::VectorXd> tau(tau_ptr, nq);
    torque_mapper.simulateModelRK4(x_k_ndof, tau, dt);
}

void CrocoddylController::reset(const casadi_real *const x_k_in)
{
    active_mpc->reset(Eigen::Map<const Eigen::VectorXd>(x_k_in, nx_red));
    tau_full_prev = Eigen::VectorXd::Zero(nq);
    reset_error_flag();
}

void CrocoddylController::reset()
{
    Eigen::VectorXd x0_init = get_act_traj_x0_red_init();
    active_mpc->reset(x0_init);
    tau_full_prev = Eigen::VectorXd::Zero(nq);
    reset_error_flag();
}

Eigen::VectorXd CrocoddylController::get_traj_x0_red_init(casadi_uint traj_select)
{
    Eigen::VectorXd x0_init = *trajectory_generator.get_traj_file_x0_init(traj_select);
    Eigen::VectorXd x0_init_red = x0_init(n_x_indices);
    return x0_init_red;
}

Eigen::VectorXd CrocoddylController::get_act_traj_x0_red_init()
{
    Eigen::VectorXd x0_init = *trajectory_generator.get_traj_x0_init();
    Eigen::VectorXd x0_init_red = x0_init(n_x_indices);
    return x0_init_red;
}

void CrocoddylController::error_check(Eigen::VectorXd &tau_full)
{
    // Check for NaN values in the torque vector
    if (tau_full.hasNaN())
    {
        reset();
        error_flag = ErrorFlag::NAN_DETECTED;
        std::cerr << "NaN values detected in the torque vector!" << std::endl;
    }

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