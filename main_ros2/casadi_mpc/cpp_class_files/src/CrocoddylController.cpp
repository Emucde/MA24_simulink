#include "CrocoddylController.hpp"

CrocoddylController::CrocoddylController(const std::string &urdf_filename,
                                         const std::string &mpc_config_filename,
                                         const std::string &general_config_filename)
    : CommonBaseController(urdf_filename, mpc_config_filename, general_config_filename)
{
    // Initialize MPC objects
    for (int i = 0; i < static_cast<int>(CrocoddylMPCType::COUNT); ++i)
    {
        CrocoddylMPCType mpc = static_cast<CrocoddylMPCType>(i);
        if (mpc != CrocoddylMPCType::INVALID)
        {                                                                                                          // Ensures we are within valid enum range
            crocoddyl_mpcs.push_back(CrocoddylMPC(mpc, robot_model, mpc_config_filename, trajectory_generator)); // Initialize all MPCs
        }
    }

    nlohmann::json general_config = read_config(general_config_filename);

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

Eigen::VectorXd CrocoddylController::update_control(const Eigen::VectorXd &x_nq)
{
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);
    
    double x_k[nx_red];

    // Convert nx to nx_red state
    for (casadi_uint i = 0; i < nx_red; i++)
    {
        x_k[i] = x_nq[n_x_indices[i]];
    }
    
    bool hasConverged = active_mpc->solve(Eigen::Map<Eigen::VectorXd>(x_k, nx_red));
    if (!hasConverged)
    {
        std::cerr << "Error in Crocoddyl function call." << std::endl;
        error_flag = ErrorFlag::CROCODDYL_ERROR;
        tau_full_prev = tau_full; // zero torque
        reset(x_k);
        return tau_full;          // Return zero torque
    }

    u_k_ptr = active_mpc->get_optimal_control();
    Eigen::Map<Eigen::VectorXd> u_k(u_k_ptr, nq_red);

    tau_full = torque_mapper.calc_full_torque(u_k, x_nq);

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

void CrocoddylController::update_config()
{
    nlohmann::json general_config = read_config(general_config_filename);
    update_mpc_weights();
    
    active_mpc->init_config();
    dt = active_mpc->get_dt();
    
    torque_mapper.update_config(dt);
    torque_mapper.set_kinematic_mpc_flag(active_mpc->is_kinematic);
}

void CrocoddylController::setActiveMPC(CrocoddylMPCType mpc_type)
{
    if (mpc_type != CrocoddylMPCType::INVALID)
    {
        selected_mpc_type = mpc_type;
        active_mpc = &crocoddyl_mpcs[static_cast<int>(selected_mpc_type)];
        update_config();
        u_k_ptr = active_mpc->get_optimal_control();
    }
    else
    {
        std::cerr << "Invalid MPC type." << std::endl;
    }
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