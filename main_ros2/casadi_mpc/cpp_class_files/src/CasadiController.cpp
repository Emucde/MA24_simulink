#include "CasadiController.hpp"
#include "eigen_templates.hpp"

CasadiController::CasadiController(const std::string &urdf_filename,
                                   const std::string &mpc_config_filename,
                                   const std::string &general_config_filename)
    : CommonBaseController(urdf_filename, mpc_config_filename, general_config_filename),
      mpc_config_filename(mpc_config_filename),
      param_mpc_weight(read_config(mpc_config_filename)),
      standard_solver(nullptr, robot_config), planner_solver(nullptr, param_mpc_weight, robot_config)
{
    nlohmann::json general_config = read_config(general_config_filename);
    use_planner = get_config_value<bool>(general_config, "use_casadi_planner");
    CasadiMPCType default_active_mpc = string_to_casadi_mpctype(get_config_value<std::string>(general_config, "default_casadi_mpc"));
    casadi_uint default_traj_select = get_config_value<casadi_uint>(general_config, "trajectory_selection");

    // Initialize MPC objects
    for (int i = 0; i < static_cast<int>(CasadiMPCType::COUNT); ++i)
    {

        CasadiMPCType mpc = static_cast<CasadiMPCType>(i);
        if (mpc != CasadiMPCType::INVALID) // Ensures we are within valid enum range
        {
            casadi_mpcs.push_back(CasadiMPC(mpc, robot_config, trajectory_generator)); // Initialize all MPCs
        }
    }

    update_mpc_weights();

    // 1. use force planner. If false it could be that the active mpc is planner only.
    // This case is handled in setActiveMPC method

    setActiveMPC(default_active_mpc); // Set the default MPC
    switch_traj(default_traj_select); // Set the default trajectory

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
    Eigen::VectorXd x_k_ndof = Eigen::Map<const Eigen::VectorXd>(x_k_ndof_ptr, nx);
    double x_k[nx_red];

    // Convert nx to nx_red state
    for (casadi_uint i = 0; i < nx_red; i++)
    {
        x_k[i] = x_k_ndof[n_x_indices[i]];
    }

    // Solve the MPC
    int flag = solver->solveMPC(x_k);
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

    error_check(tau_full);

    return tau_full;
}

bool CasadiController::PlannerSolver::solveMPC(const casadi_real *const x_k_in)
{
    // in case of planner only it uses active_mpc->solve_planner(), otherwise it uses active_mpc->solve()
    // for pseudo planner.
    bool flag = solver->solveMPC(x_k_in);
    Eigen::Map<Eigen::VectorXd> q_d(x_d_ptr, nq_red);
    Eigen::Map<Eigen::VectorXd> q_p_d(x_d_ptr + nq_red, nq_red);
    Eigen::Map<Eigen::VectorXd> q_pp_d(q_pp_d_ptr, nq_red);

    Eigen::Map<const Eigen::VectorXd> q(x_k_in, nq_red);
    Eigen::Map<const Eigen::VectorXd> q_p(x_k_in + nq_red, nq_red);

    // use PD jointspace Controller
    // robot_model.updateState(Eigen::Map<const Eigen::VectorXd>(x_k_in, nx_red)); // TODO: koennte man auch vom torque mapper holen
    // u_opt = robot_model.dynamicsData.M * (q_pp_d - K_D_q * (q_p - q_p_d) - K_P_q * (q - q_d)) +
    //                       robot_model.dynamicsData.C_rnea + robot_model.dynamicsData.g;
    u_opt = q_pp_d - K_D_q.cwiseProduct(q_p - q_p_d) - K_P_q.cwiseProduct(q - q_d);

    return flag;
}

nlohmann::json CasadiController::read_config(std::string file_path)
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

void CasadiController::update_mpc_weights()
{
    param_mpc_weight = read_config(mpc_config_filename);

    for (int i = 0; i < static_cast<int>(CasadiMPCType::COUNT); ++i)
    {
        if (static_cast<CasadiMPCType>(i) != CasadiMPCType::INVALID)
        {
            std::string mpc_name = casadi_mpcs[i].get_mpc_name();
            casadi_mpcs[i].update_mpc_weights(param_mpc_weight[mpc_name]);
        }
    }
}

void CasadiController::update_config()
{
    update_mpc_weights();
    nlohmann::json general_config = read_config(general_config_filename);
    use_planner = get_config_value<bool>(general_config, "use_casadi_planner");
}

void CasadiController::switch_traj(uint traj_select)
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
            casadi_mpcs[i].switch_traj(x_k);
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

        w_ptr = active_mpc->get_w();
        traj_data_per_horizon = active_mpc->get_traj_data_per_horizon_len();
        mpc_traj_indices = const_cast<casadi_uint *>(active_mpc->get_mpc_traj_indices());
        dt = active_mpc->get_dt();

        // 2. Planner has a active mpc now
        planner_solver.switch_controller(active_mpc);
        standard_solver.switch_controller(active_mpc);

        //update mpc weights (maybe not necessary)
        active_mpc->update_mpc_weights(param_mpc_weight[active_mpc->get_mpc_name()]);

        // 3. check if active mpc is a planner only. In this case ignore force planner flag.
        set_planner_mode(use_planner); // Set the planner mode
        u_k_ptr = solver->get_optimal_control();
    }
    else
    {
        std::cerr << "Invalid MPC type." << std::endl;
    }
}

void CasadiController::set_planner_mode(bool use_planner_new)
{
    if (active_mpc->is_planner_mpc)
    {
        use_planner = true; // overwrite setting because it makes no sense to use a planner mpc as non-planner
        // but each non-planner mpc can be used as a planner too.
    }
    else
    {
        use_planner = use_planner_new;
    }

    mpc_config_t * mpc_config = active_mpc->get_mpc_config();

    if (use_planner)
    {
        planner_solver.update_planner_params();
        solver = &planner_solver;
        // planner mpc is kinematic only in this case the dynamic mpc
        // deliver their q_d and q_p_d values to the controller
        // Otherwise the toqrue mapper would be confused.
        if(mpc_config->kinematic_mpc == 0)
        {
            active_mpc->is_kinematic_mpc = true;
        }
    }
    else
    {
        solver = &standard_solver;
        // dynamic mpc gets back dynamic if it was previously kinematic due to planner
        if(mpc_config->kinematic_mpc == 0)
        {
            active_mpc->is_kinematic_mpc = false;
        }
    }
}

void CasadiController::PlannerSolver::switch_controller(CasadiMPC *new_mpc)
{
    active_mpc = new_mpc;
    planner_only_solver.active_mpc = new_mpc;
    planner_and_controller_solver.active_mpc = new_mpc;
    update_planner_params();
}

void CasadiController::PlannerSolver::update_planner_params()
{
    double* u_k_ptr = active_mpc->get_optimal_control();
    planner_mpc = active_mpc->is_planner_mpc;
    if (planner_mpc) // then u_k_ptr shows to [x_d_0, q_pp_d_0]
    {
        x_d_ptr = u_k_ptr;
        q_pp_d_ptr = u_k_ptr + nx_red;
        solver = &planner_only_solver;
    }
    else // then u_k_ptr shows to q_pp_0
    {
        x_d_ptr = u_k_ptr + nq_red;    // x_d_1
        q_pp_d_ptr = x_d_ptr + nx_red; // q_pp_d_1
        solver = &planner_and_controller_solver;
    }

    std::string mpc_name = active_mpc->get_mpc_name();
    K_P_q = Eigen::VectorXd::Map(param_mpc_weight[mpc_name]["K_P_q"].get<std::vector<double>>().data(), nq_red);
    K_D_q = Eigen::VectorXd::Map(param_mpc_weight[mpc_name]["K_D_q"].get<std::vector<double>>().data(), nq_red);
}

// Hat fuer R_q ueberhaupt nicht geklappt maybe fuer xprev
void CasadiController::collinearity_weight_x(const casadi_real *const x_k)
{
    robot_model.updateState(Eigen::Map<const Eigen::VectorXd>(x_k, nx_red));
    Eigen::MatrixXd J = robot_model.kinematicsData.J;
    Eigen::VectorXd J_scale = J.colwise().norm().cwiseInverse();                 // J_scale = J / colsum(J)
    Eigen::MatrixXd J_tilde = J.array().rowwise() * J_scale.transpose().array(); // J_tilde = J_scale * J_scale'
    Eigen::MatrixXd JJ_colin = J_tilde.transpose() * J_tilde;                    // JJ_colin = J_tilde' * J_tilde
    Eigen::VectorXd R_q_p = JJ_colin.cwiseAbs().colwise().sum().cwiseInverse();

    // convert param_mpc_weight["R_q_p] into a eigen Vector
    std::string mpc_name = active_mpc->get_mpc_name();
    Eigen::VectorXd R_q_p_weight = Eigen::VectorXd::Map(param_mpc_weight[mpc_name]["R_q_p"].get<std::vector<double>>().data(), nq_red);
    Eigen::VectorXd R_q_fin = R_q_p_weight.cwiseProduct(R_q_p);
    active_mpc->set_param("R_q_p", R_q_fin.data());
}

void CasadiController::reset(const double *const x_k_in)
{
    active_mpc->reset(x_k_in);
    tau_full_prev = Eigen::VectorXd::Zero(nq);
    reset_error_flag();
}

void CasadiController::reset()
{
    Eigen::VectorXd x0_init = get_act_traj_x0_red_init();
    active_mpc->reset(x0_init.data());
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