#include "CrocoddylMPC.hpp"

CrocoddylMPC::CrocoddylMPC(CrocoddylMPCType mpc_type,
                 RobotModel &robot_model,
                 const std::string &crocoddyl_config_path,
                 TrajectoryGenerator &trajectory_generator)
        : mpc_type(mpc_type),
          robot_model(robot_model),
          crocoddyl_config_path(crocoddyl_config_path),
          trajectory_generator(trajectory_generator),
          nq(robot_model.nq), nx(robot_model.nx), // this is nq_red and nx_red!!!
          dt(robot_model.robot_config.dt),
          traj_data(trajectory_generator.get_traj_data()),
          traj_data_real_len(trajectory_generator.get_traj_data_real_len()),
          traj_rows(trajectory_generator.get_traj_data()->rows()), 
          traj_cols(trajectory_generator.get_traj_data()->cols()),
          traj_count(0)
    {
        // TODO: Own for each MPC
        init_config();
        create_mpc_solver();
    }

void CrocoddylMPC::init_config()
{
    std::ifstream file(crocoddyl_config_path);
    if (!file.is_open())
    {
        std::cerr << "Error: Could not open JSON file." << std::endl;
        return;
    }

    nlohmann::json jsonData;
    file >> jsonData; // Parse JSON file
    file.close();
    mpc_settings = jsonData["mpc_settings"];
    param_mpc_weight = jsonData["param_mpc_weight"];

    N_MPC = get_setting<uint>("N_MPC");
    N_step = static_cast<uint>( get_setting<double>("Ts_MPC") / get_setting<double>("Ts") );
    N_solver_steps = get_setting<uint>("solver_steps");
    mpc_traj_indices = Eigen::VectorXi::LinSpaced(N_MPC+1, 0, N_MPC * N_step);
    is_kinematic = get_setting<bool>("is_kinematic");

    robot_config_t &robot_config = robot_model.robot_config;

    Eigen::VectorXi n_indices = ConstIntVectorMap(robot_config.n_indices, robot_config.nq_red); // robot_config.nq = 7

    // is also in model available
    Eigen::VectorXd q_min = Eigen::Map<const Eigen::VectorXd>(robot_config.q_limit_lower, robot_config.nq); 
    Eigen::VectorXd q_p_min = Eigen::Map<const Eigen::VectorXd>(robot_config.q_p_limit_lower, robot_config.nq);

    Eigen::VectorXd q_max = Eigen::Map<const Eigen::VectorXd>(robot_config.q_limit_upper, robot_config.nq);
    Eigen::VectorXd q_p_max = Eigen::Map<const Eigen::VectorXd>(robot_config.q_p_limit_upper, robot_config.nq);

    x_min = Eigen::VectorXd::Zero(nx); // robot_config.nx = 14, nx here is 12
    x_max = Eigen::VectorXd::Zero(nx); // robot_config.nx = 14, nx here is 12

    x_mean = Eigen::VectorXd::Zero(nx); // nx here is 12
    x_mean << (q_min(n_indices) + q_max(n_indices)) / 2, (q_p_min(n_indices) + q_p_max(n_indices)) / 2;

    x_min << q_min(n_indices), q_p_min(n_indices);
    x_max << q_max(n_indices), q_p_max(n_indices);

    Eigen::VectorXd u_min_ndof = Eigen::Map<const Eigen::VectorXd>(robot_config.torque_limit_lower, robot_config.nq);
    Eigen::VectorXd u_max_ndof = Eigen::Map<const Eigen::VectorXd>(robot_config.torque_limit_upper, robot_config.nq);

    u_min = Eigen::VectorXd::Zero(nq); // robot_config.nx = 14, nx here is 12
    u_min = Eigen::VectorXd::Zero(nq); // robot_config.nx = 14, nx here is 12

    u_min = u_min_ndof(n_indices);
    u_max = u_max_ndof(n_indices);

    // init default init guess
    xs_init_guess.resize(N_MPC+1);
    us_init_guess.resize(N_MPC);
}

std::shared_ptr<BaseCrocoddylIntegrator> CrocoddylMPC::create_integrator(const std::string &int_type)
{
    if (int_type == "euler")
    {
        return std::make_shared<IntegratorEuler>(int_type);
    }
    else if (int_type == "RK2")
    {
        return std::make_shared<IntegratorRK2>(int_type);
    }
    else if (int_type == "RK3")
    {
        return std::make_shared<IntegratorRK3>(int_type);
    }
    else if (int_type == "RK4")
    {
        return std::make_shared<IntegratorRK4>(int_type);
    }
    else
    {
        throw std::invalid_argument("Integration method not supported.");
    }
}

bool CrocoddylMPC::solve(const Eigen::VectorXd &x_k)
{
    set_references(x_k);
    // Provide implementation or leave it empty if not needed
    bool hasConverged = ddp->solve(xs_init_guess, us_init_guess, N_solver_steps, false, 1e-5);

    us_init_guess = ddp->get_us();
    xs_init_guess = ddp->get_xs();

    return hasConverged;
}

void CrocoddylMPC::set_coldstart_init_guess(const Eigen::VectorXd &x_k)
{
    // Resize vectors to the number of rows in the respective matrices
    xs_init_guess.resize(N_MPC+1);
    us_init_guess.resize(N_MPC);

    // Populate xs_init_guess
    for (uint i = 0; i < N_MPC+1; ++i) {
        xs_init_guess[i] = x_k.transpose();
    }

    robot_model.updateState(x_k);

    // Populate us_init_guess
    for (uint i = 0; i < N_MPC; ++i) {
        us_init_guess[i] = robot_model.dynamicsData.g.transpose();
    }
}


void CrocoddylMPC::switch_traj(const Eigen::VectorXd &x_k)
{
    traj_data = trajectory_generator.get_traj_data();
    traj_rows = traj_data->rows();
    traj_cols = traj_data->cols();

    traj_data_real_len = trajectory_generator.get_traj_data_real_len();

    if (traj_data_real_len + mpc_traj_indices[N_MPC] > traj_cols)
    {
        throw std::runtime_error("void CasadiMPC::switch_traj(const Eigen::MatrixXd* traj_data_new, \
                                  const double *const x_k_ptr, casadi_uint traj_data_real_len_new): \
                                  Trajectory data length + mpc_traj_indices[end] = " +
                                 std::to_string(traj_data_real_len + mpc_traj_indices[N_MPC]) +
                                 " exceeds totalthrow length = " +
                                 std::to_string(traj_cols) +
                                 ". Please generate new.");
    }

    generate_trajectory_blocks();
    set_coldstart_init_guess(x_k);
    set_references(x_k);

    traj_count = 0;
}

void CrocoddylMPC::generate_trajectory_blocks()
{
    Eigen::MatrixXd p_d(3, N_MPC+1);
    Eigen::MatrixXd q_d(4, N_MPC+1);
    std::vector<Eigen::Matrix3d> R_d;

    R_d.resize(N_MPC+1);

    p_d_blocks.resize(traj_cols);
    R_d_blocks.resize(traj_cols);

    for( uint32_t i = 0; i < traj_cols; i++)
    {
        p_d << (*traj_data)(p_d_rows, mpc_traj_indices.array() + i);
        q_d << (*traj_data)(q_d_rows, mpc_traj_indices.array() + i);
        for (int j = 0; j < q_d.cols(); ++j) {
            R_d[j] = quat2rotm<double>(q_d.col(j));
        }
        p_d_blocks[i] = p_d;
        R_d_blocks[i] = R_d;
    }
}

void CrocoddylMPC::create_mpc_solver()
{
    // Build the MPC problem
    std::string int_method = get_setting<std::string>("int_method");
    uint N_MPC = get_setting<uint>("N_MPC");
    std::shared_ptr<BaseCrocoddylIntegrator> integrator = create_integrator(int_method);

    // get robot model

    Eigen::VectorXd p_d;
    Eigen::Quaterniond q_d;
    Eigen::MatrixXd R_d;

    cost_models.resize(N_MPC+1);
    residual_state_models.resize(N_MPC+1);
    residual_joint_acceleration_models.resize(N_MPC+1);
    residual_control_models.resize(N_MPC+1);
    residual_frame_translation_models.resize(N_MPC+1);
    residual_frame_rotation_models.resize(N_MPC+1);

    auto model =
        boost::make_shared<crocoddyl::StateMultibody::PinocchioModel>(robot_model.robot_model);

    auto state = boost::make_shared<crocoddyl::StateMultibody>(model);

    // Create the running cost model list
    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> runningCostModels;
    boost::shared_ptr<crocoddyl::ActionModelAbstract> terminalCostModel;

    double scalar_weight = 0;

    for (uint i = 0; i < N_MPC+1; i++)
    {
        p_d = trajectory_generator.p_d.col(i);
        q_d = vec2quat<double>(trajectory_generator.q_d.col(i));
        R_d = q_d.toRotationMatrix();

        auto actuationModel = boost::make_shared<crocoddyl::ActuationModelFull>(state);

        crocoddyl::ActivationBounds state_bounds(x_min, x_max);
        crocoddyl::ActivationBounds ctrl_bounds(u_min, u_max);

        auto activation_state_bound = boost::make_shared<crocoddyl::ActivationModelWeightedQuadraticBarrier>(
            state_bounds, get_param_vec("R_x_bounds"));
        auto activation_ctrl_bound = boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(
            ctrl_bounds);
        auto activation_q_yt = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
            get_param_vec("Q_yt"));
        auto activation_q_yt_terminate = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
            get_param_vec("Q_yt_terminal"));
        auto activation_q_yr = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
            get_param_vec("Q_yr"));
        auto activation_q_yr_terminate = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
            get_param_vec("Q_yr_terminal"));
        auto activation_q_qp = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
            get_param_vec("R_x"));
        auto activation_q_qpp = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
            get_param_vec("R_q_pp"));
        auto activation_q_xprev = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
            get_param_vec("R_xprev"));

        auto residual_xreg_bound = boost::make_shared<crocoddyl::ResidualModelState>(
        state, x_mean); // r = x_mean - x
        residual_state_models[i]["stateRegBound"] = residual_xreg_bound;
        auto residual_ureg_bound = boost::make_shared<crocoddyl::ResidualModelControl>(
            state, Eigen::VectorXd::Zero(nq)); // r = x_mean - x
        residual_control_models[i]["ctrlRegBound"] = residual_ureg_bound;
        auto residual_q_yt = boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
            state, robot_model.tcp_frame_id, p_d); // r = p_d - p
        residual_frame_translation_models[i]["TCP_pose"] = residual_q_yt;
        auto residual_q_yr = boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
            state, robot_model.tcp_frame_id, R_d); // r = R_d^T * R
        residual_frame_rotation_models[i]["TCP_rot"] = residual_q_yr;
        auto residual_q_qp = boost::make_shared<crocoddyl::ResidualModelState>(
            state, Eigen::VectorXd::Zero(nx)); // r = q_p
        residual_state_models[i]["q_pReg"] = residual_q_qp;
        auto residual_q_qpp = boost::make_shared<crocoddyl::ResidualModelJointAcceleration>(
            state, Eigen::VectorXd::Zero(nq)); // r = q_pp
        residual_joint_acceleration_models[i]["q_ppReg"] = residual_q_qpp;
        auto residual_xprev = boost::make_shared<crocoddyl::ResidualModelState>(
            state, Eigen::VectorXd::Zero(nx)); // r = x_prev - x_prev_ref (is later set)
        residual_state_models[i]["xprevReg"] = residual_xprev;
        auto residual_u = boost::make_shared<crocoddyl::ResidualModelControl>(
            state, Eigen::VectorXd::Zero(nq)); // r = u
        residual_control_models[i]["ctrlReg"] = residual_u;
        auto residual_u_prev = boost::make_shared<crocoddyl::ResidualModelControl>(
            state, Eigen::VectorXd::Zero(nq)); // r = u - u_prev (is later set)
        residual_control_models[i]["ctrlPrev"] = residual_u_prev;
        auto xRegBoundCost = boost::make_shared<crocoddyl::CostModelResidual>(
            state, activation_state_bound, residual_xreg_bound);
        auto uRegBoundCost = boost::make_shared<crocoddyl::CostModelResidual>(
            state, activation_ctrl_bound, residual_ureg_bound);
        auto q_qpCost = boost::make_shared<crocoddyl::CostModelResidual>(
            state, activation_q_qp, residual_q_qp);
        auto q_qppCost = boost::make_shared<crocoddyl::CostModelResidual>(
            state, activation_q_qpp, residual_q_qpp);
        auto xprevCost = boost::make_shared<crocoddyl::CostModelResidual>(
            state, activation_q_xprev, residual_xprev);
        auto uRegCost = boost::make_shared<crocoddyl::CostModelResidual>(
            state, residual_u); // ActivationModelQuadTpl as default activation model
        auto uRegPrevCost = boost::make_shared<crocoddyl::CostModelResidual>(
            state, residual_u_prev); // ActivationModelQuadTpl as default activation model

        if (i < N_MPC)
        {
            auto q_ytCost = boost::make_shared<crocoddyl::CostModelResidual>(
                state, activation_q_yt, residual_q_yt);
            auto q_yrCost = boost::make_shared<crocoddyl::CostModelResidual>(
                state, activation_q_yr, residual_q_yr);

            auto runningDifferentialCostModel = boost::make_shared<crocoddyl::CostModelSum>(state);

            scalar_weight = get_param<double>("q_x_bound_cost");
            if (scalar_weight > 0)
            {
                runningDifferentialCostModel->addCost("stateRegBound", xRegBoundCost, scalar_weight);
                cost_models[i]["stateRegBound"] = runningDifferentialCostModel->get_costs().at("stateRegBound");
            }
            scalar_weight = get_param<double>("q_u_bound_cost");
            if (scalar_weight > 0)
            {
                runningDifferentialCostModel->addCost("ctrlRegBound", uRegBoundCost, scalar_weight);
                cost_models[i]["ctrlRegBound"] = runningDifferentialCostModel->get_costs().at("ctrlRegBound");
            }
            scalar_weight = get_param<double>("q_yt_common_weight");
            if (scalar_weight > 0)
            {
                runningDifferentialCostModel->addCost("TCP_pose", q_ytCost, scalar_weight);
                cost_models[i]["TCP_pose"] = runningDifferentialCostModel->get_costs().at("TCP_pose");
            }
            scalar_weight = get_param<double>("q_yr_common_weight");
            if (scalar_weight > 0)
            {
                runningDifferentialCostModel->addCost("TCP_rot", q_yrCost, scalar_weight);
                cost_models[i]["TCP_rot"] = runningDifferentialCostModel->get_costs().at("TCP_rot");
            }
            scalar_weight = get_param<double>("q_p_common_weight");
            if (scalar_weight > 0)
            {
                runningDifferentialCostModel->addCost("q_pReg", q_qpCost, scalar_weight);
                cost_models[i]["q_pReg"] = runningDifferentialCostModel->get_costs().at("q_pReg");
            }
            scalar_weight = get_param<double>("q_pp_common_weight");
            if (scalar_weight > 0)
            {
                runningDifferentialCostModel->addCost("q_ppReg", q_qppCost, scalar_weight);
                cost_models[i]["q_ppReg"] = runningDifferentialCostModel->get_costs().at("q_ppReg");
            }
            scalar_weight = get_param<double>("q_xprev_common_weight");
            if (scalar_weight > 0)
            {
                runningDifferentialCostModel->addCost("xprevReg", xprevCost, scalar_weight);
                cost_models[i]["xprevReg"] = runningDifferentialCostModel->get_costs().at("xprevReg");
            }
            scalar_weight = get_param<double>("q_ureg_cost");
            if (scalar_weight > 0)
            {
                runningDifferentialCostModel->addCost("ctrlReg", uRegCost, scalar_weight);
                cost_models[i]["ctrlReg"] = runningDifferentialCostModel->get_costs().at("ctrlReg");
            }
            scalar_weight = get_param<double>("q_uprev_cost");
            if (scalar_weight > 0)
            {
                runningDifferentialCostModel->addCost("ctrlPrev", uRegPrevCost, scalar_weight);
                cost_models[i]["ctrlPrev"] = runningDifferentialCostModel->get_costs().at("ctrlPrev");
            }

            auto DAM = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
            state, actuationModel, runningDifferentialCostModel);

            // integrate the differential cost models
            auto IAM = integrator->integrate(DAM, dt);
            runningCostModels.push_back(
                boost::static_pointer_cast<crocoddyl::ActionModelAbstract>(IAM));
        }
        else
        {
            auto q_yt_terminateCost = boost::make_shared<crocoddyl::CostModelResidual>(
            state, activation_q_yt_terminate, residual_q_yt);
            auto q_yr_terminateCost = boost::make_shared<crocoddyl::CostModelResidual>(
                state, activation_q_yr_terminate, residual_q_yr);

            auto terminalDifferentialCostModel = boost::make_shared<crocoddyl::CostModelSum>(state);

            scalar_weight = get_param<double>("q_x_bound_cost");
            if (scalar_weight > 0)
            {
                terminalDifferentialCostModel->addCost("stateRegBound", xRegBoundCost, scalar_weight);
                cost_models[i]["stateRegBound"] = terminalDifferentialCostModel->get_costs().at("stateRegBound");
            }
            scalar_weight = get_param<double>("q_u_bound_cost");
            if (scalar_weight > 0)
            {
                terminalDifferentialCostModel->addCost("ctrlRegBound", uRegBoundCost, scalar_weight);
                cost_models[i]["ctrlRegBound"] = terminalDifferentialCostModel->get_costs().at("ctrlRegBound");
            }
            scalar_weight = get_param<double>("q_yt_terminal_common_weight");
            if (scalar_weight > 0)
            {
                terminalDifferentialCostModel->addCost("TCP_pose", q_yt_terminateCost, scalar_weight);
                cost_models[i]["TCP_pose"] = terminalDifferentialCostModel->get_costs().at("TCP_pose");
            }
            scalar_weight = get_param<double>("q_yr_terminal_common_weight");
            if (scalar_weight > 0)
            {
                terminalDifferentialCostModel->addCost("TCP_rot", q_yr_terminateCost, scalar_weight);
                cost_models[i]["TCP_rot"] = terminalDifferentialCostModel->get_costs().at("TCP_rot");
            }
            scalar_weight = get_param<double>("q_p_common_weight");
            if (scalar_weight > 0)
            {
                terminalDifferentialCostModel->addCost("q_pReg", q_qpCost, scalar_weight);
                cost_models[i]["q_pReg"] = terminalDifferentialCostModel->get_costs().at("q_pReg");
            }
            scalar_weight = get_param<double>("q_pp_common_weight");
            if (scalar_weight > 0)
            {
                terminalDifferentialCostModel->addCost("q_ppReg", q_qppCost, scalar_weight);
                cost_models[i]["q_ppReg"] = terminalDifferentialCostModel->get_costs().at("q_ppReg");
            }
            scalar_weight = get_param<double>("q_xprev_common_weight");
            if (scalar_weight > 0)
            {
                terminalDifferentialCostModel->addCost("xprevReg", xprevCost, scalar_weight);
                cost_models[i]["xprevReg"] = terminalDifferentialCostModel->get_costs().at("xprevReg");
            }
            scalar_weight = get_param<double>("q_ureg_cost"); // hmm maybe ignored
            if (scalar_weight > 0)
            {
                terminalDifferentialCostModel->addCost("ctrlReg", uRegCost, scalar_weight);
                cost_models[i]["ctrlReg"] = terminalDifferentialCostModel->get_costs().at("ctrlReg");
            }
            scalar_weight = get_param<double>("q_uprev_cost"); // hmm maybe ignored
            if (scalar_weight > 0)
            {
                terminalDifferentialCostModel->addCost("ctrlPrev", uRegPrevCost, scalar_weight);
                cost_models[i]["ctrlPrev"] = terminalDifferentialCostModel->get_costs().at("ctrlPrev");
            }

            // Integrate the terminal cost model
            auto terminalDAM = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
                state, actuationModel, terminalDifferentialCostModel);

            auto terminalIAM = integrator->integrate(terminalDAM, dt);

            terminalCostModel = boost::static_pointer_cast<crocoddyl::ActionModelAbstract>(terminalIAM);
        }
    }

    auto problem = boost::make_shared<crocoddyl::ShootingProblem>(Eigen::VectorXd::Zero(nx), runningCostModels, terminalCostModel);

    problem_reference = problem;

    ddp = boost::static_pointer_cast<crocoddyl::SolverAbstract>(
        boost::make_shared<crocoddyl::SolverBoxDDP>(problem));
}

void CrocoddylMPC::set_references(const Eigen::VectorXd &x_k)
{   
    for (uint i = 0; i < N_MPC+1; i++)
    {
        residual_frame_translation_models[i]["TCP_pose"]->set_reference(p_d_blocks[traj_count].col(i));
        residual_frame_rotation_models[i]["TCP_rot"]->set_reference(R_d_blocks[traj_count][i]);
        residual_state_models[i]["xprevReg"]->set_reference(xs_init_guess[i]); // init guess
        if(i < N_MPC)
            residual_control_models[i]["ctrlPrev"]->set_reference(us_init_guess[i]); // init guess
    }
    problem_reference->set_x0(x_k);
    traj_count++;
}