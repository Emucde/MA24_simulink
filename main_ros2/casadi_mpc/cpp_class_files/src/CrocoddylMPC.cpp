#include "CrocoddylMPC.hpp"

void BaseCrocoddylMPC::read_config_file()
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
}

std::shared_ptr<BaseCrocoddylIntegrator> BaseCrocoddylMPC::create_integrator(const std::string &int_type)
{
    if (int_type == "Euler")
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

void BaseCrocoddylMPC::solve(const Eigen::VectorXd &x)
{
    // Provide implementation or leave it empty if not needed
}

void ClassicDynMPC::create_mpc_solver()
{
    // Build the MPC problem
    std::string int_method = json2value<std::string>("integration_method");
    uint N_MPC = json2value<uint>("N_MPC");
    std::shared_ptr<BaseCrocoddylIntegrator> integrator = create_integrator(int_method);

    robot_config_t &robot_config = robot_model.robot_config;

    // is also in model available
    Eigen::VectorXd q_min = Eigen::Map<const Eigen::VectorXd>(robot_config.q_limit_lower, robot_config.nq);
    Eigen::VectorXd q_p_min = Eigen::Map<const Eigen::VectorXd>(robot_config.q_p_limit_lower, robot_config.nq);

    Eigen::VectorXd q_max = Eigen::Map<const Eigen::VectorXd>(robot_config.q_limit_upper, robot_config.nq);
    Eigen::VectorXd q_p_max = Eigen::Map<const Eigen::VectorXd>(robot_config.q_p_limit_upper, robot_config.nq);

    Eigen::VectorXd x_min = Eigen::VectorXd::Zero(robot_config.nx);
    Eigen::VectorXd x_max = Eigen::VectorXd::Zero(robot_config.nx);

    Eigen::VectorXd x_mean = Eigen::VectorXd::Zero(robot_config.nx);
    x_mean << (q_min + q_max) / 2, (q_p_min + q_p_max) / 2;

    x_min << q_min, q_p_min;
    x_max << q_max, q_p_max;
    // get robot model

    Eigen::VectorXd p_d;
    Eigen::Quaterniond q_d;
    Eigen::MatrixXd R_d;

    auto model =
        boost::make_shared<crocoddyl::StateMultibody::PinocchioModel>(robot_model.robot_model);

    auto state = boost::make_shared<crocoddyl::StateMultibody>(model);

    // Create the running cost model list
    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> runningCostModels;
    auto terminalDifferentialCostModel = boost::make_shared<crocoddyl::CostModelSum>(state);
    auto actuationModel = boost::make_shared<crocoddyl::ActuationModelFull>(state);

    crocoddyl::ActivationBounds bounds(x_min, x_max);

    auto activation_bound = boost::make_shared<crocoddyl::ActivationModelWeightedQuadraticBarrier>(
        bounds, json2vec("R_x_bounds"));
    auto activation_q_yt = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
        json2vec("Q_yt"));
    auto activateion_q_yt_terminate = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
        json2vec("Q_yt_terminal"));
    auto activation_q_yr = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
        json2vec("Q_yr"));
    auto activation_q_yr_terminate = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
        json2vec("Q_yr_terminal"));
    auto activation_q_qp = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
        json2vec("R_q_p_weight_vec"));
    auto activation_q_qpp = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
        json2vec("R_q_pp_weight_vec"));
    auto activation_q_xprev = boost::make_shared<crocoddyl::ActivationModelWeightedQuad>(
        json2vec("R_xprev"));

    auto residual_xreg_bound = boost::make_shared<crocoddyl::ResidualModelState>(
        state, x_mean); // r = x_mean - x
    auto residual_q_yt = boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(
        state, robot_model.tcp_frame_id, p_d); // r = p_d - p
    auto residual_q_yr = boost::make_shared<crocoddyl::ResidualModelFrameRotation>(
        state, robot_model.tcp_frame_id, R_d); // r = R_d^T * R
    auto residual_q_qp = boost::make_shared<crocoddyl::ResidualModelState>(
        state, Eigen::VectorXd::Zero(robot_config.nq)); // r = q_p
    auto residual_q_qpp = boost::make_shared<crocoddyl::ResidualModelState>(
        state, Eigen::VectorXd::Zero(robot_config.nq)); // r = q_pp
    auto residual_xprev = boost::make_shared<crocoddyl::ResidualModelState>(
        state, Eigen::VectorXd::Zero(robot_config.nx)); // r = x_prev - x_prev_ref (is later set)
    auto residual_u = boost::make_shared<crocoddyl::ResidualModelControl>(
        state, Eigen::VectorXd::Zero(robot_config.nq)); // r = u
    auto residual_u_prev = boost::make_shared<crocoddyl::ResidualModelControl>(
        state, Eigen::VectorXd::Zero(robot_config.nq)); // r = u - u_prev (is later set)

    auto xRegBoundCost = boost::make_shared<crocoddyl::CostModelResidual>(
        state, activation_bound, residual_xreg_bound);
    auto q_ytCost = boost::make_shared<crocoddyl::CostModelResidual>(
        state, activation_q_yt, residual_q_yt);
    auto q_yt_terminateCost = boost::make_shared<crocoddyl::CostModelResidual>(
        state, activateion_q_yt_terminate, residual_q_yt);
    auto q_yrCost = boost::make_shared<crocoddyl::CostModelResidual>(
        state, activation_q_yr, residual_q_yr);
    auto q_yr_terminateCost = boost::make_shared<crocoddyl::CostModelResidual>(
        state, activation_q_yr_terminate, residual_q_yr);
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

    double scalar_weight = 0;

    for (uint i = 0; i < N_MPC; i++)
    {
        p_d = trajectory_generator.p_d.col(i);
        q_d = vec2quat<double>(trajectory_generator.q_d.col(i));
        R_d = q_d.toRotationMatrix();

        auto runningDifferentialCostModel = boost::make_shared<crocoddyl::CostModelSum>(state);

        if (i < N_MPC - 1)
        {
            scalar_weight = json2value<double>("q_x_bound_cost");
            if (scalar_weight > 0)
                runningDifferentialCostModel->addCost("stateRegBound", xRegBoundCost, scalar_weight);
            scalar_weight = json2value<double>("q_yt_cost");
            if (scalar_weight > 0)
                runningDifferentialCostModel->addCost("stateRegBound", q_ytCost, scalar_weight);
            scalar_weight = json2value<double>("q_yr_cost");
            if (scalar_weight > 0)
                runningDifferentialCostModel->addCost("stateRegBound", q_yrCost, scalar_weight);
            scalar_weight = json2value<double>("q_qp_cost");
            if (scalar_weight > 0)
                runningDifferentialCostModel->addCost("stateRegBound", q_qpCost, scalar_weight);
            scalar_weight = json2value<double>("q_qpp_cost");
            if (scalar_weight > 0)
                runningDifferentialCostModel->addCost("stateRegBound", q_qppCost, scalar_weight);
            scalar_weight = json2value<double>("q_xprev_cost");
            if (scalar_weight > 0)
                runningDifferentialCostModel->addCost("stateRegBound", xprevCost, scalar_weight);
            scalar_weight = json2value<double>("u_reg_cost");
            if (scalar_weight > 0)
                runningDifferentialCostModel->addCost("stateRegBound", uRegCost, scalar_weight);
            scalar_weight = json2value<double>("u_reg_prev_cost");
            if (scalar_weight > 0)
                runningDifferentialCostModel->addCost("stateRegBound", uRegPrevCost, scalar_weight);
        }
        else
        {
            scalar_weight = json2value<double>("q_x_bound_cost");
            if (scalar_weight > 0)
                terminalDifferentialCostModel->addCost("stateRegBound", xRegBoundCost, scalar_weight);
            scalar_weight = json2value<double>("q_yt_cost");
            if (scalar_weight > 0)
                terminalDifferentialCostModel->addCost("stateRegBound", q_yt_terminateCost, scalar_weight);
            scalar_weight = json2value<double>("q_yr_cost");
            if (scalar_weight > 0)
                terminalDifferentialCostModel->addCost("stateRegBound", q_yr_terminateCost, scalar_weight);
            scalar_weight = json2value<double>("q_qp_cost");
            if (scalar_weight > 0)
                terminalDifferentialCostModel->addCost("stateRegBound", q_qpCost, scalar_weight);
            scalar_weight = json2value<double>("q_qpp_cost");
            if (scalar_weight > 0)
                terminalDifferentialCostModel->addCost("stateRegBound", q_qppCost, scalar_weight);
            scalar_weight = json2value<double>("q_xprev_cost");
            if (scalar_weight > 0)
                terminalDifferentialCostModel->addCost("stateRegBound", xprevCost, scalar_weight);
            scalar_weight = json2value<double>("u_reg_cost"); // hmm maybe ignored
            if (scalar_weight > 0)
                terminalDifferentialCostModel->addCost("stateRegBound", uRegCost, scalar_weight);
            scalar_weight = json2value<double>("u_reg_prev_cost"); // hmm maybe ignored
            if (scalar_weight > 0)
                terminalDifferentialCostModel->addCost("stateRegBound", uRegPrevCost, scalar_weight);
        }

        auto DAM = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
            state, actuationModel, runningDifferentialCostModel);

        // integrate the differential cost models
        auto IAM = integrator->integrate(DAM, dt);
        runningCostModels.push_back(
            boost::static_pointer_cast<crocoddyl::ActionModelAbstract>(IAM));
    }

    // Integrate the terminal cost model
    auto terminalDAM = boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
        state, actuationModel, terminalDifferentialCostModel);

    auto terminalIAM = integrator->integrate(terminalDAM, dt);

    auto terminalCostModel = boost::static_pointer_cast<crocoddyl::ActionModelAbstract>(terminalIAM);

    auto problem = boost::make_shared<crocoddyl::ShootingProblem>(Eigen::VectorXd::Zero(robot_config.nq), runningCostModels, terminalCostModel);

    ddp = boost::static_pointer_cast<crocoddyl::SolverAbstract>(
        boost::make_shared<crocoddyl::SolverBoxDDP>(problem));
}

void ClassicDynMPC::set_references(casadi_real *x_k_in)
{
    // Provide implementation or leave it empty if not needed
}