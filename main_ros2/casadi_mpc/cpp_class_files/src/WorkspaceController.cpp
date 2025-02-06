#include "WorkspaceController.hpp"

Eigen::MatrixXd BaseController::computeJacobianRegularization()
{
    // Assume necessary member variables exist:
    // `J` (the reduced Jacobian matrix), `x_err` (the error vector),
    // `nq` (dimensions), `sing_method` (regularization method),
    // `ctrl_param` (contains the regularization parameters)

    Eigen::MatrixXd J = robot_model.kinematicsData.J;
    Eigen::VectorXd J_pinv = Eigen::MatrixXd::Zero(J.cols(), J.rows());

    switch (sing_method)
    {
    case SingularityRobustnessMode::None:
        J_pinv = (J.transpose() * J).inverse() * J.transpose();
        break;
    case SingularityRobustnessMode::Damping:
    {
        double k = regularization_settings.k;
        J_pinv = (J.transpose() * J + k * Eigen::MatrixXd::Identity(nq, nq)).inverse() * J.transpose();
    }
    break;
    case SingularityRobustnessMode::CompleteOrthogonalDecomposition:
        J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
        break;
    case SingularityRobustnessMode::JacobiSVD:
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        J_pinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
    }
    break;
    case SingularityRobustnessMode::ThresholdSmallSingularValues:
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd S = svd.singularValues().asDiagonal();

        double epsilon = regularization_settings.eps; // Threshold for small singular values

        S.diagonal() = S.diagonal().cwiseMax(epsilon);

        // Reconstruct the matrix with modified singular values
        Eigen::MatrixXd J_new = svd.matrixU() * S * svd.matrixV().transpose();
        J_pinv = J_new.completeOrthogonalDecomposition().pseudoInverse(); // Or use SVD again
        // J_pinv = (J_new.transpose() * J_new).inverse() * J_new.transpose();
    }
    break;
    case SingularityRobustnessMode::TikhonovRegularization:
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd S = svd.singularValues().asDiagonal();

        double epsilon = regularization_settings.eps; // Threshold for small singular values

        S.diagonal() = (S.diagonal().array() < epsilon).select((S.diagonal().array().square() + epsilon * epsilon).sqrt(), S.diagonal());

        // Reconstruct the matrix with modified singular values
        Eigen::MatrixXd J_new = svd.matrixU() * S * svd.matrixV().transpose();
        J_pinv = J_new.completeOrthogonalDecomposition().pseudoInverse(); // Or use SVD again
        // J_pinv = (J_new.transpose() * J_new).inverse() * J_new.transpose();
    }
    break;
    case SingularityRobustnessMode::SimpleCollinearity:
    {
        // calculate singular values of J
        Eigen::VectorXd J_scale = J.colwise().norm().cwiseInverse();                 // J_scale = J / colsum(J)
        Eigen::MatrixXd J_tilde = J.array().rowwise() * J_scale.transpose().array(); // J_tilde = J_scale * J_scale'
        Eigen::MatrixXd JJ_colin = J_tilde.transpose() * J_tilde;                    // JJ_colin = J_tilde' * J_tilde

        // Calculate Eigenvalues of JJ_colin
        Eigen::EigenSolver<Eigen::MatrixXd> solver(JJ_colin);
        Eigen::VectorXd eigenvalues = solver.eigenvalues().real();

        // Get the amount of all eigenvalues that are smaller than regularization_settings.eps_collinear
        int ew_count = (eigenvalues.array() < regularization_settings.eps_collinear).count();

        if (ew_count > 0)
        {
            // Calculate collinearity column norm (changed from row to column)
            Eigen::VectorXd R = JJ_colin.colwise().norm();

            // Create a sorted copy of R (ascending order)
            Eigen::VectorXd sortedR = R;
            std::sort(sortedR.data(), sortedR.data() + sortedR.size());

            // Find the threshold value
            double threshold = sortedR(sortedR.size() - ew_count);

            // Create a boolean mask for columns to keep
            Eigen::Array<bool, Eigen::Dynamic, 1> keepMask = R.array() >= threshold;

            // Create an index vector for columns to keep
            Eigen::VectorXi keepIndices = Eigen::VectorXi::LinSpaced(J.cols(), 0, J.cols() - 1);
            keepIndices = keepIndices(keepMask);

            // Use these indices to create a new matrix with the desired columns
            Eigen::MatrixXd Juced = J(Eigen::all, keepIndices);

            J_pinv = Juced.completeOrthogonalDecomposition().pseudoInverse();
        }
        else
        {
            J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
        }
    }
    break;
    case SingularityRobustnessMode::SteinboeckCollinearity:
    {
        Eigen::VectorXd J_scale = J.rowwise().norm().cwiseInverse();                 // J_scale = J / rowsum(J)
        Eigen::MatrixXd J_tilde = J.array().rowwise() * J_scale.transpose().array(); // J_tilde = J_scale * J_scale'
        Eigen::MatrixXd JJ_colin = J_tilde.transpose() * J_tilde;                    // JJ_colin = J_tilde' * J_tilde

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(JJ_colin, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd V = svd.matrixV().real();

        // Get the singular values
        Eigen::VectorXd singularValues = svd.singularValues();

        // Create a boolean vector for values above threshold
        Eigen::Array<bool, Eigen::Dynamic, 1> aboveThreshold = (singularValues.array() > regularization_settings.lambda_min);

        // Create an index vector for columns to keep
        Eigen::VectorXi indices = Eigen::VectorXi::LinSpaced(aboveThreshold.size(), 0, aboveThreshold.size() - 1);
        indices = indices(aboveThreshold);

        // Select columns using the index vector
        Eigen::MatrixXd V_new = V(Eigen::all, indices);

        Eigen::MatrixXd T_bar = J_scale.asDiagonal() * V_new;
        Eigen::MatrixXd J_new = J * T_bar;

        J_pinv = T_bar * (J_new.fullPivLu().inverse()); // More robust inverse calculation
    }
    break;
    case SingularityRobustnessMode::RegularizationBySingularValues:
    {
        // Handle small singular values according to this method
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd S = svd.singularValues().asDiagonal();

        for (int i = 0; i < S.diagonal().size(); ++i)
        {
            if (S(i, i) < regularization_settings.eps)
            {
                S(i, i) = (S(i, i) >= 0 ? 1 : -1) / (robot_model.robot_config.q_limit_upper[i]); // Fixed to the robot parameter
            }
        }

        // Reconstruct the matrix with modified singular values
        Eigen::MatrixXd J_new = svd.matrixU() * S * svd.matrixV().transpose();
        J_pinv = J_new.completeOrthogonalDecomposition().pseudoInverse(); // Or use SVD again
    }
    break;
    default:
        // Handle the case when the singularity robustness mode is not recognized
        throw std::invalid_argument("Singularity robustness mode not recognized");
    }

    return J_pinv;
}

void BaseController::calculateControlData(const Eigen::VectorXd &x)
{
    // Update the robot model state with joint positions and velocities
    robot_model.updateState(x);

    // Parameters
    q = robot_model.jointData.q;     // Time derivative of generalized coordinates
    q_p = robot_model.jointData.q_p; // Time derivative of generalized coordinates

    // Get matrices and variables from the robot model
    M = robot_model.dynamicsData.M; // Inertia matrix
    C = robot_model.dynamicsData.C; // Coriolis matrix
    g = robot_model.dynamicsData.g; // Gravitational forces

    J = robot_model.kinematicsData.J;     // Geometric Jacobian to end-effector
    J_p = robot_model.kinematicsData.J_p; // Time derivative of geometric Jacobian

    // Current pose of end-effector
    Eigen::Vector3d p = robot_model.kinematicsData.p;
    Eigen::VectorXd y_p = J * q_p;

    Eigen::Vector3d p_p = y_p.head(3);     // Linear velocity of the end-effector
    Eigen::Vector3d omega_e = y_p.tail(3); // Angular velocity of the end-effector

    // Desired trajectory
    Eigen::VectorXd p_d = trajectory_generator.p_d.col(traj_count);
    Eigen::VectorXd p_d_p = trajectory_generator.p_d_p.col(traj_count);
    Eigen::VectorXd p_d_pp = trajectory_generator.p_d_pp.col(traj_count);

    Eigen::VectorXd q_d = trajectory_generator.q_d.col(traj_count);
    Eigen::VectorXd omega_d = trajectory_generator.omega_d.col(traj_count);
    Eigen::VectorXd omega_d_p = trajectory_generator.omega_d_p.col(traj_count);

    // Errors
    Eigen::Quaterniond quat = robot_model.kinematicsData.quat;

    Eigen::VectorXd q_err_tmp = quat * q_d.conjugate(); // Quaternion error
    Eigen::VectorXd q_err = q_err_tmp.tail(3);          // Only the orientation error part

    x_err = Eigen::VectorXd::Zero(6);
    x_err << (p - p_d), q_err; // Error as quaternion

    x_err_p = Eigen::VectorXd::Zero(6);
    x_err_p << (p_p - p_d_p), (omega_e - omega_d);

    x_d_p = Eigen::VectorXd::Zero(6);
    x_d_p << p_d_p, omega_d;

    x_d_pp = Eigen::VectorXd::Zero(6);
    x_d_pp << p_d_pp, omega_d_p;

    Eigen::MatrixXd J_pinv = computeJacobianRegularization();
}

WorkspaceController::WorkspaceController(
    const std::string &urdf_path,
    const std::string &tcp_frame_name,
    bool use_gravity,
    ControllerSettings controller_settings) : urdf_path(urdf_path),
                                              tcp_frame_name(tcp_frame_name),
                                              robot_config(get_robot_config()),
                                              controller_settings(controller_settings),
                                              robot_model(urdf_path, tcp_frame_name, robot_config, use_gravity, true),
                                              torque_mapper(urdf_path, tcp_frame_name, robot_config, use_gravity, false),
                                              trajectory_generator(torque_mapper, robot_config.dt),
                                              sing_method(SingularityRobustnessMode::None),
                                              ct_controller(robot_model, sing_method, controller_settings, trajectory_generator),
                                              pd_plus_controller(robot_model, sing_method, controller_settings, trajectory_generator),
                                              inverse_dyn_controller(robot_model, sing_method, controller_settings, trajectory_generator),
                                              active_controller(&ct_controller),
                                              n_x_indices(ConstIntVectorMap(robot_config.n_x_indices, robot_config.nx_red))
{
}

WorkspaceController::WorkspaceController(
    const std::string &urdf_path,
    const std::string &tcp_frame_name,
    bool use_gravity) : urdf_path(urdf_path),
                        tcp_frame_name(tcp_frame_name),
                        robot_config(get_robot_config()),
                        robot_model(urdf_path, tcp_frame_name, robot_config, use_gravity, true),
                        torque_mapper(urdf_path, tcp_frame_name, robot_config, use_gravity, false),
                        trajectory_generator(torque_mapper, robot_config.dt),
                        sing_method(SingularityRobustnessMode::None),
                        ct_controller(robot_model, sing_method, controller_settings, trajectory_generator),
                        pd_plus_controller(robot_model, sing_method, controller_settings, trajectory_generator),
                        inverse_dyn_controller(robot_model, sing_method, controller_settings, trajectory_generator),
                        active_controller(&ct_controller),
                        n_x_indices(ConstIntVectorMap(robot_config.n_x_indices, robot_config.nx_red))
{
    init_default_config();
}

void WorkspaceController::switchController(ControllerType type)
{
    switch (type)
    {
    case ControllerType::CT:
        active_controller = &ct_controller;
        break;
    case ControllerType::PDPlus:
        active_controller = &pd_plus_controller;
        break;
    case ControllerType::InverseDynamics:
        active_controller = &inverse_dyn_controller;
        inverse_dyn_controller.init = true;
        break;
    }
    active_controller->traj_count = 0;
}

void WorkspaceController::init_default_config()
{
    // Initialize the default configuration
    Eigen::MatrixXd K_d = Eigen::MatrixXd::Zero(6, 6);
    K_d.diagonal() << 100, 200, 500, 200, 50, 50;
    Eigen::MatrixXd D_d = (2 * K_d).array().sqrt();

    Eigen::MatrixXd Kp1 = Eigen::MatrixXd::Zero(6, 6);
    Kp1.diagonal() << 100, 200, 500, 200, 50, 50;
    Eigen::MatrixXd Kd1 = (2 * Kp1).array().sqrt();

    controller_settings.id_settings.Kd1 = Kd1;
    controller_settings.id_settings.Kp1 = Kp1;
    controller_settings.id_settings.D_d = D_d;
    controller_settings.id_settings.K_d = K_d;

    controller_settings.pd_plus_settings.D_d = D_d;
    controller_settings.pd_plus_settings.K_d = K_d;

    controller_settings.ct_settings.Kd1 = Kd1;
    controller_settings.ct_settings.Kp1 = Kp1;
}

Eigen::VectorXd WorkspaceController::update(const double *const x_nq)
{
    double x_nq_red[robot_config.nx_red];

    // Convert nx to nx_red state
    for (casadi_uint i = 0; i < robot_config.nx_red; i++)
    {
        x_nq_red[i] = x_nq[n_x_indices[i]];
    }

    Eigen::Map<const Eigen::VectorXd> x_nq_vec(x_nq, robot_config.nx);
    Eigen::Map<const Eigen::VectorXd> x_nq_red_vec(x_nq_red, robot_config.nx_red);
    Eigen::VectorXd tau_red = active_controller->control(x_nq_red_vec);
    Eigen::VectorXd tau_full = torque_mapper.calc_full_torque(tau_red, x_nq_vec);
    return tau_full;
}

Eigen::VectorXd WorkspaceController::CTController::control(const Eigen::VectorXd &x)
{
    // Control parameters
    Eigen::MatrixXd Kd1 = controller_settings.ct_settings.Kd1;
    Eigen::MatrixXd Kp1 = controller_settings.ct_settings.Kp1;

    // Calculate J, J_pinv, J_p, C, M, C_rnea, g, q_p, x_err, x_err_p, x_d_p, x_d_pp
    calculateControlData(x);

    Eigen::VectorXd v = J_pinv * (x_d_pp - Kd1 * x_err_p - Kp1 * x_err - J_p * q_p);

    // Control Law Calculation
    Eigen::VectorXd tau = M * v + C_rnea;
    return tau;
}

Eigen::VectorXd WorkspaceController::InverseDynamicsController::control(const Eigen::VectorXd &x)
{
    // Control parameters
    Eigen::MatrixXd Kd1 = controller_settings.id_settings.Kd1;
    Eigen::MatrixXd Kp1 = controller_settings.id_settings.Kp1;
    Eigen::MatrixXd D_d = controller_settings.id_settings.D_d;
    Eigen::MatrixXd K_d = controller_settings.id_settings.K_d;

    calculateControlData(x);

    if (!init)
    {
        // Initialize on first iteration
        q_d_prev = q;
        q_p_d_prev = q_p;
        init = false;
    }

    Eigen::VectorXd q_d_pp = J_pinv * (x_d_pp - Kd1 * x_err_p - Kp1 * x_err - J_p * q_p);

    // Integrate q_pp two times to get q and q_p
    // Explicit euler with mid point rule

    // concatenate q and qp to a vector
    Eigen::VectorXd q_d = q_d_prev + dt * q_p_d_prev;
    Eigen::VectorXd q_p_d = q_p_d_prev + dt * q_d_pp;

    Eigen::VectorXd tau = M * q_d_pp + C * q_p_d + g - D_d * (q_p - q_p_d - K_d * (q - q_d));

    q_d_prev = q_d;
    q_p_d_prev = q_p_d;

    return tau;
}

Eigen::VectorXd WorkspaceController::PDPlusController::control(const Eigen::VectorXd &x)
{
    // Control parameters
    Eigen::MatrixXd D_d = controller_settings.pd_plus_settings.D_d;
    Eigen::MatrixXd K_d = controller_settings.pd_plus_settings.K_d;

    // Calculate J, J_pinv, J_p, C, M, C_rnea, g, q_p, x_err, x_err_p, x_d_p, x_d_pp
    calculateControlData(x);

    Eigen::MatrixXd J_pinv_T = J_pinv.transpose(); // Assuming J_pinv is computed somewhere earlier
    Eigen::MatrixXd Lambda = J_pinv_T * M * J_pinv;
    Lambda = 0.5 * (Lambda + Lambda.transpose());
    Eigen::VectorXd mu = J_pinv_T * (C - M * J_pinv * J_p) * J_pinv;
    Eigen::VectorXd F_g = J_pinv_T * g; // g is zero here

    Eigen::VectorXd F = Lambda * x_d_pp + mu * x_d_p + F_g - D_d * x_err_p - K_d * x_err;

    Eigen::VectorXd tau = J.transpose() * F;
    return tau;
}

void WorkspaceController::init_file_trajectory(casadi_uint traj_select, const casadi_real *x_k_ndof_ptr,
                                               double T_start, double T_poly, double T_end)
{
    trajectory_generator.init_file_trajectory(traj_select, x_k_ndof_ptr, T_start, T_poly, T_end);
}

void WorkspaceController::init_custom_trajectory(ParamPolyTrajectory param)
{
    trajectory_generator.init_custom_trajectory(param);
}