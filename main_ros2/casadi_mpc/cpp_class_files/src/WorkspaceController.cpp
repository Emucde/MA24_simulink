#include "WorkspaceController.hpp"


WorkspaceController::WorkspaceController(
    const std::string &urdf_path,
    const std::string &tcp_frame_name,
    bool use_gravity) : urdf_path(urdf_path),
                        tcp_frame_name(tcp_frame_name),
                        robot_config(get_robot_config()),
                        n_indices(ConstIntVectorMap(robot_config.n_indices, robot_config.nq_red)),
                        n_x_indices(ConstIntVectorMap(robot_config.n_x_indices, robot_config.nx_red)),
                        nq(robot_config.nq), nx(robot_config.nx), nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
                        controller_settings(init_default_controller_settings()),
                        robot_model(urdf_path, tcp_frame_name, robot_config, use_gravity, true),
                        torque_mapper(urdf_path, tcp_frame_name, robot_config, use_gravity, false),
                        trajectory_generator(torque_mapper, robot_config.dt),
                        sing_method(RegularizationMode::None),
                        ct_controller(robot_model, sing_method, controller_settings, trajectory_generator),
                        pd_plus_controller(robot_model, sing_method, controller_settings, trajectory_generator),
                        inverse_dyn_controller(robot_model, sing_method, controller_settings, trajectory_generator),
                        active_controller(&ct_controller)
{
}


Eigen::MatrixXd BaseWorkspaceController::computeJacobianRegularization()
{
    // Assume necessary member variables exist:
    // `J` (the reduced Jacobian matrix), `x_err` (the error vector),
    // `nq` (dimensions), `sing_method` (regularization method),
    // `ctrl_param` (contains the regularization parameters)

    Eigen::MatrixXd J = robot_model.kinematicsData.J;
    Eigen::MatrixXd J_pinv = Eigen::MatrixXd::Zero(J.cols(), J.rows());

    switch (sing_method)
    {
    case RegularizationMode::None:
        J_pinv = (J.transpose() * J).inverse() * J.transpose();
        break;
    case RegularizationMode::Damping:
    {
        double k = regularization_settings.k;
        // J_pinv = (J.transpose() * J + k * Eigen::MatrixXd::Identity(nq, nq)).inverse() * J.transpose();
        J_pinv = (J.transpose() * J + k * Eigen::MatrixXd::Identity(nq, nq))
                .ldlt()
                .solve(J.transpose());
    }
    break;
    case RegularizationMode::CompleteOrthogonalDecomposition:
        J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
        break;
    case RegularizationMode::JacobiSVD:
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        J_pinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
    }
    break;
    case RegularizationMode::ThresholdSmallSingularValues:
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
    case RegularizationMode::TikhonovRegularization:
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
    case RegularizationMode::SimpleCollinearity:
    {
        // calculate singular values of J
        Eigen::VectorXd J_scale = J.colwise().norm().cwiseInverse();                 // J_scale = J / colsum(J)
        Eigen::MatrixXd J_tilde = J.array().rowwise() * J_scale.transpose().array(); // J_tilde = J_scale * J_scale'
        Eigen::MatrixXd JJ_colin = J_tilde.transpose() * J_tilde;                    // JJ_colin = J_tilde' * J_tilde

        // Calculate Eigenvalues of JJ_colin
        // Eigen::EigenSolver<Eigen::MatrixXd> solver(JJ_colin);
        // Eigen::VectorXd eigenvalues = solver.eigenvalues().real();

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(JJ_colin, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd singular_values = svd.singularValues();

        // Get the amount of all eigenvalues that are smaller than regularization_settings.eps_collinear
        int ew_count = (singular_values.array() < regularization_settings.eps_collinear).count();
        if (ew_count > 0)
        {
            // Calculate collinearity column norm (changed from row to column)
            Eigen::VectorXd R = JJ_colin.cwiseAbs().colwise().sum();

            // Create a sorted copy of R (ascending order)
            Eigen::VectorXd sortedR = R;
            std::sort(sortedR.data(), sortedR.data() + sortedR.size());

            double R_threshold;
            if(ew_count == singular_values.size())
            {
                // ensure that J_new has at least one column
                // If all columns are collinear, the one with the smallest collinearity is chosen
                R_threshold = sortedR[0];
            }
            else
            {
                // sortedR.size() = 6
                // ew_count = 1 ... 5
                // max_idx = 6-1-1 = 4 ... 6-1-5 = 0
                double max_idx = sortedR.size() - 1 - ew_count;
                R_threshold = sortedR[max_idx];
            }

            // Create a boolean vector for values below this threshold
            Eigen::Array<bool, Eigen::Dynamic, 1> mask = (R.array() > R_threshold);

            std::vector<double> col_indices;
            for (int i = 0; i < sortedR.size(); ++i)
            {
                if (!mask[i])
                {
                    col_indices.push_back(i);
                }
            }

            // Select columns using the index vector
            Eigen::MatrixXd J_new = J(Eigen::all, col_indices);

            Eigen::MatrixXd J_pinv_new = J_new.completeOrthogonalDecomposition().pseudoInverse();
            J_pinv = Eigen::MatrixXd::Zero(J.cols(), J.rows());
            J_pinv(col_indices, Eigen::all) = J_pinv_new;
        }
        else
        {
            J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
        }
        /*
        // calculate singular values of J
        Eigen::VectorXd J_scale = J.colwise().norm().cwiseInverse();                 // J_scale = J / colsum(J)
        Eigen::MatrixXd J_tilde = J.array().rowwise() * J_scale.transpose().array(); // J_tilde = J_scale * J_scale'
        Eigen::MatrixXd JJ_colin = J_tilde.transpose() * J_tilde;                    // JJ_colin = J_tilde' * J_tilde

        double damping_factor = 0.1;
        // Calculate Eigenvalues of JJ_colin
        // Eigen::EigenSolver<Eigen::MatrixXd> solver(JJ_colin);
        // Eigen::VectorXd eigenvalues = solver.eigenvalues().real();

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(JJ_colin, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd singular_values = svd.singularValues();

        // Get the amount of all eigenvalues that are smaller than regularization_settings.eps_collinear
        int ew_count = (singular_values.array() < regularization_settings.eps_collinear).count();

        // Calculate collinearity column norm (changed from row to column)
        Eigen::VectorXd R = JJ_colin.cwiseAbs().colwise().sum();

        // Create a sorted copy of R (ascending order)
        Eigen::VectorXd sortedR = R;
        std::sort(sortedR.data(), sortedR.data() + sortedR.size());

        double R_threshold;
        if (ew_count == singular_values.size())
        {
            // ensure that J_new has at least one column
            // If all columns are collinear, the one with the smallest collinearity is chosen
            R_threshold = sortedR[0];
        }
        else
        {
            // sortedR.size() = 6
            // ew_count = 1 ... 5
            // max_idx = 6-1-1 = 4 ... 6-1-5 = 0
            double max_idx = sortedR.size() - 1 - ew_count;
            R_threshold = sortedR[max_idx];
        }

        Eigen::Array<bool, Eigen::Dynamic, 1> mask = (R.array() > R_threshold);

        if (ew_count > 0)
        {
            for (int i = 0; i < R.size(); ++i)
            {
                if (mask[i])
                {
                    // Gradually decrease weight for highly collinear columns
                    column_weights[i] -= damping_factor * (column_weights[i] - 0.0); // Smoothly approach 0
                }
            }
        }
        else
        {
            for (int i = 0; i < R.size(); ++i)
            {
                // Gradually restore weight for non-collinear columns
                column_weights[i] += damping_factor * (1.0 - column_weights[i]); // Smoothly approach 1
            }
        }
        std::cout << "Column weights: " << column_weights.transpose() << std::endl;
        column_weights.cwiseMin(Eigen::VectorXd::Constant(column_weights.size(), 1))
            .cwiseMax(Eigen::VectorXd::Constant(column_weights.size(), 0));

        Eigen::MatrixXd J_new = J.array().rowwise() * column_weights.transpose().array();

        J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
        */
    }
    break;
    case RegularizationMode::SteinboeckCollinearity:
    {
        Eigen::VectorXd J_scale = J.colwise().norm().cwiseInverse();                 // J_scale = J / colsum(J)
        Eigen::MatrixXd J_tilde = J.array().rowwise() * J_scale.transpose().array(); // J_tilde = J_scale * J_scale'
        Eigen::MatrixXd JJ_colin = J_tilde.transpose() * J_tilde;                    // JJ_colin = J_tilde' * J_tilde

        // Print matrices
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(JJ_colin, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd V = svd.matrixV().real();

        // Get the singular values
        Eigen::VectorXd singularValues = svd.singularValues();

        // Create an index vector for columns to keep
        Eigen::Array<bool, Eigen::Dynamic, 1> mask = (singularValues.array() >regularization_settings.lambda_min);
        std::vector<double> col_indices;
        for (int i = 0; i < singularValues.size(); ++i)
        {
            if (mask[i])
            {
                col_indices.push_back(i);
            }
        }

        // Select columns using the index vector
        Eigen::MatrixXd V_new = V(Eigen::all, col_indices);

        Eigen::MatrixXd T_bar = J_scale.asDiagonal() * V_new;
        Eigen::MatrixXd J_new = J * T_bar;

        J_pinv = T_bar * (J_new.fullPivLu().inverse()); // More robust inverse calculation
    }
    break;
    case RegularizationMode::RegularizationBySingularValues:
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



Eigen::VectorXd WorkspaceController::update(const double *const x_nq)
{
    double x_nq_red[nx_red];

    // Convert nx to nx_red state
    for (int i = 0; i < nx_red; i++)
    {
        x_nq_red[i] = x_nq[n_x_indices[i]];
    }

    Eigen::Map<const Eigen::VectorXd> x_nq_vec(x_nq, nx);
    Eigen::Map<const Eigen::VectorXd> x_nq_red_vec(x_nq_red, nx_red);
    Eigen::VectorXd tau_red = active_controller->control(x_nq_red_vec);
    Eigen::VectorXd tau_full = torque_mapper.calc_full_torque(tau_red, x_nq_vec);
    return tau_full;
}

void WorkspaceController::reset()
{
    active_controller->traj_count = 0;
    tau_full_prev = Eigen::VectorXd::Zero(nq);

    Eigen::VectorXd x_0_init = get_act_traj_x0_red_init();
    inverse_dyn_controller.q_d_prev = x_0_init.head(nq_red);
    inverse_dyn_controller.q_p_d_prev = x_0_init.tail(nq_red);
    inverse_dyn_controller.init = true;
    reset_error_flag();
}

void BaseWorkspaceController::calculateControlData(const Eigen::VectorXd &x)
{
    // Update the robot model state with joint positions and velocities
    robot_model.updateState(x);

    // Parameters
    q = robot_model.jointData.q;     // Time derivative of generalized coordinates
    q_p = robot_model.jointData.q_p; // Time derivative of generalized coordinates

    // Get matrices and variables from the robot model
    M = robot_model.dynamicsData.M; // Inertia matrix
    C = robot_model.dynamicsData.C; // Coriolis matrix
    C_rnea = robot_model.dynamicsData.C_rnea; // Coriolis matrix computed with RNEA
    g = robot_model.dynamicsData.g; // Gravitational forces

    J = robot_model.kinematicsData.J;     // Geometric Jacobian to end-effector
    J_p = robot_model.kinematicsData.J_p; // Time derivative of geometric Jacobian
    //Eigen::Matrix3d R = robot_model.kinematicsData.R;     // Rotation matrix of end-effector

    // Current pose of end-effector
    Eigen::Vector3d p = robot_model.kinematicsData.p;
    Eigen::VectorXd y_p = J * q_p;

    Eigen::Vector3d p_p = y_p.head(3);     // Linear velocity of the end-effector
    Eigen::Vector3d omega_e = y_p.tail(3); // Angular velocity of the end-effector

    // Desired trajectory
    Eigen::VectorXd p_d = trajectory_generator.p_d.col(traj_count);
    Eigen::VectorXd p_d_p = trajectory_generator.p_d_p.col(traj_count);
    Eigen::VectorXd p_d_pp = trajectory_generator.p_d_pp.col(traj_count);

    Eigen::Quaterniond q_d = vec2quat<double>(trajectory_generator.q_d.col(traj_count));
    Eigen::VectorXd omega_d = trajectory_generator.omega_d.col(traj_count);
    Eigen::VectorXd omega_d_p = trajectory_generator.omega_d_p.col(traj_count);

    // Errors
    Eigen::Matrix3d R_d = q_d.toRotationMatrix();
    Eigen::Matrix3d R = robot_model.kinematicsData.R;
    Eigen::Matrix3d dR = R * R_d.transpose();
    // Eigen::Quaterniond q_err_tmp(dR);

    // Eigen::Quaterniond quat = robot_model.kinematicsData.quat;
    // Eigen::Quaterniond q_err_tmp = quat * q_d.conjugate();
    // Eigen::Vector3d q_err = q_err_tmp.vec(); // Only the orientation error part
    
    Eigen::Vector4d q_err_tmp = rotm2quat_v4<double>(dR);
    Eigen::Vector3d q_err = q_err_tmp.tail(3); // Only the orientation error part

    x_err = Eigen::VectorXd::Zero(6);
    x_err << (p - p_d), q_err; // Error as quaternion

    x_err_p = Eigen::VectorXd::Zero(6);
    x_err_p << (p_p - p_d_p), (omega_e - omega_d);

    x_d_p = Eigen::VectorXd::Zero(6);
    x_d_p << p_d_p, omega_d;

    x_d_pp = Eigen::VectorXd::Zero(6);
    x_d_pp << p_d_pp, omega_d_p;

    J_pinv = computeJacobianRegularization();
    traj_count++;
}

void BaseWorkspaceController::calculateControlDataID(const Eigen::VectorXd &x, const Eigen::VectorXd &x_d)
{
    robot_model.jointData.q = x.head(nq);
    robot_model.jointData.q_p = x.tail(nq);
    robot_model.computeKinematics();

    // Parameters
    q = robot_model.jointData.q;     // Time derivative of generalized coordinates
    q_p = robot_model.jointData.q_p; // Time derivative of generalized coordinates

    // Current pose of end-effector
    Eigen::Vector3d p = robot_model.kinematicsData.p;
    Eigen::VectorXd y_p = J * q_p;

    Eigen::Vector3d p_p = y_p.head(3);     // Linear velocity of the end-effector
    Eigen::Vector3d omega_e = y_p.tail(3); // Angular velocity of the end-effector

    // Desired trajectory
    Eigen::VectorXd p_d = trajectory_generator.p_d.col(traj_count);
    Eigen::VectorXd p_d_p = trajectory_generator.p_d_p.col(traj_count);
    Eigen::VectorXd p_d_pp = trajectory_generator.p_d_pp.col(traj_count);

    Eigen::Quaterniond q_d = vec2quat<double>(trajectory_generator.q_d.col(traj_count));
    Eigen::VectorXd omega_d = trajectory_generator.omega_d.col(traj_count);
    Eigen::VectorXd omega_d_p = trajectory_generator.omega_d_p.col(traj_count);

    // Errors
    Eigen::Matrix3d R_d = q_d.toRotationMatrix();
    Eigen::Matrix3d R = robot_model.kinematicsData.R;
    Eigen::Matrix3d dR = R * R_d.transpose();
    // Eigen::Quaterniond q_err_tmp(dR);

    // Eigen::Quaterniond quat = robot_model.kinematicsData.quat;
    // Eigen::Quaterniond q_err_tmp = quat * q_d.conjugate();
    // Eigen::Vector3d q_err = q_err_tmp.vec(); // Only the orientation error part
    
    Eigen::Vector4d q_err_tmp = rotm2quat_v4<double>(dR);
    Eigen::Vector3d q_err = q_err_tmp.tail(3); // Only the orientation error part

    x_err = Eigen::VectorXd::Zero(6);
    x_err << (p - p_d), q_err; // Error as quaternion

    x_err_p = Eigen::VectorXd::Zero(6);
    x_err_p << (p_p - p_d_p), (omega_e - omega_d);

    x_d_p = Eigen::VectorXd::Zero(6);
    x_d_p << p_d_p, omega_d;

    x_d_pp = Eigen::VectorXd::Zero(6);
    x_d_pp << p_d_pp, omega_d_p;

    // Update the robot model state with joint positions and velocities
    robot_model.updateState(x_d);

    // Get matrices and variables from the robot model
    M = robot_model.dynamicsData.M; // Inertia matrix
    C = robot_model.dynamicsData.C; // Coriolis matrix
    C_rnea = robot_model.dynamicsData.C_rnea; // Coriolis matrix computed with RNEA
    g = robot_model.dynamicsData.g; // Gravitational forces

    J = robot_model.kinematicsData.J;     // Geometric Jacobian to end-effector
    J_p = robot_model.kinematicsData.J_p; // Time derivative of geometric Jacobian
    //Eigen::Matrix3d R = robot_model.kinematicsData.R;     // Rotation matrix of end-effector

    J_pinv = computeJacobianRegularization();

    traj_count++;
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

    Eigen::VectorXd x_d = Eigen::VectorXd::Zero(nx);
    x_d << q_d_prev, q_p_d_prev;

    calculateControlDataID(x, x_d);

    if (init)
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

    Eigen::VectorXd tau = M * q_d_pp + C * q_p_d + g - D_d * (q_p - q_p_d) - K_d * (q - q_d);

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
    Eigen::MatrixXd mu = J_pinv_T * (C - M * J_pinv * J_p) * J_pinv;
    Eigen::VectorXd F_g = J_pinv_T * g; // g is zero here

    Eigen::VectorXd F = Lambda * x_d_pp + mu * x_d_p + F_g - D_d * x_err_p - K_d * x_err;

    Eigen::VectorXd tau = J.transpose() * F;
    return tau;
}

ControllerType WorkspaceController::get_classic_controller_type(const std::string &controller_type_str)
{
    if (controller_type_str == "PD")
        return ControllerType::PDPlus;
    else if (controller_type_str == "CT")
        return ControllerType::CT;
    else if (controller_type_str == "ID")
        return ControllerType::InverseDynamics;
    else
    {
        throw std::invalid_argument("Invalid controller type");
        return ControllerType::PDPlus;
    }
}

std::string WorkspaceController::get_classic_controller_string(ControllerType controller_type)
{
    switch (controller_type)
    {
        case ControllerType::PDPlus:
            return "PD";
        case ControllerType::CT:
            return "CT";
        case ControllerType::InverseDynamics:
            return "ID";
        default:
            throw std::invalid_argument("Invalid controller type");
    }
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


void WorkspaceController::simulateModelEuler(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt)
{
    Eigen::Map<Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);

    // Check for errors
    if (x_k_ndof.hasNaN())
    {
        reset();
        error_flag = ErrorFlag::NAN_DETECTED;
        std::cerr << "NaN values detected in the joint vector!" << std::endl;
        return;
    }

    Eigen::Map<const Eigen::VectorXd> tau(tau_ptr, nq);
    torque_mapper.simulateModelEuler(x_k_ndof, tau, dt);
}


void WorkspaceController::simulateModelRK4(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt)
{
    Eigen::Map<Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);

    // Check for errors
    if (x_k_ndof.hasNaN())
    {
        reset();
        error_flag = ErrorFlag::NAN_DETECTED;
        std::cerr << "NaN values detected in the joint vector!" << std::endl;
        return;
    }

    Eigen::Map<const Eigen::VectorXd> tau(tau_ptr, nq);
    torque_mapper.simulateModelRK4(x_k_ndof, tau, dt);
}


ControllerSettings WorkspaceController::init_default_controller_settings()
{
    // Initialize the default configuration
    Eigen::MatrixXd K_d = Eigen::DiagonalMatrix<double, 6>(100, 100, 100, 20, 20, 20);
    Eigen::MatrixXd D_d = (2 * K_d).array().sqrt();

    Eigen::MatrixXd Kp1 = Eigen::DiagonalMatrix<double, 6>(100, 100, 100, 50, 50, 50);
    Eigen::MatrixXd Kd1 = (2 * Kp1).array().sqrt();

    ControllerSettings controller_settings_default;

    controller_settings_default.id_settings.Kd1 = Kd1;
    controller_settings_default.id_settings.Kp1 = Kp1;
    controller_settings_default.id_settings.D_d = D_d;
    controller_settings_default.id_settings.K_d = K_d;

    controller_settings_default.pd_plus_settings.D_d = D_d;
    controller_settings_default.pd_plus_settings.K_d = K_d;

    controller_settings_default.ct_settings.Kd1 = Kd1;
    controller_settings_default.ct_settings.Kp1 = Kp1;

    controller_settings_default.regularization_settings = init_default_regularization_settings();

    return controller_settings_default;
}


RegularizationSettings WorkspaceController::init_default_regularization_settings()
{
    // Initialize the default configuration
    RegularizationSettings regularization_settings_default;

    Eigen::MatrixXd W_bar_N = Eigen::MatrixXd::Identity(nq_red, nq_red);
    Eigen::Map<const Eigen::VectorXd>sugihara_limb_vector(robot_config.sugihara_limb_vector, nq);
    W_bar_N.diagonal() << sugihara_limb_vector(n_indices);
    // Eigen::Map<Eigen::VectorXd>(W_bar_N.diagonal().data(), nq_red) = Eigen::Map<const Eigen::VectorXd>(robot_config.sugihara_limb_vector, robot_config.nq)(n_indices);
    Eigen::MatrixXd W_E = Eigen::MatrixXd::Identity(nq_red, nq_red);

    regularization_settings_default.mode = RegularizationMode::None; // Regularization mode
    // RegularizationMode::Damping
    regularization_settings_default.k = 1e-5;             // Regularization parameter J_pinv = (J^T J + k I)^-1 J^T

    // Sugihara Method, not implemented
    regularization_settings_default.W_bar_N = W_bar_N;    // Parameter for singularity robustness (sugihara)
    regularization_settings_default.W_E = W_E;            // Weight matrix for the error (sugihara)
    
    // RegularizationMode::TikhonovRegularization
    // RegularizationMode::ThresholdSmallSingularValues
    // RegularizationMode::RegularizationBySingularValues
    regularization_settings_default.eps = 1e-3;           // Regularization parameter for singular values
    
    // RegularizationMode::SimpleCollinearity
    regularization_settings_default.eps_collinear = 1e-3; // Threshold for collinear joints
    
    // RegularizationMode::SteinboeckCollinearity
    regularization_settings_default.lambda_min = 1e-2;    // Minimum eigenvalue for regularization
    
    return regularization_settings_default;
}

Eigen::VectorXd WorkspaceController::get_traj_x0_red_init(casadi_uint traj_select)
{
    Eigen::VectorXd x0_init = *trajectory_generator.get_traj_file_x0_init(traj_select);
    Eigen::VectorXd x0_init_red = x0_init(n_x_indices);
    return x0_init_red;
}

Eigen::VectorXd WorkspaceController::get_act_traj_x0_red_init()
{
    Eigen::VectorXd x0_init = *trajectory_generator.get_traj_x0_init();
    Eigen::VectorXd x0_init_red = x0_init(n_x_indices);
    return x0_init_red;
}

void WorkspaceController::error_check(Eigen::VectorXd &tau_full)
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

    // Check for Inf values in the torque vector

}