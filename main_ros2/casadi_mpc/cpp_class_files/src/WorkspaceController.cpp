#include "WorkspaceController.hpp"


WorkspaceController::WorkspaceController(const std::string &urdf_filename,
                                         const std::string &general_config_filename) 
                      : CommonBaseController(urdf_filename, "", general_config_filename),
                        ct_controller(robot_model, general_config_filename, trajectory_generator, tau_max_jump),
                        pd_plus_controller(robot_model, general_config_filename, trajectory_generator, tau_max_jump),
                        inverse_dyn_controller(robot_model, general_config_filename, trajectory_generator, tau_max_jump),
                        active_controller(&ct_controller), selected_controller_type(ControllerType::CT)
{
    update_config();
}

Eigen::MatrixXd BaseWorkspaceController::computeJacobianRegularization()
{
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
        // J_pinv = (J.transpose() * J + k * Eigen::MatrixXd::Identity(nq, nq))
        //         .ldlt()
        //         .solve(J.transpose());

        J_pinv = J.transpose() * (J * J.transpose() + k * Eigen::MatrixXd::Identity(6, 6)).ldlt().solve(Eigen::MatrixXd::Identity(6, nq));

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

        S.diagonal() = (S.diagonal().array() < epsilon).select(S.diagonal().array() / (S.diagonal().array().square() + epsilon * epsilon), S.diagonal());

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

void BaseWorkspaceController::update_controller_settings()
{
    nlohmann::json general_config = read_config<>(general_config_filename);
// PD CONTROLLER
    auto classic_ctl_settings = get_config_value<nlohmann::json>(general_config, "classic_controller_settings");
    Eigen::MatrixXd K_I_pd = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["PD"], "K_I").data(), 6).asDiagonal();
    Eigen::MatrixXd K_d_pd = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["PD"], "K_d").data(), 6).asDiagonal();
    Eigen::MatrixXd D_d_pd = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["PD"], "D_d").data(), 6).asDiagonal();

    // CT CONTROLLER
    Eigen::MatrixXd K_I_ct = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["CT"], "K_I").data(), 6).asDiagonal();
    Eigen::MatrixXd Kp1_ct = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["CT"], "Kp1").data(), 6).asDiagonal();
    Eigen::MatrixXd Kd1_ct = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["CT"], "Kd1").data(), 6).asDiagonal();

    // ID CONTROLLER
    Eigen::MatrixXd Kp1_id = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["ID"], "Kp1").data(), 6).asDiagonal();
    Eigen::MatrixXd Kd1_id = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["ID"], "Kd1").data(), 6).asDiagonal();

    Eigen::MatrixXd K_d_id = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["ID"], "K_d").data(), 6).asDiagonal();
    Eigen::MatrixXd D_d_id = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["ID"], "D_d").data(), 6).asDiagonal();

    // State Observer
    Eigen::MatrixXd Kd1_state_observer = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["StateObserver"], "Kd1").data(), 6).asDiagonal();
    Eigen::MatrixXd Kp1_state_observer = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(classic_ctl_settings["StateObserver"], "Kp1").data(), 6).asDiagonal();

    // REGULARIZATION SETTINGS
    auto reg_settings = get_config_value<nlohmann::json>(classic_ctl_settings, "regularization_settings");
    Eigen::VectorXd W_bar_N_nq = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(reg_settings, "W_bar_N").data(), 7);
    Eigen::MatrixXd W_bar_N = W_bar_N_nq(n_indices).asDiagonal();
    Eigen::VectorXd W_E_nq = Eigen::VectorXd::Map(get_config_value<std::vector<double>>(reg_settings, "W_E").data(), 7);
    Eigen::MatrixXd W_E = W_E_nq(n_indices).asDiagonal();

    controller_settings.pd_plus_settings.K_I = K_I_pd;
    controller_settings.pd_plus_settings.D_d = D_d_pd;
    controller_settings.pd_plus_settings.K_d = K_d_pd;
    controller_settings.ct_settings.K_I = K_I_ct;
    controller_settings.ct_settings.Kd1 = Kd1_ct;
    controller_settings.ct_settings.Kp1 = Kp1_ct;
    controller_settings.id_settings.Kd1 = Kd1_id;
    controller_settings.id_settings.Kp1 = Kp1_id;
    controller_settings.id_settings.D_d = D_d_id;
    controller_settings.id_settings.K_d = K_d_id;
    controller_settings.state_observer_settings.Kd1 = Kd1_state_observer;
    controller_settings.state_observer_settings.Kp1 = Kp1_state_observer;
    controller_settings.regularization_settings.mode = stringToRegularizationMode<std::string>(get_config_value<std::string>(reg_settings, "mode"));
    controller_settings.regularization_settings.k = get_config_value<double>(reg_settings, "k");
    controller_settings.regularization_settings.W_bar_N = W_bar_N;
    controller_settings.regularization_settings.W_E = W_E;
    controller_settings.regularization_settings.eps = get_config_value<double>(reg_settings, "eps");
    controller_settings.regularization_settings.eps_collinear = get_config_value<double>(reg_settings, "eps_collinear");
    controller_settings.regularization_settings.lambda_min = get_config_value<double>(reg_settings, "lambda_min");

    regularization_settings = controller_settings.regularization_settings;
    sing_method = controller_settings.regularization_settings.mode;

    traj_data_real_len = trajectory_generator.get_traj_data_real_len();

    tau_max_jump = get_config_value<double>(general_config, "tau_max_jump");
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

    Eigen::Quaterniond quat_d = vec2quat<double>(trajectory_generator.q_d.col(traj_count));
    Eigen::VectorXd omega_d = trajectory_generator.omega_d.col(traj_count);
    Eigen::VectorXd omega_d_p = trajectory_generator.omega_d_p.col(traj_count);

    // Errors
    Eigen::Matrix3d R_d = quat_d.toRotationMatrix();
    Eigen::Matrix3d R = robot_model.kinematicsData.R;
    Eigen::Matrix3d dR = R_d.transpose() * R;
    // Eigen::Quaterniond q_err_tmp(dR);
    Eigen::Vector4d q_err_tmp = rotm2quat_v4<double>(dR);
    // Eigen::Vector3d q_err = q_err_tmp.tail(3); // Only the orientation error part

    Eigen::Vector3d epsilon = q_err_tmp.tail(3);
    double eta = q_err_tmp(0);

    Eigen::Vector3d q_err = 2 * R_d * (eta * Eigen::MatrixXd::Identity(3, 3) + skew<>(epsilon)) * epsilon;

    x_e_p << p_p, omega_e;
    x_err << (p - p_d), q_err; // Error as quaternion
    x_err_p << (p_p - p_d_p), (omega_e - omega_d);

    x_d_p << p_d_p, omega_d;
    x_d_pp << p_d_pp, omega_d_p;

    J_pinv = computeJacobianRegularization();
}

void BaseWorkspaceController::calculateControlDataRPY(const Eigen::VectorXd &x)
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

    Eigen::MatrixXd J_geo = robot_model.kinematicsData.J;     // Geometric Jacobian to end-effector
    Eigen::MatrixXd J_geo_p = robot_model.kinematicsData.J_p; // Time derivative of geometric Jacobian
    //Eigen::Matrix3d R = robot_model.kinematicsData.R;     // Rotation matrix of end-effector

    // Current pose of end-effector
    Eigen::Vector3d p = robot_model.kinematicsData.p;
    Eigen::Matrix3d R = robot_model.kinematicsData.R;

    Eigen::Vector3d phi = rotm2rpy<double>(R); // Roll, pitch, yaw angles

    J = T_ext_inv<>(phi) * J_geo; // analytical Jacobian to end-effector

    Eigen::VectorXd y_p = J * q_p;
    Eigen::Vector3d p_p = y_p.head(3);     // Linear velocity of the end-effector
    Eigen::Vector3d phi_p = y_p.tail(3); // Angular velocity of the end-effector

    J_p = T_ext_inv<>(phi) * (J_geo_p - dT_ext<>(phi, phi_p) * J); // Time derivative of analytical Jacobian

    // Desired trajectory
    Eigen::VectorXd p_d = trajectory_generator.p_d.col(traj_count);
    Eigen::VectorXd p_d_p = trajectory_generator.p_d_p.col(traj_count);
    Eigen::VectorXd p_d_pp = trajectory_generator.p_d_pp.col(traj_count);

    Eigen::MatrixXd phi_d = trajectory_generator.phi_d.col(traj_count);     // Target rpy orientation
    Eigen::MatrixXd phi_d_p = trajectory_generator.phi_d_p.col(traj_count); // Target rpy orientation velocity
    Eigen::MatrixXd phi_d_pp = trajectory_generator.phi_d_pp.col(traj_count); // Target rpy orientation acceleration

    // Errors
    Eigen::Matrix3d R_d = rpy2rotm<double>(phi_d);
    Eigen::Matrix3d dR = R * R_d.transpose();
    
    Eigen::Vector3d phi_err = rotm2rpy<double>(dR);

    x_e_p << p_p, phi_p;

    x_err << (p - p_d), phi_err; // Error as quaternion
    x_err_p << (p_p - p_d_p), (phi_p - phi_d_p);

    x_d_p << p_d_p, phi_d_p;
    x_d_pp << p_d_pp, phi_d_pp;

    J_pinv = computeJacobianRegularization();
}

void BaseWorkspaceController::calculateControlDataID(const Eigen::VectorXd &x)
{
    robot_model.updateState(x);

    // Parameters
    q = x.head(nq);     // Time derivative of generalized coordinates
    q_p = x.tail(nq); // Time derivative of generalized coordinates

    // Current pose of end-effector
    Eigen::Vector3d p = robot_model.kinematicsData.p;
    J = robot_model.kinematicsData.J;     // Geometric Jacobian to end-effector
    J_p = robot_model.kinematicsData.J_p; // Time derivative of geometric Jacobian
    J_pinv = computeJacobianRegularization();

    Eigen::VectorXd y_p = J * q_p;

    Eigen::Vector3d p_p = y_p.head(3);     // Linear velocity of the end-effector
    Eigen::Vector3d omega_e = y_p.tail(3); // Angular velocity of the end-effector

    // Desired trajectory
    Eigen::VectorXd p_d = trajectory_generator.p_d.col(traj_count);
    Eigen::VectorXd p_d_p = trajectory_generator.p_d_p.col(traj_count);
    Eigen::VectorXd p_d_pp = trajectory_generator.p_d_pp.col(traj_count);

    Eigen::Quaterniond quat_d = vec2quat<double>(trajectory_generator.q_d.col(traj_count));
    Eigen::VectorXd omega_d = trajectory_generator.omega_d.col(traj_count);
    Eigen::VectorXd omega_d_p = trajectory_generator.omega_d_p.col(traj_count);

    // Errors
    Eigen::Matrix3d R_d = quat_d.toRotationMatrix();
    Eigen::Matrix3d R = robot_model.kinematicsData.R;
    Eigen::Matrix3d dR = R * R_d.transpose();

    // Eigen::Quaterniond q_err_tmp(dR);

    // Eigen::Quaterniond quat = robot_model.kinematicsData.quat;
    // Eigen::Quaterniond q_err_tmp = quat * quat_d.conjugate();
    // Eigen::Vector3d q_err = q_err_tmp.vec(); // Only the orientation error part
    
    Eigen::Vector4d q_err_tmp = rotm2quat_v4<double>(dR);
    Eigen::Vector3d q_err = q_err_tmp.tail(3); // Only the orientation error part

    x_err << (p - p_d), q_err; // Error as quaternion
    x_err_p << (p_p - p_d_p), (omega_e - omega_d);
    x_d_p << p_d_p, omega_d;
    x_d_pp << p_d_pp, omega_d_p;

    M = robot_model.dynamicsData.M; // Inertia matrix
    C = robot_model.dynamicsData.C; // Coriolis matrix
    C_rnea = robot_model.dynamicsData.C_rnea; // Coriolis matrix computed with RNEA
    g = robot_model.dynamicsData.g; // Gravitational forces
}

void BaseWorkspaceController::stateObserver(const Eigen::VectorXd &x)
{
    // Control parameters
    Eigen::MatrixXd Kd1 = controller_settings.state_observer_settings.Kd1;
    Eigen::MatrixXd Kp1 = controller_settings.state_observer_settings.Kp1;

    if (init) // Initialize on first iteration
    {
        q_d = x.head(nq);
        q_p_d = x.tail(nq);
        init = false;
    }

    Eigen::VectorXd x_d = Eigen::VectorXd::Zero(nx);
    x_d << q_d, q_p_d;

    calculateControlDataID(x);

    Eigen::VectorXd q_d_pp = J_pinv * (x_d_pp - Kd1 * x_err_p - Kp1 * x_err - J_p * q_p_d);

    // Integrate q_pp two times to get q and q_p
    q_d = q_d + dt * q_p_d;
    q_p_d = q_p_d + dt * q_d_pp;
}

Eigen::VectorXd WorkspaceController::update_control(const Eigen::VectorXd &x_nq)
{
    double x_k[nx_red];

    // Convert nx to nx_red state
    for (uint i = 0; i < nx_red; i++)
    {
        x_k[i] = x_nq[n_x_indices[i]];
    }

    Eigen::Map<const Eigen::VectorXd> x_k_vec(x_k, nx_red);
    Eigen::VectorXd tau_red = active_controller->control(x_k_vec);
    active_controller->increment_traj_count();
    Eigen::VectorXd tau_full = torque_mapper.calc_full_torque(tau_red, x_nq);

    error_check(tau_full);

    return tau_full;
}

void WorkspaceController::reset(const casadi_real *const x_k_in)
{
    Eigen::Map<const Eigen::VectorXd> x_k(x_k_in, nx_red);
    active_controller->reset();
    tau_full_prev = Eigen::VectorXd::Zero(nq);

    active_controller->q_d = x_k.head(nq_red);
    active_controller->q_p_d = x_k.tail(nq_red);
    reset_error_flag();
}

void WorkspaceController::reset()
{
    Eigen::VectorXd x_0_init = get_transient_traj_x0_red_init();
    reset(x_0_init.data());
}

Eigen::VectorXd WorkspaceController::CTController::control(const Eigen::VectorXd &x)
{
    // Control parameters
    Eigen::MatrixXd K_I = controller_settings.ct_settings.K_I;
    Eigen::MatrixXd Kd1 = controller_settings.ct_settings.Kd1;
    Eigen::MatrixXd Kp1 = controller_settings.ct_settings.Kp1;

    calculateControlData(x);

    Eigen::VectorXd v = J_pinv * (x_d_pp - Kd1 * x_err_p - Kp1 * x_err - K_I*x_I_err - J_p * q_p);

    // // Control Law Calculation
    Eigen::VectorXd tau = M * v + C_rnea;

    x_I_err << x_I_err + x_err * dt;

    return tau;
}


Eigen::VectorXd WorkspaceController::InverseDynamicsController::control(const Eigen::VectorXd &x)
{
    // Control parameters
    Eigen::MatrixXd Kd1 = controller_settings.id_settings.Kd1;
    Eigen::MatrixXd Kp1 = controller_settings.id_settings.Kp1;
    Eigen::MatrixXd D_d = controller_settings.id_settings.D_d;
    Eigen::MatrixXd K_d = controller_settings.id_settings.K_d;

    if (init) // Initialize on first iteration
    {
        q_d = x.head(nq);
        q_p_d = x.tail(nq);
        init = false;
    }

    calculateControlDataID(x);

    Eigen::VectorXd q_d_pp = J_pinv * (x_d_pp - Kd1 * x_err_p - Kp1 * x_err - J_p * q_p_d);

    // Integrate q_pp two times to get q and q_p
    q_d = q_d + dt * q_p_d;
    q_p_d = q_p_d + dt * q_d_pp;

    q = x.head(nq);
    q_p = x.tail(nq);

    Eigen::VectorXd tau = M * q_d_pp + C * q_p_d + g - D_d * (q_p - q_p_d) - K_d * (q - q_d);

    return tau;
}

Eigen::VectorXd WorkspaceController::PDPlusController::control(const Eigen::VectorXd &x)
{
    // Control parameters
    Eigen::MatrixXd K_I = controller_settings.pd_plus_settings.K_I;
    Eigen::MatrixXd D_d = controller_settings.pd_plus_settings.D_d;
    Eigen::MatrixXd K_d = controller_settings.pd_plus_settings.K_d;

    calculateControlData(x);

    Eigen::MatrixXd M_inv = M.inverse();
    Eigen::MatrixXd lambda = (J * M_inv * J.transpose()).inverse();
    J_pinv = M_inv * J.transpose() * lambda;
    Eigen::MatrixXd mu_bar = lambda * (J * M_inv * C - J_p);
    Eigen::MatrixXd mu_x = mu_bar * J_pinv;
    Eigen::VectorXd F_g = lambda * J * M_inv * g;

    Eigen::VectorXd F = lambda * x_d_pp + mu_bar * q_p - mu_x * x_err_p + 0 * F_g - D_d * x_err_p - K_d * x_err - K_I * x_I_err;

    Eigen::MatrixXd K_0 = Eigen::MatrixXd::Identity(7, 7);
    Eigen::MatrixXd K_1 = Eigen::MatrixXd::Identity(7, 7);
    const Eigen::VectorXd q_n = (Eigen::VectorXd(7) << 0, -M_PI / 4, 0, -3 * M_PI / 4, 0, M_PI / 2, M_PI / 4).finished();
    Eigen::VectorXd tau_n = (Eigen::MatrixXd::Identity(7, 7) - J.transpose() * J_pinv.transpose()) * (-K_0 * q_p - K_1 * (q - q_n));

    Eigen::VectorXd tau = J.transpose() * F + g;

    x_I_err << x_I_err + x_err * dt;

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
        break;
    }
    selected_controller_type = type;
    active_controller->reset();
}


void WorkspaceController::update_trajectory_data(const double *x_k_ndof_ptr)
{
    Eigen::Map<const Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);
    Eigen::VectorXd x_k = Eigen::VectorXd::Zero(nx_red);
    reset(x_k.data());
}

void WorkspaceController::update_config()
{
    update_controller_settings();
    // dt = active_controller->get_dt();
    // torque_mapper.update_config(dt);
}