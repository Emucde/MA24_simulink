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

WorkspaceController::WorkspaceController(
    const std::string &urdf_path,
    const std::string &tcp_frame_name,
    bool use_gravity,
    ControllerSettings &controller_settings) : urdf_path(urdf_path),
                                               tcp_frame_name(tcp_frame_name),
                                               robot_config(get_robot_config()),
                                               controller_settings(controller_settings),
                                               robot_model(urdf_path, tcp_frame_name, robot_config, use_gravity),
                                               torque_mapper(urdf_path, tcp_frame_name, robot_config, use_gravity, false),
                                               trajectory_generator(torque_mapper, robot_config.dt),
                                               sing_method(SingularityRobustnessMode::None),
                                               ct_controller(robot_model, sing_method, controller_settings, trajectory_generator),
                                               pd_plus_controller(robot_model, sing_method, controller_settings, trajectory_generator),
                                               inverse_dyn_controller(robot_model, sing_method, controller_settings, trajectory_generator)
{
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
}

 void WorkspaceController::update(const double *const x) {
	// calculateRobotData(x);
    // active_controller->control();
    return;
 }

Eigen::VectorXd WorkspaceController::CTController::control(double* x)
{
    return Eigen::VectorXd();
    /*
    function tau  = fcn(robot_model, robot_model_fixed, param_traj_data, traj_select, state_traj, K_d_in, D_d_in, ctrl_param, param_robot)

    persistent cnt;
    persistent run_flag;

    if(isempty(cnt))
        cnt = 1;
        run_flag = 0;
    end

    % Parameters
    %q = robot_model.q;      % generalized coordinates
    q_p = robot_model.q_p;  % time derivative of generalized coordinates

    M = robot_model.M;      % inertia matrix
    C = robot_model.C;      % coriolis matrix
    %g = robot_model.g;      % gravitational forces
    %C_rnea = robot_model.C_rnea; % C_rnea(q, q_p) = C(q, q_p)q_p + g(q) = n(q, q_p)

    J = robot_model.J;      % geometric jacobian to end-effector

    J_p = robot_model.J_p;  % time derivative of geometric jacobian to end-effector
    H = robot_model.H;

    n = param_robot.n_DOF;
    n_red = param_robot.n_red;
    n_indices = param_robot.n_indices;
    n_indices_fixed = param_robot.n_indices_fixed;

    yt_indices = param_robot.yt_indices;
    yr_indices = param_robot.yr_indices;
    y_indices =  [yt_indices 3+yr_indices];

    m = numel(yt_indices) + numel(yr_indices);

    M_red = M(n_indices, n_indices);
    C_red = C(n_indices, n_indices);
    %C_rnea_red = C_rnea(n_indices);

    J_red = J(y_indices, n_indices);
    J_p_red = J_p(y_indices, n_indices);

    %g_red = g(n_indices);

    % Control parameters
    %D_d = ctrl_param.pd.D_d(y_indices, y_indices);
    %K_d = ctrl_param.pd.K_d(y_indices, y_indices);
    D_d = diag(D_d_in(y_indices));
    K_d = diag(K_d_in(y_indices));

    % Current Pose of Endeffector
    x_p = J*q_p;

    p = H(yt_indices,4);
    R = H(1:3,1:3);
    p_p  = x_p(yt_indices);
    omega_e = x_p(3+yr_indices); % Drehwinkelgeschwindigkeit des Endeffektors

    % Desired Trajectory
    p_d    = param_traj_data.p_d(yt_indices, cnt, traj_select);
    p_d_p  = param_traj_data.p_d_p(yt_indices, cnt, traj_select);
    p_d_pp = param_traj_data.p_d_pp(yt_indices, cnt, traj_select);

    q_d = param_traj_data.q_d(:, cnt, traj_select);
    R_d = param_traj_data.R_d(:, :, cnt, traj_select);
    omega_d = param_traj_data.omega_d(yr_indices, cnt, traj_select);
    omega_d_p = param_traj_data.omega_d_p(yr_indices, cnt, traj_select);

    % Errors
    %RR = R*R_d';
    %RR = R_d'*R;
    %q_err_tmp   = rotm2quat_v4(RR);
    quat = rotm2quat_v4(R);
    quat = quat * my_sign(quat(1));
    q_d = q_d * my_sign(q_d(1));
    q_err_tmp = quat_mult(quat, quat_inv(q_d));
    %RR = R*R_d' - R_d*R';
    %q_err_tmp = [1; RR(3,2); RR(1,3); RR(2,1)];

    q_err = q_err_tmp(1+yr_indices);

    x_err   = [p-p_d;      q_err];   % Error as quaternion
    x_err_p = [p_p-p_d_p;  omega_e-omega_d];
    x_d_p   = [p_d_p;      omega_d];
    x_d_pp  = [p_d_pp;     omega_d_p];


    % CT control law
    %% Cartesian Endeffector Pose

    sing_method = ctrl_param.regularization.mode;

    J_pinv_T = J_pinv';
    Lambda = J_pinv_T*M_red*J_pinv;
    Lambda = 1/2 * (Lambda + Lambda');
    mu = J_pinv_T*(C_red - M_red*J_pinv*J_p_red)*J_pinv;
    %mu = Lambda * (J / (M) * C - J_p);
    %F_g = J_pinv_T*g_red; gravity is ignored here!

    %Lambda = inv(J)' * (C - M * inv(J)*J_p)*inv(J);
    %mu = inv(J)' * (C - M*inv(J)*J_p)*inv(J);
    %F_g = inv(J)'*g;

    F_red = Lambda*x_d_pp + mu*x_d_p - D_d*x_err_p - K_d*x_err;

    tau_red = J_red' * F_red;

    q_pp_red = M_red\(tau_red - C(n_indices, :)*q_p - 0); % gravity is ignored here
    q_pp = zeros(n, 1);
    q_pp(n_indices) = q_pp_red;

    % v1
    %tau = M*q_pp + C*q_p;

    %v2
    tau = zeros(n,1);
    tau(n_indices) = tau_red;

    % e_fixed = zeros(n,1);
    % e_fixed(n_indices_fixed)=1;
    % K = eye(n);
    % K(:,n_indices_fixed) = [];
    % 
    % tau(n_indices_fixed) = e_fixed' * (M*K*q_pp_red + C*q_p);
    %v3: fehler zw. v2,v3 zu v1: 1e-16
    tau(n_indices_fixed) = M(n_indices_fixed,:)*q_pp + C(n_indices_fixed,:)*q_p;

    [run_flag, cnt] = update_traj_cnt(state_traj, run_flag, cnt, param_traj_data.N);

    end

    function y = my_sign(x)
        y = (x >= 0)*2 - 1;
    end
    */
}

Eigen::VectorXd WorkspaceController::InverseDynamicsController::control(double* x)
{
    return Eigen::VectorXd();
}

Eigen::VectorXd WorkspaceController::PDPlusController::control(double* x)
{
    // Update the robot model state with joint positions and velocities
    robot_model.updateState(Eigen::VectorXd::Map(x, nx));

    // Parameters
    Eigen::VectorXd q_p = robot_model.jointData.q_p; // Time derivative of generalized coordinates

    // Get matrices and variables from the robot model
    Eigen::MatrixXd M = robot_model.dynamicsData.M; // Inertia matrix
    Eigen::MatrixXd C = robot_model.dynamicsData.C; // Coriolis matrix
    Eigen::VectorXd g = robot_model.dynamicsData.g; // Gravitational forces

    Eigen::MatrixXd J = robot_model.kinematicsData.J;     // Geometric Jacobian to end-effector
    Eigen::MatrixXd J_p = robot_model.kinematicsData.J_p; // Time derivative of geometric Jacobian
    Eigen::MatrixXd H = robot_model.kinematicsData.H;     // Homogeneous transformation

    // Control parameters
    Eigen::MatrixXd D_d = controller_settings.pd_plus_settings.D_d;
    Eigen::MatrixXd K_d = controller_settings.pd_plus_settings.K_d;

    // Current pose of end-effector
    Eigen::Vector3d p = robot_model.kinematicsData.p;
    Eigen::VectorXd y_p = J * q_p;

    Eigen::Vector3d p_p = y_p.head(3); // Linear velocity of the end-effector
    Eigen::Vector3d omega_e = y_p.tail(3); // Angular velocity of the end-effector

    // Desired trajectory
    Eigen::VectorXd p_d = trajectory_generator.p_d;
    Eigen::VectorXd p_d_p = trajectory_generator.p_d_p;
    Eigen::VectorXd p_d_pp = trajectory_generator.p_d_pp;

    Eigen::VectorXd q_d = trajectory_generator.q_d;
    Eigen::VectorXd omega_d = trajectory_generator.omega_d;
    Eigen::VectorXd omega_d_p = trajectory_generator.omega_d_p;

    // Errors
    Eigen::Quaterniond quat = robot_model.kinematicsData.quat;

    Eigen::VectorXd q_err_tmp = quat * q_d.conjugate();  // Quaternion error
    Eigen::VectorXd q_err = q_err_tmp.tail(3); // Only the orientation error part

    Eigen::VectorXd x_err = Eigen::VectorXd::Zero(6);
    x_err << (p - p_d), q_err; // Error as quaternion
    
    Eigen::VectorXd x_err_p = Eigen::VectorXd::Zero(6);
    x_err_p << (p_p - p_d_p), (omega_e - omega_d);
    
    Eigen::VectorXd x_d_p = Eigen::VectorXd::Zero(6);
    x_d_p << p_d_p, omega_d;
    
    Eigen::VectorXd x_d_pp = Eigen::VectorXd::Zero(6);
    x_d_pp << p_d_pp, omega_d_p;

    // Control law implementation
    Eigen::MatrixXd J_pinv = computeJacobianRegularization();

    Eigen::MatrixXd J_pinv_T = J_pinv.transpose(); // Assuming J_pinv is computed somewhere earlier
    Eigen::MatrixXd Lambda = J_pinv_T * M * J_pinv;
    Lambda = 0.5 * (Lambda + Lambda.transpose());
    Eigen::VectorXd mu = J_pinv_T * (C - M * J_pinv * J_p) * J_pinv;

    Eigen::VectorXd F = Lambda * x_d_pp + mu * x_d_p - D_d * x_err_p - K_d * x_err;

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