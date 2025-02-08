%% CT Controller Parameters
K_d_t = 3*diag([1 1 1]);
K_p_t = K_d_t^2/4;

K_d_r = 3*diag([1 1 1]);
K_p_r = K_d_r^2/4;

ctrl_param.ct.Kd1 = blkdiag(K_d_t, K_d_r);
ctrl_param.ct.Kp1 = blkdiag(K_p_t, K_p_r);

% Cartesian PD Control
% M2: eigenmodes - wie soll das gehen wenn M in R7 ist?
% K_0 = 10*eye(n);
% xi_0 = 0.1;

% q_0_ref = param_traj.q_0(:, 1); % ist eh für alle Trajektorien gleich (TODO)
% M_0 = inertia_matrix_py(q_0_ref);
% m_0 = diag(M_0);
% omega_0 = sqrt(K_0 ./ m_0);
% D_0 = eye(n) * (m_0 * 2 * xi_0 .* omega_0);
% ctrl_param.pd.D_d = D_0;
% ctrl_param.pd.K_d = K_0;

ctrl_param.pd.D_d = 3*eye(6); % (NOT USED)
% ctrl_param.pd.K_d = 3*eye(6); % (NOT USED)
ctrl_param.pd.K_d = ctrl_param.pd.D_d^2/4; % (NOT USED)

if(~exist('current_traj_value', 'var'))
    current_traj_value = 1;
end

%% Jointspace Control
ctrl_param.pd.D_d_jointspace = 3*eye(n); % (NOT USED)
ctrl_param.pd.K_d_jointspace = ctrl_param.pd.D_d_jointspace^2/4; % (NOT USED)

%% Singularity robustness
% 0: no sing robust (not fully true, use pinv() from matlab - this commands eliminates too small singular values)
% 1: simpliy use J_pinv = (J'*J + k*E)^(-1)*J' = (J'*J + k*E)\J'
% 2: use Sugihara singular robust method: J_pinv = (J'*W_E*J + W_N)^(-1)*J' = (J'*W_E*J + W_N)\J'
% 3: set sing values sigma_i < eps to eps
% 4: set sing values sigma_i < eps to sqrt(sigma_i^2 + eps^2)
% 5: collinearity approach 1: only works if joint is exact replaceable (linear dependent) by one other joint (but not by multiple joints!!)
% 6: collinearity approach after steinböck: works for all joints
ctrl_param.regularization.mode = 0;

% 1:
ctrl_param.regularization.k = 1e-5;

% 2:
ctrl_param.regularization.W_bar_N = 1e-3*param_robot.sugihara_limb_vector;
% ctrl_param.regularization.W_bar_N = 1e-1*ones(n,1);
ctrl_param.regularization.W_E = 1 * eye(6); %ctrl_param.regularization.w_bar_N;

% 3, 4:
ctrl_param.regularization.eps  = 1e-1;

% 5:
ctrl_param.regularization.eps_collinear = 0.95;

ctrl_param.regularization.lambda_min = 5e-3;

%% nullspace for CT controller
ctrl_param.ct.q_n = param_robot.q_n; % q_n = (q_max + q_min) / 2;
ctrl_param.ct.K_n = 1e-2*eye(n);
ctrl_param.ct.D_n = sqrt(4*ctrl_param.ct.K_n) + 1 * eye(n);

%ctrl_param.ct.K_n = 64*eye(n);
%ctrl_param.ct.D_n = 16*eye(n);

ctrl_param.k_n_nl = 1e-10*eye(n);
ctrl_param.nl_spring_threshold = [1; 1; 1; 1; 1; 1; 1];

% DEBUG
ew_test_CT_CTRL = [diag(-ctrl_param.ct.Kd1/2 + sqrt(ctrl_param.ct.Kd1^2/4 - ctrl_param.ct.Kp1)) diag(-ctrl_param.ct.Kd1/2 - sqrt(ctrl_param.ct.Kd1^2/4 - ctrl_param.ct.Kp1))]';
plot_eigenvalues_controller_text([ew_test_CT_CTRL ew_test_CT_CTRL*0], 'Eigenvalues CT Ctrl', '');