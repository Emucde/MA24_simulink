%% CT Controller Parameters
K_d_t = 16*diag([1 1 1]);
K_p_t = K_d_t^2/4;

K_d_r = 16*diag([1 1 1]);
K_p_r = K_d_r^2/4;

ct_ctrl_param.Kd1 = blkdiag(K_d_t, K_d_r);
ct_ctrl_param.Kp1 = blkdiag(K_p_t, K_p_r);

%% Singularity robustness
% 0: no sing robust
% 1: simpliy use J_pinv = (J'*J + k*E)^(-1)*J' = (J'*J + k*E)\J'
% 2: use Sugihara singular robust method: J_pinv = (J'*W_E*J + W_N)^(-1)*J' = (J'*W_E*J + W_N)\J'
% 3: set sing values sigma_i < eps to sign(sigma_i)/q_i_max
ct_ctrl_param.mode = 4;

% 1:
ct_ctrl_param.k = 1e-2;

% 2:
ct_ctrl_param.W_bar_N = 1e-3*param_robot.sugihara_limb_vector;
% ct_ctrl_param.W_bar_N = 1e-1*ones(n,1);
ct_ctrl_param.W_E = 1e-2 * eye(n); %ct_ctrl_param.w_bar_N;

% 3:
ct_ctrl_param.eps  = 1e-1;

% 4:
ct_ctrl_param.eps_collinear = 0.99;
%ct_ctrl_param.D_n_colin = 1e0*eye(n);
%ct_ctrl_param.K_n_colin = ct_ctrl_param.D_n_colin^2/4;



%% nullspace for CT controller
ct_ctrl_param.q_n = param_robot.q_n; % q_n = (q_max + q_min) / 2;
ct_ctrl_param.K_n = 1e-2*eye(n);
ct_ctrl_param.D_n = sqrt(4*ct_ctrl_param.K_n) + 1 * eye(n);

%ct_ctrl_param.K_n = 64*eye(n);
%ct_ctrl_param.D_n = 16*eye(n);

ct_ctrl_param.k_n_nl = 10*eye(n);
ct_ctrl_param.nl_spring_threshold = [0.2; 0.2; 0.2; 0.2; 0.2; 0.2; 0.2];

% DEBUG
ew_test_CT_CTRL = [diag(-ct_ctrl_param.Kd1/2 + sqrt(ct_ctrl_param.Kd1^2/4 - ct_ctrl_param.Kp1)) diag(-ct_ctrl_param.Kd1/2 - sqrt(ct_ctrl_param.Kd1^2/4 - ct_ctrl_param.Kp1))]';
plot_eigenvalues_controller_text([ew_test_CT_CTRL ew_test_CT_CTRL*0], 'Eigenvalues CT Ctrl', '');