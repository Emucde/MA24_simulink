if(~exist('parameter_str', 'var'))
    parameters_7dof;
end

n = param_robot.n_DOF; % Define the number of degrees of freedom.

param_sing = struct();
param_sing.n = n;
param_sing.n_red = 6;
param_sing.y_d_t = [0.3921; 0.3921; 0.5211];
param_sing.R_d = [0 1 0; 1 0 0; 0 0 -1];

param_sing.y_d_p = [0; 0; 0; 0; 0; 0];
param_sing.y_d_pp = [0; 0; 0; 0; 0; 0];

param_sing.k_q = 1e2*ones(n,1);
param_sing.k_q_p = 1e2*ones(n,1);
param_sing.KK_q = 1e1*eye(n);
param_sing.KK_q_p = 1e1*eye(n);

param_sing.Q_y = 1*eye(6);
param_sing.Q_y_p = 1e2*diag([1, 1, 1, 1, 1, 1]);
param_sing.Q_y_pp = eye(6);

param_sing.k_m = 1e10;

param_sing.coplanar_axes = [1 2]; % joint 1 and 2 should be coplanar

% hom_transform_joint_1_py
% ...
% TODO: Kostenfunktion so ändern, dass
% 1. k Gelenksachsen sind kollinear zueinander: Kreuzprodukt = 0
% 2. k Gelenksachsen in einer Ebene liegen: det(a1, a2, ..., ak) = 0
% 3. k Gelenksachsen sich in einem Punkt schneiden: 
% oder kombinationen davon.
% d. h. man muss nur die Gelenksachsen berechnen überprüfen.

% Achtung: rij \in [-1, 1] und kij >= 0
R_J_d = struct( ...
    'r21', 0, ...
    'r31', 0, 'r32', 0, ...
    'r41', 0, 'r42', 0, 'r43', 0, ...
    'r51', 0, 'r52', 0, 'r53', 0, 'r54', 0, ...
    'r61', 0, 'r62', 0, 'r63', 0, 'r64', 0, 'r65', 0 ...
);

K_J = struct(... 
    'k21', 0, ...
    'k31', 0, 'k32', 0, ...
    'k41', 0, 'k42', 0, 'k43', 0, ...
    'k51', 0, 'k52', 0, 'k53', 0, 'k54', 0, ...
    'k61', 0, 'k62', 0, 'k63', 0, 'k64', 0, 'k65', 0 ...
);
% x = [q, q_p, q_pp] \in \mathbb R^(3n)

param_sing = set_collin_matrices(R_J_d, K_J, param_sing);

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point', 'MaxFunctionEvaluations', 8e6);

A = [];
b = [];
Aeq = [];
beq = [];
delta = 0.2;
lb = [param_robot.q_limit_lower + delta; -inf(6,1); -inf(6,1)];
ub = [param_robot.q_limit_upper - delta; inf(6,1); inf(6,1)];

% % Solve the constrained optimization problem
% [x_opt, f_val] = fmincon(@(x) f_cost(x, param_sing), [q_0; q_0_p; q_0_pp], A, b, Aeq, beq, lb, ub, [], options);

% % Extract the optimized values
% q_opt = x_opt(1:n);
% q_p_opt = x_opt(n+1:2*n);
% q_pp_opt = x_opt(2*n+1:3*n);

% H = hom_transform_endeffector_py(q_opt);
% J = geo_jacobian_endeffector_py(q_opt);
% J_tilde = J ./ vecnorm(J, 2);
% R_J = J_tilde' * J_tilde;

% w = sqrt(det(J * J'));

input_dir = [s_fun_path, '/casadi_functions/'];
traj_select_mpc = 1;

% q_0 = param_traj.q_0(:, traj_select_mpc)+rand(n,1);

q_0 = rand(n,1);
q_1 = q_0 + ones(n,1)*0.1;
q_0 = min(max(q_0, param_robot.q_limit_lower*0.5), param_robot.q_limit_upper*0.5);
q_1 = min(max(q_1, param_robot.q_limit_lower*0.5), param_robot.q_limit_upper*0.5);
q_0(param_robot.n_indices_fixed) = 0;
q_1(param_robot.n_indices_fixed) = 0;

H_0 = hom_transform_endeffector_py(q_0);

p_d_0 = H_0(1:3, 4);
R_d_0 = H_0(1:3, 1:3);
q_d_0 = quat_R_endeffector_py(H_0(1:3, 1:3));

MPC_solver = 'fatrop';
use_jit = false;
opt_problem_find_multiple_singular_solutions;
gen_opt_problem_test;

coptimflags = '-Ofast -march=native -flto'; % Optimization flag for compilation
% casadi_fun_to_mex(f_opt, [s_fun_path, '/mpc_c_sourcefiles'], [s_fun_path, '/matlab_functions'], casadi_func_name, coptimflags, MPC_solver, false);


N_min = 10;
qq_min = repmat({[q_0, q_1]}, 1, N_min);
w_min = 100*ones(N_min, 1);
delta_H_min = 100*ones(N_min, 1);
delta_q_min = 100*ones(N_min, 1);
krit_min = 100*ones(N_min, 1);

% könnte man auch zufällig ändern
Qt_1_ref = 1e2*diag([1, 1, 1]);
Qr_1_ref = 1e0*diag([1, 1, 1]);
Qt_2_ref = 1e2*diag([1, 1, 1]);
Qr_2_ref = 1e0*diag([1, 1, 1]);
Qt_3_ref = 1e8*diag([1, 1, 1]);
Qr_3_ref = 1e8*diag([1, 1, 1]);
Q4_ref = 1e1*eye(n_red);
Q5_ref = 1e1*eye(n_red);
q1_manip_ref = 1e4;
q2_manip_ref = 1e4;
dq_eps_ref = 1e2;
q_eps_ref = 1e4;

mpc_init_reference_values = [y_d_0(:); Qt_1_ref(:); Qr_1_ref(:); Qt_2_ref(:); Qr_2_ref(:); Qt_3_ref(:); Qr_3_ref(:); Q4_ref(:); Q5_ref(:); q1_manip_ref; q2_manip_ref; dq_eps_ref; q_eps_ref];
init_guess_0 = [x_init_guess_0(:); epsilon_max; lam_x_init_guess_0(:); lam_g_init_guess_0(:)];
qq_n_sol = zeros(n, 2);
q_min = zeros(n, 1);
min_idx=1;
tic
for i=1:20000
    q_0 = q_min + pi*rand(n, 1);
    q_1 = q_0 + pi*rand(n, 1);
    q_0 = min(max(q_0, param_robot.q_limit_lower*0.5), param_robot.q_limit_upper*0.5);
    q_1 = min(max(q_1, param_robot.q_limit_lower*0.5), param_robot.q_limit_upper*0.5);
    q_0(param_robot.n_indices_fixed) = 0;
    q_1(param_robot.n_indices_fixed) = 0;

    H_0 = hom_transform_endeffector_py(q_0);
    p_d_0 = H_0(1:3, 4);
    q_d_0 = quat_R_endeffector_py(H_0(1:3, 1:3));
    
    y_d_0 = [p_d_0; q_d_0];
    mpc_init_reference_values(1:7) = y_d_0;

    x_init_guess_0 = [q_0(n_indices) q_1(n_indices)];
    init_guess_0(1:2*n_red) = x_init_guess_0(:);

    xsol = find_singularities(mpc_init_reference_values, init_guess_0); % Dont run this without args otherwise matlab crashes!!!
    qq_sol = reshape(full(xsol), n_red, 2);
    qq_n_sol(n_indices, :) = qq_sol;

    H1=hom_transform_endeffector_py(qq_n_sol(:,1));
    H2=hom_transform_endeffector_py(qq_n_sol(:,2));
    JJ1 = geo_jacobian_endeffector_py(qq_n_sol(:,1));
    JJ2 = geo_jacobian_endeffector_py(qq_n_sol(:,2));
    w1 = sqrt(det(JJ1 * JJ1'));
    w2 = sqrt(det(JJ2 * JJ2'));
    delta_q = norm(qq_n_sol(:,1)-qq_n_sol(:,2), 2);
    w = (w1 + w2) / 2;
    delta_H = norm(H1-H2, 2);
    krit_min_new = (1/delta_q + delta_H + w) / 3;
    indices = find(krit_min_new < krit_min);

    if(~isempty(indices))
        [~, max_idx] = max(krit_min);
        qq_min{max_idx} = qq_n_sol;
        w_min(max_idx) = w;
        delta_H_min(max_idx) = delta_H;
        delta_q_min(max_idx) = delta_q;
        krit_min(max_idx) = krit_min_new;
        [~, min_idx] = min(krit_min);
        q_min(n_indices) = qq_min{min_idx}(n_indices,1+round(rand(1)));
        disp(['Iteration: ', num2str(i), ', krit_min: ', num2str(krit_min_new), ', w: ', num2str(w), ', delta_H: ', num2str(delta_H), ', delta_q: ', num2str(delta_q)]);
    end
end
toc;

% display all solutions
for i=1:N_min
    fprintf('Solution %d:\n\n', i);
    disp('H1:');
    disp(hom_transform_endeffector_py(qq_min{i}(:,1)));
    disp('H2:');
    disp(hom_transform_endeffector_py(qq_min{i}(:,2)));
    JJ1 = geo_jacobian_endeffector_py(qq_min{i}(:,1));
    JJ2 = geo_jacobian_endeffector_py(qq_min{i}(:,2));
    w1 = sqrt(det(JJ1 * JJ1'));
    w2 = sqrt(det(JJ2 * JJ2'));
    fprintf('q1=[');fprintf('%f, ', qq_min{i}(1:end-1,1));fprintf('%f];\n\n', qq_min{i}(end,1));
    fprintf('q2=[');fprintf('%f, ', qq_min{i}(1:end-1,2));fprintf('%f];\n\n', qq_min{i}(end,2));
    fprintf('w1 = %f, w2 = %f\n\n', w1, w2);
    fprintf('||q1 - q2|| = %f\n\n', norm(qq_min{i}(:,1) - qq_min{i}(:,2), 2));
    fprintf('------------------------------------------------------------------------------------\n\n');
end

function f = f_cost(x, param)
    n = param.n;

    q = x(1:n);
    q_p = x(n+1:2*n);
    q_pp = x(2*n+1:3*n);

    y_d_t = param.y_d_t;
    R_d = param.R_d;
    R_J_d = param.R_J_d;
    K_J = param.K_J;
    k_q = param.k_q;
    k_q_p = param.k_q_p;
    KK_q = param.KK_q;
    KK_q_p = param.KK_q_p;
    k_m = param.k_m;

    y_d_p = param.y_d_p;
    y_d_pp = param.y_d_pp;

    Q_y = param.Q_y;
    Q_y_p = param.Q_y_p;
    Q_y_pp = param.Q_y_pp;

    %{
    H_q1 = hom_transform_joint_1_py(q);
    H_q2 = hom_transform_joint_2_py(q);
    H_q3 = hom_transform_joint_3_py(q);
    H_q4 = hom_transform_joint_4_py(q);
    H_q5 = hom_transform_joint_5_py(q);
    H_q6 = hom_transform_joint_6_py(q);

    ax_q1 = get_rotax(H_q1(1:3,1:3));
    ax_q2 = get_rotax(H_q2(1:3,1:3));
    ax_q3 = get_rotax(H_q3(1:3,1:3));
    ax_q4 = get_rotax(H_q4(1:3,1:3));
    ax_q5 = get_rotax(H_q5(1:3,1:3));
    ax_q6 = get_rotax(H_q6(1:3,1:3));
    %}

    H = hom_transform_endeffector_py(q);
    J = geo_jacobian_endeffector_py(q);
    J_p = geo_jacobian_endeffector_p_py(q, q_p);

    y_t = H(1:3,4);
    R = H(1:3,1:3);

    y_p = J * q_p;
    y_pp = J_p * q_p + J * q_pp;

    q_err = rotation2quaternion(R*R_d');
    vec_err = q_err(2:4);

    f_y = Q_norm([y_t - y_d_t; vec_err], Q_y) + Q_norm(y_p - y_d_p, Q_y_p) + Q_norm(y_pp - y_d_pp, Q_y_pp);

    J_tilde = J ./ vecnorm(J, 2);
    R_J = J_tilde' * J_tilde;

    f_J = sum( 1/2 * K_J .* (abs(R_J) - R_J_d).^2, 'all');

    f_q = k_q' * exp(-KK_q * abs(q_p)) + k_q_p' * exp(-KK_q_p * abs(q_pp));

    f_m = k_m * sqrt(det(J*J'));

    f = f_y + f_J + f_q + f_m;
end

function Q = Q_norm(z, Q)
    Q = dot( z, mtimes(Q, z));
end

function rot_ax = get_rotax(R)
   quat = rotation2quaternion(R);
   rot_alpha_scale = 2*acos(quat(1));
   if(rot_alpha_scale == 0)
       rot_ax = [0; 0; 0];
   else
       rot_ax = quat(2:4) / sin(rot_alpha_scale/2);
   end
end