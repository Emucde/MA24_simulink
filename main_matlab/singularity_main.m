if(~exist('parameter_str', 'var'))
    parameters_7dof;
end

n = param_robot.n_DOF; % Define the number of degrees of freedom.

param_sing = struct();
param_sing.n = n;
param_sing.y_d_t = [0.3921; 0.3921; 0.5211];
param_sing.R_d = [0 1 0; 1 0 0; 0 0 -1];

param_sing.y_d_p = [0; 0; 0; 0; 0; 0];
param_sing.y_d_pp = [0; 0; 0; 0; 0; 0];

param_sing.k_q = ones(n,1);
param_sing.k_q_p = ones(n,1);
param_sing.KK_q = eye(n);
param_sing.KK_q_p = eye(n);

param_sing.Q_y = 1*eye(6);
param_sing.Q_y_p = diag([1, 1, 1e5, 1, 1, 1]);
param_sing.Q_y_pp = eye(6);

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
    'r41', 1, 'r42', 0, 'r43', 0, ...
    'r51', 0, 'r52', 0, 'r53', 0, 'r54', 0, ...
    'r61', 0, 'r62', 0, 'r63', 0, 'r64', 0, 'r65', 0 ...
);

K_J = struct(... 
    'k21', 0, ...
    'k31', 0, 'k32', 0, ...
    'k41', 1e5, 'k42', 0, 'k43', 0, ...
    'k51', 0, 'k52', 0, 'k53', 0, 'k54', 0, ...
    'k61', 0, 'k62', 0, 'k63', 0, 'k64', 0, 'k65', 0 ...
);
% x = [q, q_p, q_pp] \in \mathbb R^(3n)

param_sing = set_collin_matrices(R_J_d, K_J, param_sing);

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');

A = [];
b = [];
Aeq = [];
beq = [];
lb = [param_robot.q_limit_lower; -inf(7,1); -inf(7,1)];
ub = [param_robot.q_limit_upper; inf(7,1); inf(7,1)];

% Solve the constrained optimization problem
[x_opt, f_val] = fmincon(@(x) f_cost(x, param_sing), [q_0; q_0_p; q_0_pp], A, b, Aeq, beq, lb, ub, [], options);

% Extract the optimized values
q_opt = x_opt(1:n);
q_p_opt = x_opt(n+1:2*n);
q_pp_opt = x_opt(2*n+1:3*n);

H = hom_transform_endeffector_py(q_opt);
J = geo_jacobian_endeffector_py(q_opt);
J_tilde = J ./ vecnorm(J, 2);
R_J = J_tilde' * J_tilde;

w = sqrt(det(J * J'));

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

    y_d_p = param.y_d_p;
    y_d_pp = param.y_d_pp;

    Q_y = param.Q_y;
    Q_y_p = param.Q_y_p;
    Q_y_pp = param.Q_y_pp;

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

    f_J = sum( 1/2 * K_J .* (R_J - R_J_d).^2, 'all');

    f_q = k_q' * exp(-KK_q * abs(q_p)) + k_q_p' * exp(-KK_q_p * abs(q_pp));

    f = f_y + f_J + f_q;
end

function Q = Q_norm(z, Q)
    Q = dot( z, mtimes(Q, z));
end

function param = set_collin_matrices(R_J_d, K_J, param)
    n = param.n;
    R = eye(n);
    K = zeros(n);
    for i = 2:n
        for j = 1:i-1
            R(i,j) = R_J_d.("r"+i+""+j);
            R(j,i) = R(i,j);
            K(i,j) = K_J.("k"+i+""+j);
            K(j,i) = K(i,j);
        end
    end
    param.R_J_d = R;
    param.K_J = K;
end