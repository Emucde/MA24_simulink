%% Trajectory X: random sing, Workspace, Polynomial
q_0 = param_robot.q_0_ref';
q_0(n_indices) = rand(6,1);
q_0 = min(max(q_0, param_robot.q_limit_lower*0.5), param_robot.q_limit_upper*0.5);
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); quat_R_endeffector_py(R_init)]; % better to exact start in point

mx = 5;

Q_pos = diag(10.^(mx*rand(7,1)))*diag(rand(7,1));  % Weight for the position error in the cost function.
Q_m = 1e10;                               % Weight for the manipulability error in the cost function.
Q_q = diag(10.^(2*rand(6,1)))*diag(rand(6,1));            % Weight for the deviaton of q_sol to q_d
Q_nl = 0 * eye(n_red);                    % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!

R_J_d = struct( ...
    'r21', round(rand(1)), ...
    'r31', round(rand(1)), 'r32', round(rand(1)), ...
    'r41', round(rand(1)), 'r42', round(rand(1)), 'r43', round(rand(1)), ...
    'r51', round(rand(1)), 'r52', round(rand(1)), 'r53', round(rand(1)), 'r54', round(rand(1)), ...
    'r61', round(rand(1)), 'r62', round(rand(1)), 'r63', round(rand(1)), 'r64', round(rand(1)), 'r65', round(rand(1)) ...
);


K_J = struct(... 
    'k21', 10.^(mx*rand(1))*round(rand(1)), ...
    'k31', 10.^(mx*rand(1))*round(rand(1)), 'k32', 10.^(mx*rand(1))*round(rand(1)), ...
    'k41', 10.^(mx*rand(1))*round(rand(1)), 'k42', 10.^(mx*rand(1))*round(rand(1)), 'k43', 10.^(mx*rand(1))*round(rand(1)), ...
    'k51', 10.^(mx*rand(1))*round(rand(1)), 'k52', 10.^(mx*rand(1))*round(rand(1)), 'k53', 10.^(mx*rand(1))*round(rand(1)), 'k54', 10.^(mx*rand(1))*round(rand(1)), ...
    'k61', 10.^(mx*rand(1))*round(rand(1)), 'k62', 10.^(mx*rand(1))*round(rand(1)), 'k63', 10.^(mx*rand(1))*round(rand(1)), 'k64', 10.^(mx*rand(1))*round(rand(1)), 'k65', 10.^(10*rand(1))*round(rand(1)) ...
);

param_inv_kin = struct;
param_inv_kin.xe0 = xe0;
param_inv_kin.q_d = q_0(n_indices);
param_inv_kin.Q_pos = Q_pos;
param_inv_kin.Q_m = Q_m;
param_inv_kin.Q_q = Q_q;
param_inv_kin.Q_nl = Q_nl;
param_inv_kin.K_J = K_J;
param_inv_kin.R_J_d = R_J_d;
param_inv_kin.tolerance = 1e-3;
param_inv_kin.random_q0_count = 10;

fun_list = {@hom_transform_joint_1_py, @hom_transform_joint_2_py, @hom_transform_joint_4_py, @hom_transform_joint_5_py, @hom_transform_joint_6_py, @hom_transform_joint_7_py};
rsel = 1+round(5*rand(1));
hom_transform_jointi = fun_list{rsel};
H = hom_transform_jointi(q_0);
xe_d = [H(1:3,4); quat_R_endeffector_py(H(1:3,1:3))];


param_inv_kin.xpos_joint1 = xe_d;
param_inv_kin.xpos_joint2 = xe_d;
param_inv_kin.xpos_joint3 = xe_d;
param_inv_kin.xpos_joint4 = xe_d;
param_inv_kin.xpos_joint5 = xe_d;
param_inv_kin.xpos_joint6 = xe_d;
param_inv_kin.xpos_joint7 = xe_d;

param_inv_kin.Q_joint1 = 10.^(mx*rand(1))*round(rand(1))*round(rand(1));
param_inv_kin.Q_joint2 = 10.^(mx*rand(1))*round(rand(1))*round(rand(1));
param_inv_kin.Q_joint3 = 10.^(mx*rand(1))*round(rand(1))*round(rand(1));
param_inv_kin.Q_joint4 = 10.^(mx*rand(1))*round(rand(1))*round(rand(1));
param_inv_kin.Q_joint5 = 10.^(mx*rand(1))*round(rand(1))*round(rand(1));
param_inv_kin.Q_joint6 = 10.^(mx*rand(1))*round(rand(1))*round(rand(1));
param_inv_kin.Q_joint7 = 10.^(mx*rand(1))*round(rand(1))*round(rand(1));

[q_target_red, ~] = inverse_kinematics(param_robot, param_inv_kin, 'not used');
q_target = q_0;
q_target(n_indices) = q_target_red;


H_target = hom_transform_endeffector_py(q_target);
R_target = H_target(1:3,1:3);
xeT = [H_target(1:3,4); quat_R_endeffector_py(R_target)];

J = geo_jacobian_endeffector_py(q_target);
J_red = J(:, n_indices);

spd = 0.2;

vec = zeros(6,3);
vec2 = zeros(6,3);
% aus sicht des inertialsystems
%vec(:,1) = [spd 0 0 0 0 0]';
%vec(:,2) = [0 spd 0 0 0 0]';
%vec(:,3) = [0 0 spd 0 0 0];

% aus sicht des tcp
Hx = H_target * [1/2*spd 0 0 1]';
Hy = H_target * [0 1/2*spd 0 1]';
Hz = H_target * [0 0 1/2*spd 1]';

vec(:,1) = [Hx(1:3); 0;0;0];
vec(:,2) = [Hy(1:3); 0;0;0];
vec(:,3) = [Hz(1:3); 0;0;0];

Hx2 = H_target * [-1*spd 0 0 1]';
Hy2 = H_target * [0 -1*spd 0 1]';
Hz2 = H_target * [0 0 -1*spd 1]';

vec2(:,1) = [Hx2(1:3); 0;0;0];
vec2(:,2) = [Hy2(1:3); 0;0;0];
vec2(:,3) = [Hz2(1:3); 0;0;0];

q_p1 = J_red\vec(:,1);
q_p2 = J_red\vec(:,2);
q_p3 = J_red\vec(:,3);

%[q_p_min, idx] = min([norm(q_p1), norm(q_p2), norm(q_p3)]);
[q_p_min, idx] = max([norm(q_p1), norm(q_p2), norm(q_p3)]);

%xe2 = xeT + [1/2*vec(:, idx);0];
%xe3 = xeT - [vec(:, idx);0];
xe2 = [vec(1:3, idx); xeT(4:7)];
xe3 = [vec2(1:3, idx); xeT(4:7)];

traj_struct = struct;
traj_struct.N = 5;
traj_struct.q_0 = q_target;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = -ones(n,traj_struct.N);
traj_struct.pose = [xeT, xe2, xe2, xe3, xe3];
traj_struct.rotation = cat(3, R_target, R_target, R_target, R_target, R_target);
traj_struct.time = [0; 1; 2; 3; 10];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'test sing 1 init point, Polynomial, Jointspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;