%% Calculate target positions
cnt = 1;
random_traj = false;

if(random_traj)
    param_traj_fr3_no_hand_6dof_random_traj;
else

%% Trajectory 1: Stretch arm out
disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);
q_0 = param_robot.q_0_ref;
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); quat_R_endeffector_py(R_init)]; % better to exact start in point

Q_pos = 1*diag([1,1e3,1e3,1e-5,1e5,1e5,1e5]);  % Weight for the position error in the cost function.
Q_m = 1e8;             % Weight for the manipulability error in the cost function.
Q_q = 1e5*diag(ones(n_red, 1));    % Weight for the deviaton of q_sol to q_d
Q_nl = 1e-1 * eye(n_red);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
q_d = [0.000, 0.781, 0, -0.467, 0.000, 1.248, 0.785]';

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

H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); quat_R_endeffector_py(H_d(1:3,1:3))];

param_inv_kin = struct;
param_inv_kin.xe0 = xe_d;
param_inv_kin.q_d = q_d(n_indices);
param_inv_kin.Q_pos = Q_pos;
param_inv_kin.Q_m = Q_m;
param_inv_kin.Q_q = Q_q;
param_inv_kin.Q_nl = Q_nl;
param_inv_kin.R_J_d = R_J_d; % desired collinearity matrix
param_inv_kin.K_J = K_J; % weight of collinearity matrix
param_inv_kin.tolerance = 1e-3;
param_inv_kin.random_q0_count = 100;

param_inv_kin.xpos_joint1 = zeros(7,1);
param_inv_kin.xpos_joint2 = zeros(7,1);
param_inv_kin.xpos_joint3 = zeros(7,1);
param_inv_kin.xpos_joint4 = zeros(7,1);
param_inv_kin.xpos_joint5 = zeros(7,1);
param_inv_kin.xpos_joint6 = zeros(7,1);
param_inv_kin.xpos_joint7 = zeros(7,1);

param_inv_kin.Q_joint1 = eye(7);
param_inv_kin.Q_joint2 = eye(7);
param_inv_kin.Q_joint3 = eye(7);
param_inv_kin.Q_joint4 = eye(7);
param_inv_kin.Q_joint5 = eye(7);
param_inv_kin.Q_joint6 = eye(7);
param_inv_kin.Q_joint7 = eye(7);

[q_target_red, ~] = inverse_kinematics(param_robot, param_inv_kin, 'not used');
q_target = q_0;
q_target(n_indices) = q_target_red;
H_target = hom_transform_endeffector_py(q_target);
R_target = H_target(1:3,1:3);
xeT = [H_target(1:3,4); quat_R_endeffector_py(R_target)];
% xeT(1) = xeT(1) + 0.2;

traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
traj_struct.pose = [xe0, xeT, xeT];
traj_struct.rotation = cat(3, R_init, R_target, R_target);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Singularity stretched arm out, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%% Trajectory 2: Stretch arm in
disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);
traj_struct = struct;
traj_struct.q_0 = q_target;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
traj_struct.pose = [xeT, xe0, xe0];
traj_struct.rotation = cat(3, R_target, R_init, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Singularity stretched arm in, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%% Trajectory 3: Wrist singularity 1, Polynomial
disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);
q_0 = param_robot.q_0_ref;
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); quat_R_endeffector_py(R_init)]; % better to exact start in point

Q_pos = 1*diag([1,1,1,1,1,1,1]);  % Weight for the position error in the cost function.
Q_m = 1e8;             % Weight for the manipulability error in the cost function.
Q_q = 1e3*diag(ones(n_red, 1));    % Weight for the deviaton of q_sol to q_d
Q_nl = 0 * eye(n_red);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
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

q_d = [0.686, -0.099, 0, -2.416, 0.339, 3.312, 2.606]';
H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); quat_R_endeffector_py(H_d(1:3,1:3))];

param_inv_kin = struct;
param_inv_kin.xe0 = xe_d;
param_inv_kin.q_d = q_d(n_indices);
param_inv_kin.Q_pos = Q_pos;
param_inv_kin.Q_m = Q_m;
param_inv_kin.Q_q = Q_q;
param_inv_kin.Q_nl = Q_nl;
param_inv_kin.K_J = K_J;
param_inv_kin.R_J_d = R_J_d;
param_inv_kin.tolerance = 1e-3;
param_inv_kin.random_q0_count = 100;

param_inv_kin.xpos_joint1 = zeros(7,1);
param_inv_kin.xpos_joint2 = zeros(7,1);
param_inv_kin.xpos_joint3 = zeros(7,1);
param_inv_kin.xpos_joint4 = zeros(7,1);
param_inv_kin.xpos_joint5 = zeros(7,1);
param_inv_kin.xpos_joint6 = zeros(7,1);
param_inv_kin.xpos_joint7 = zeros(7,1);

param_inv_kin.Q_joint1 = eye(7);
param_inv_kin.Q_joint2 = eye(7);
param_inv_kin.Q_joint3 = eye(7);
param_inv_kin.Q_joint4 = eye(7);
param_inv_kin.Q_joint5 = eye(7);
param_inv_kin.Q_joint6 = eye(7);
param_inv_kin.Q_joint7 = eye(7);

[q_target_red, ~] = inverse_kinematics(param_robot, param_inv_kin, 'not used');
q_target = q_0;
q_target(n_indices) = q_target_red;
H_target = hom_transform_endeffector_py(q_target);
R_target = H_target(1:3,1:3);
xeT = [H_target(1:3,4); quat_R_endeffector_py(R_target)];

spd = 0.2;

vec = zeros(6,3);
vec2 = zeros(6,3);

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

xe2 = [vec(1:3, 2); xeT(4:7)];
xe3 = [vec2(1:3, 2); xeT(4:7)];


traj_struct = struct;
traj_struct.N = 5;
traj_struct.q_0 = q_target;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = -ones(n,traj_struct.N);
traj_struct.pose = [xeT, xe2, xe2, xe3, xe3];
traj_struct.rotation = cat(3, R_target, R_target, R_target, R_target, R_target);
traj_struct.time = [0; 0.5; 2; 3; 10];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Wrist singularity 1, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%% Trajectory 4: Wrist singularity 1, Jointspace, Polynomial
disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);
q_0 = param_robot.q_0_ref;
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); quat_R_endeffector_py(R_init)]; % better to exact start in point

Q_pos = 1*diag([1,1,1,1,1,1,1]);  % Weight for the position error in the cost function.
Q_m = 1e8;             % Weight for the manipulability error in the cost function.
Q_q = 1e3*diag(ones(n_red, 1));    % Weight for the deviaton of q_sol to q_d
Q_nl = 0 * eye(n_red);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
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

q_d = [0.686, -0.099, 0, -2.416, 0.339, 3.312, 2.606]';
H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); quat_R_endeffector_py(H_d(1:3,1:3))];

param_inv_kin = struct;
param_inv_kin.xe0 = xe_d;
param_inv_kin.q_d = q_d(n_indices);
param_inv_kin.Q_pos = Q_pos;
param_inv_kin.Q_m = Q_m;
param_inv_kin.Q_q = Q_q;
param_inv_kin.Q_nl = Q_nl;
param_inv_kin.K_J = K_J;
param_inv_kin.R_J_d = R_J_d;
param_inv_kin.tolerance = 1e-3;
param_inv_kin.random_q0_count = 100;

param_inv_kin.xpos_joint1 = zeros(7,1);
param_inv_kin.xpos_joint2 = zeros(7,1);
param_inv_kin.xpos_joint3 = zeros(7,1);
param_inv_kin.xpos_joint4 = zeros(7,1);
param_inv_kin.xpos_joint5 = zeros(7,1);
param_inv_kin.xpos_joint6 = zeros(7,1);
param_inv_kin.xpos_joint7 = zeros(7,1);

param_inv_kin.Q_joint1 = eye(7);
param_inv_kin.Q_joint2 = eye(7);
param_inv_kin.Q_joint3 = eye(7);
param_inv_kin.Q_joint4 = eye(7);
param_inv_kin.Q_joint5 = eye(7);
param_inv_kin.Q_joint6 = eye(7);
param_inv_kin.Q_joint7 = eye(7);

[q_target_red, ~] = inverse_kinematics(param_robot, param_inv_kin, 'not used');
q_target = q_0;
q_target(n_indices) = q_target_red;
H_target = hom_transform_endeffector_py(q_target);
R_target = H_target(1:3,1:3);
xeT = [H_target(1:3,4); quat_R_endeffector_py(R_target)];

spd = 0.2;

vec = zeros(6,3);
vec2 = zeros(6,3);

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

xe2 = [vec(1:3, 2); xeT(4:7)];
xe3 = [vec2(1:3, 2); xeT(4:7)];


traj_struct = struct;
traj_struct.N = 5;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [q_0; q_target; q_target; q_0; q_0]';
traj_struct.pose = -ones(n,traj_struct.N);
traj_struct.rotation = cat(3, -eye(3), -eye(3), -eye(3), -eye(3), -eye(3));
traj_struct.time = [0; 4; 5; 9; 10];
traj_struct.traj_type = [traj_mode.polynomial_jointspace];
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Wrist singularity 1, Polynomial, Jointspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%% Trajectory 5: Shoulder - Wrist singularity 1, Polynomial
disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);
q_0 = param_robot.q_0_ref;
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); quat_R_endeffector_py(R_init)]; % better to exact start in point

Q_pos = 1*diag([1,1,1,1,1,1,1]);  % Weight for the position error in the cost function.
Q_m = 1e8;             % Weight for the manipulability error in the cost function.
Q_q = 1e3*diag(ones(n_red, 1));    % Weight for the deviaton of q_sol to q_d
Q_nl = 0 * eye(n_red);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
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

q_d = [0.317, -0.244, 0, -0.446, -1.579, 3.276, 1.945]';
H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); quat_R_endeffector_py(H_d(1:3,1:3))];

param_inv_kin = struct;
param_inv_kin.xe0 = xe_d;
param_inv_kin.q_d = q_d(n_indices);
param_inv_kin.Q_pos = Q_pos;
param_inv_kin.Q_m = Q_m;
param_inv_kin.Q_q = Q_q;
param_inv_kin.Q_nl = Q_nl;
param_inv_kin.K_J = K_J;
param_inv_kin.R_J_d = R_J_d;
param_inv_kin.tolerance = 1e-3;
param_inv_kin.random_q0_count = 100;

param_inv_kin.xpos_joint1 = zeros(7,1);
param_inv_kin.xpos_joint2 = zeros(7,1);
param_inv_kin.xpos_joint3 = zeros(7,1);
param_inv_kin.xpos_joint4 = zeros(7,1);
param_inv_kin.xpos_joint5 = zeros(7,1);
param_inv_kin.xpos_joint6 = zeros(7,1);
param_inv_kin.xpos_joint7 = zeros(7,1);

param_inv_kin.Q_joint1 = eye(7);
param_inv_kin.Q_joint2 = eye(7);
param_inv_kin.Q_joint3 = eye(7);
param_inv_kin.Q_joint4 = eye(7);
param_inv_kin.Q_joint5 = eye(7);
param_inv_kin.Q_joint6 = eye(7);
param_inv_kin.Q_joint7 = eye(7);

[q_target_red, ~] = inverse_kinematics(param_robot, param_inv_kin, 'not used');
q_target = q_0;
q_target(n_indices) = q_target_red;
H_target = hom_transform_endeffector_py(q_target);
R_target = H_target(1:3,1:3);
xeT = [H_target(1:3,4); quat_R_endeffector_py(R_target)];

xe2 = [xeT(1:3)-[0;0;0.3]; xeT(4:7)];


traj_struct = struct;
traj_struct.N = 5;
traj_struct.q_0 = q_target;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = -ones(n,traj_struct.N);
traj_struct.pose = [xeT, xe2, xe2, xeT, xeT];
traj_struct.rotation = cat(3, R_target, R_target, R_target, R_target, R_target);
traj_struct.time = [0; 3; 4; 7; 10];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Shoulder - Wrist singularity 1, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;


%% Trajectory 6: Shoulder Singularity, Polynomial
disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);
q_0 = [0.0059   -0.6074         0   -1.0201   -0.1695    1.4666    0.7390];
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); quat_R_endeffector_py(R_init)]; % better to exact start in point

q_d = [0.0156   -0.5211         0   -0.8741   -0.0393    1.4949    0.1117]';
H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); quat_R_endeffector_py(H_d(1:3,1:3))];
R_d = H_d(1:3,1:3);

traj_struct = struct;
traj_struct.N = 5;
traj_struct.q_0 = q_d;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = -ones(n,traj_struct.N);
traj_struct.pose = [xe_d, xe0, xe0, xe_d, xe_d];
traj_struct.rotation = cat(3, R_d, R_init, R_init, R_d, R_d);
traj_struct.time = [0; 3; 4; 7; 10];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Shoulder Sing 2, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;


%% Trajectory 7: Wrist Singularity, Polynomial
disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);
q_0 = [0.1264   -0.5106         0   -0.9332    1.4865    1.5759    1.6095];
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); quat_R_endeffector_py(R_init)]; % better to exact start in point

q_d = [-0.0160   -0.6184         0   -1.0897    0.8911    2.0872    1.1087]';
H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); quat_R_endeffector_py(H_d(1:3,1:3))];
R_d = H_d(1:3,1:3);

traj_struct = struct;
traj_struct.N = 5;
traj_struct.q_0 = q_d;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = -ones(n,traj_struct.N);
traj_struct.pose = [xe_d, xe0, xe0, xe_d, xe_d];
traj_struct.rotation = cat(3, R_d, R_init, R_init, R_d, R_d);
traj_struct.time = [0; 3; 4; 7; 10];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Wrist singularity 3, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;


%% Trajectory 8: Shoulder Singularity, Polynomial
disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);
q_0 = [-0.0268   -0.6276         0   -1.1201    1.0205    1.9086   -0.0059];
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); quat_R_endeffector_py(R_init)]; % better to exact start in point

q_d = [1.1650    0.6433         0   -1.0427    0.7211    3.3464    1.3652]';
H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); quat_R_endeffector_py(H_d(1:3,1:3))];
R_d = H_d(1:3,1:3);

traj_struct = struct;
traj_struct.N = 5;
traj_struct.q_0 = q_d;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = -ones(n,traj_struct.N);
traj_struct.pose = [xe_d, xe0, xe0, xe_d, xe_d];
traj_struct.rotation = cat(3, R_d, R_init, R_init, R_d, R_d);
traj_struct.time = [0; 3; 4; 7; 10];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Shoulder singularity 3, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%% Trajectory 9: Wrist Singularity, Polynomial
try
    single_sing_pose = readmatrix('./main_c/sorted_output_single.csv');
    idx1 = round(length(single_sing_pose)*rand(1));
    idx2 = round(length(single_sing_pose)*rand(1));
    q_0 = single_sing_pose(idx1, :);
    q_d = single_sing_pose(idx2, :);
catch
    q_0=[0.042506 -0.637644 0 -1.13312 0.96853 1.90296 -0.936423];
    q_d=[1.393712 -0.494812 0 -0.903915 1.501274 3.032111 1.63013];
end

disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);
%q_0 = [1.4617   -0.4724         0   -0.8640    1.5707    1.4862    1.7096];
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); quat_R_endeffector_py(R_init)]; % better to exact start in point

%q_d = [0.0115   -0.6668         0   -1.0631    0.0044    2.5219    0.0006]';
H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); quat_R_endeffector_py(H_d(1:3,1:3))];
R_d = H_d(1:3,1:3);

traj_struct = struct;
traj_struct.N = 5;
traj_struct.q_0 = q_d;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = -ones(n,traj_struct.N);
traj_struct.pose = [xe_d, xe0, xe0, xe_d, xe_d];
traj_struct.rotation = cat(3, R_d, R_init, R_init, R_d, R_d);
traj_struct.time = [0; 3; 4; 7; 10];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Wrist singularity 4, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%% Trajectory 10: Shoulder Singularity, Polynomial
q0=[1.021569 -0.597543 0 -1.071024 1.094256 2.142251 1.198465];
q1=[0.732774 -0.627773 0 -1.1205 1.027423 1.969581 0.854355];
q2=[7e-06 -0.631139 0 -1.117351 0.949181 2.109465 1.8e-05];
q3=[0.310517 -0.646585 0 -1.117622 0.719843 2.167447 0.743774];
q4=[0.160677 -0.623731 0 -1.10831 0.980397 2.045906 -0.016053];
q5=[1.223525 0.807631 0 -1.073598 1.237168 3.414369 1.435172];

% q0=[0.042506 -0.637644 0 -1.13312 0.96853 1.90296 -0.936423];
% q1=[1.393712 -0.494812 0 -0.903915 1.501274 3.032111 1.63013];
% q2=[1.328816 -0.583995 0 -1.066319 1.461657 1.613708 1.572034];
% q3=[0.996771 -0.607271 0 -1.079529 1.067266 2.496667 1.169019];
% q4=[1.44727 -0.511306 0 -0.866249 0.328108 1.491637 1.687238];
% q5=[0.949407 -0.622713 0 -1.102866 0.990681 2.315268 1.111184];

% zwei richtig lange drin
% q0=[0.002351 -0.624674 0 -1.046274 0.000791 1.594809 1.558001];
% q1=[1.318474 0.876235 0 -1.052913 1.361861 3.428177 1.542566];
% q2=[1.396151 -0.475007 0 -0.868637 1.520378 1.497545 1.64078];
% q3=[0.081853 -0.493264 0 -0.901775 1.50035 1.548162 0.768765];
% q4=[0.157365 -0.630526 0 -1.118806 0.977285 2.126835 0.178159];
% q5=[0.907199 -0.618487 0 -1.098646 0.989883 2.144418 1.087999];

% eine schone lange
% q0=[0.186041 -0.617382 0 -1.066982 0.670605 2.007633 1.22873];
% q1=[0.461971 -0.585031 0 -0.867125 0.39112 2.919606 0.997766];
% q2=[0.078568 -0.631227 0 -1.120268 0.976286 2.105079 0];
% q3=[0.27487 0.861499 0 -1.126202 0.064255 3.261212 1.52233];
% q4=[-0.02802 -0.628775 0 -1.119977 1.008873 2.03333 0];
% q5=[-0.059001 -0.649485 0 -1.130848 0.853284 2.37927 0];

% writs am ende
% q0=[0.955996 -0.596137 0 -1.061729 1.062432 2.402192 1.136429];
% q1=[-0.055166 -0.640127 0 -1.128991 0.921633 2.172856 0];
% q2=[1.424353 -0.55663 0 -0.933173 0.024384 1.495043 1.672911];
% q3=[0.123996 -0.638446 0 -1.151043 1.15903 1.680318 0.886154];
% q4=[1.406111 0.879822 0 -0.894674 1.43714 3.466062 1.646517];
% q5=[1.315911 -0.522832 0 -0.951909 1.355863 2.274959 1.562581];

% mit kleiner shoulder
% q0=[-0.009933 -0.613518 0 -1.090667 0.96188 1.82917 0];
% q1=[1.302954 -0.508471 0 -0.851247 0.00021 1.806214 0];
% q2=[1.156254 -0.590447 0 -0.992717 0.309804 1.973689 1.230717];
% q3=[0.655873 -0.642941 0 -1.150829 1.074435 2.036873 0.989054];
% q4=[1.118018 0.546898 0 -1.029919 0.89648 3.378361 1.314634];
% q5=[0.953543 -0.659691 0 -1.089352 0.306248 2.308665 1.1237];

% mit zwei wirklich schönen wrists
% q0=[1.108861 -0.583794 0 -1.05642 1.21086 1.697884 1.296404];
% q1=[1.268688 -0.831718 0 -1.509084 1.395416 2.193453 1.496371];
% q2=[1.1658 0.575133 0 -1.092335 1.162725 3.40599 1.367659];
% q3=[0.002286 -0.518169 0 -0.946901 1.478187 1.569784 1.59278];
% q4=[-0.118715 -0.647096 0 -1.141012 0.935909 2.246666 -0.084717];
% q5=[0.889749 -0.626443 0 -1.12314 1.099902 2.10575 1.06654];

% aehnlich zu vorher
% q0=[1.000342 -0.606696 0 -1.075555 1.078343 2.629348 1.173911];
% q1=[1.230009 0.828349 0 -1.137286 1.226867 3.405086 1.439463];
% q2=[1.232434 -0.668604 0 -1.217097 1.39176 2.155764 1.466212];
% q3=[0.105523 -0.632875 0 -1.1298 1.039899 2.054842 -1.059827];
% q4=[1.258925 -0.669717 0 -1.038616 0.358037 2.77123 1.476863];
% q5=[1.366694 -0.494639 0 -0.900858 1.423274 2.936039 0.951906];

disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);

H = hom_transform_endeffector_py(q0);
R0 = H(1:3,1:3);
x0 = [H(1:3,4); quat_R_endeffector_py(R0)];

H = hom_transform_endeffector_py(q1);
R1 = H(1:3,1:3);
x1 = [H(1:3,4); quat_R_endeffector_py(R1)];

H = hom_transform_endeffector_py(q2);
R2 = H(1:3,1:3);
x2 = [H(1:3,4); quat_R_endeffector_py(R2)];

H = hom_transform_endeffector_py(q3);
R3 = H(1:3,1:3);
x3 = [H(1:3,4); quat_R_endeffector_py(R3)];

H = hom_transform_endeffector_py(q4);
R4 = H(1:3,1:3);
x4 = [H(1:3,4); quat_R_endeffector_py(R4)];

H = hom_transform_endeffector_py(q5);
R5 = H(1:3,1:3);
x5 = [H(1:3,4); quat_R_endeffector_py(R5)];


traj_struct = struct;
traj_struct.N = 6;
traj_struct.q_0 = q0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = -ones(n,traj_struct.N);
traj_struct.pose = [x0, x1, x2, x3, x4, x5];
traj_struct.rotation = cat(3, R0, R1, R2, R3, R4, R5);
traj_struct.time = [0; 2; 4; 6; 8; 10];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Multipoint 1, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;


%% Trajectory 11: Shoulder Singularity, Polynomial
try
    single_sing_pose = readmatrix('./main_c/sorted_output_single.csv');
    N_row = length(single_sing_pose)
    q0 = single_sing_pose(round(N_row*rand(1)), :);
    q1 = single_sing_pose(round(N_row*rand(1)), :);
    q2 = single_sing_pose(round(N_row*rand(1)), :);
    q3 = single_sing_pose(round(N_row*rand(1)), :);
    q4 = single_sing_pose(round(N_row*rand(1)), :);
    q5 = single_sing_pose(round(N_row*rand(1)), :);
catch
    q0=[1.000342 -0.606696 0 -1.075555 1.078343 2.629348 1.173911];
    q1=[1.230009 0.828349 0 -1.137286 1.226867 3.405086 1.439463];
    q2=[1.232434 -0.668604 0 -1.217097 1.39176 2.155764 1.466212];
    q3=[0.105523 -0.632875 0 -1.1298 1.039899 2.054842 -1.059827];
    q4=[1.258925 -0.669717 0 -1.038616 0.358037 2.77123 1.476863];
    q5=[1.366694 -0.494639 0 -0.900858 1.423274 2.936039 0.951906];
end

fprintf('q0=%s;\n', mat2str(q0));
fprintf('q1=%s;\n', mat2str(q1));
fprintf('q2=%s;\n', mat2str(q2));
fprintf('q3=%s;\n', mat2str(q3));
fprintf('q4=%s;\n', mat2str(q4));
fprintf('q5=%s;\n', mat2str(q5));

disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);

H = hom_transform_endeffector_py(q0);
R0 = H(1:3,1:3);
x0 = [H(1:3,4); quat_R_endeffector_py(R0)];

H = hom_transform_endeffector_py(q1);
R1 = H(1:3,1:3);
x1 = [H(1:3,4); quat_R_endeffector_py(R1)];

H = hom_transform_endeffector_py(q2);
R2 = H(1:3,1:3);
x2 = [H(1:3,4); quat_R_endeffector_py(R2)];

H = hom_transform_endeffector_py(q3);
R3 = H(1:3,1:3);
x3 = [H(1:3,4); quat_R_endeffector_py(R3)];

H = hom_transform_endeffector_py(q4);
R4 = H(1:3,1:3);
x4 = [H(1:3,4); quat_R_endeffector_py(R4)];

H = hom_transform_endeffector_py(q5);
R5 = H(1:3,1:3);
x5 = [H(1:3,4); quat_R_endeffector_py(R5)];


traj_struct = struct;
traj_struct.N = 6;
traj_struct.q_0 = q0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = -ones(n,traj_struct.N);
traj_struct.pose = [x0, x1, x2, x3, x4, x5];
traj_struct.rotation = cat(3, R0, R1, R2, R3, R4, R5);
traj_struct.time = [0; 2; 4; 6; 8; 10];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Random Traj, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;


end