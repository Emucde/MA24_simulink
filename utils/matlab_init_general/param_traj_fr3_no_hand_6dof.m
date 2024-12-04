%% Calculate target positions
cnt = 1;
random_traj = false;

if(random_traj)
    %% Trajectory 11: ellbow sing, Workspace, Polynomial
    q_0 = param_robot.q_0_ref';
    q_0(n_indices) = rand(6,1);
    q_0 = min(max(q_0, param_robot.q_limit_lower*0.5), param_robot.q_limit_upper*0.5);
    H_0 = hom_transform_endeffector_py(q_0);
    R_init = H_0(1:3,1:3);
    xe0 = [H_0(1:3,4); rotm2quat_v4(R_init)]; % better to exact start in point
    
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
    xe_d = [H(1:3,4); rotm2quat_v4(H(1:3,1:3))];
    
    
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
    xeT = [H_target(1:3,4); rotm2quat_v4(R_target)];
    
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
    traj_struct.time = [0; 3; 4; 9; 10];
    traj_struct.traj_type = [traj_mode.polynomial];
    traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
    traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
    traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
    traj_struct.name = 'test sing 1 init point, Polynomial, Jointspace';
    traj_cell{cnt} = traj_struct;
    cnt = cnt+1;
else

%% Trajectory 1: Stretch arm out
q_0 = param_robot.q_0_ref;
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); rotm2quat_v4(R_init)]; % better to exact start in point

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
xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];

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
xeT = [H_target(1:3,4); rotm2quat_v4(R_target)];
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
q_0 = param_robot.q_0_ref;
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); rotm2quat_v4(R_init)]; % better to exact start in point

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
xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];

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
xeT = [H_target(1:3,4); rotm2quat_v4(R_target)];

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
q_0 = param_robot.q_0_ref;
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); rotm2quat_v4(R_init)]; % better to exact start in point

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
xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];

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
xeT = [H_target(1:3,4); rotm2quat_v4(R_target)];

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
q_0 = param_robot.q_0_ref;
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); rotm2quat_v4(R_init)]; % better to exact start in point

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
xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];

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
xeT = [H_target(1:3,4); rotm2quat_v4(R_target)];

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
end