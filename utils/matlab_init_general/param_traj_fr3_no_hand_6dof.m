%% Calculate target positions
cnt = 1;

q_0 = param_robot.q_0_ref;
H_0 = hom_transform_endeffector_py(q_0);
xe0 = [H_0(1:3,4); rotm2quat_v4(H_0(1:3,1:3))]; % better to exact start in point

xeT = [xe0(1:3,1) + [0; 0; -0.15]; rotm2quat_v4( Rz(pi/4)*quat2rotm_v2(xe0(4:7)) )]; % rotm2quat (from matlab) is very precise but slow

%tests;
R_init = quat2rotm_v2(xe0(4:7));
R_target = quat2rotm_v2(xeT(4:7));

traj_cell = cell(1, 4);
% Trajectory 1: Ruhelage
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
traj_struct.pose = [xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_target, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_mode.equilibrium];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Wrist Singularity 1, Equilibrium, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

% Trajectory 2: Differential filter 5th order
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
traj_struct.pose = [xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_target, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_mode.differential_filter];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Wrist Singularity 1, Diff filt, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

% Trajectory 3: Polynomial 5th order
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
traj_struct.pose = [xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_target, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Wrist Singularity 1, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

% Trajectory 4: Sinus
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
traj_struct.pose = [xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_target, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_mode.sinus];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global, 'T', 5, 'phi', 0); % Param for sinus poly trajectory
traj_struct.name = 'Wrist Singularity 1, Sinus, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

% q_0 = [q1 q2 q4 q5 q6 q7]
q_0 = [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]'; % q3 is per default 0 (not used)
q_T = [0, -pi/4, 0, -3 * pi/4 + 0.5236, 0, pi/2, pi/4]'; % q3 is per default 0 (not used)
%H_0 = hom_transform_endeffector_py(q_0);
R_init = quat2rotm_v2(xe0(4:7));

% Trajectory 5: 2DOF q2, q4 joint space
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [q_0, q_T, q_0];
traj_struct.pose = [-1, -1, -1].*ones(m+1,1);
traj_struct.rotation = cat(3, R_init, R_init, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_mode.polynomial_jointspace];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global, 'T', 1, 'phi', 0); % Param for sinus poly trajectory
traj_struct.name = '2DOF Test Trajectory, Polynomial, joint space';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%% Trajectory 6: fr3 singularity...
q_0 = param_robot.q_0_ref;
H_0 = hom_transform_endeffector_py(q_0);
R_init = H_0(1:3,1:3);
xe0 = [H_0(1:3,4); rotm2quat_v4(R_init)]; % better to exact start in point

Q_pos = 1*diag([1,1e3,1e3,1e-5,1e5,1e5,1e5]);  % Weight for the position error in the cost function.
Q_m = 1e8;             % Weight for the manipulability error in the cost function.
Q_q = 1e5*diag([1 1 1 1 1 1]);    % Weight for the deviaton of q_sol to q_d
Q_nl = 1e-1 * eye(n_red);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
q_d = [0.000, 0.781, 0, -0.467, 0.000, 1.248, 0.785]';
H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];

[q_target_red, ~] = inverse_kinematics(param_robot, xe_d, q_d(n_indices), Q_pos, Q_m, Q_q, Q_nl, 1e-3, 100, 'not used');
q_target = q_0;
q_target(n_indices) = q_target_red;
H_target = hom_transform_endeffector_py(q_target);
R_target = H_target(1:3,1:3);
xeT = [H_target(1:3,4); rotm2quat_v4(R_target)];

traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
traj_struct.pose = [xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_target, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Singularity 2, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;