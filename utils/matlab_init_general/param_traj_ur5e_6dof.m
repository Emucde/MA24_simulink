% This file initializes trajectories for the UR5e robot in MATLAB.
% It defines several trajectories with different parameters and types,
% including polynomial and sinusoidal trajectories, as well as handling
% singularities in the robot's workspace.
%% TRAJECTORY 1: Polynomial 5. Order & WRIST SINGULARITY 1
cnt = 1;

q_d = [0.3636   -1.3788    0    0.1103    0.3635   -0.0042]';
xe0 = [0.3921; 0.3921; 0.5211; rotm2quat_v4(Ry(pi))];

options = optimoptions('fsolve', 'Algorithm', 'levenberg-marquardt', 'MaxFunctionEvaluations', 10000, 'MaxIterations', 10000);
q_0 = fsolve(@(q) kin_fun(xe0, q), q_d, options); % test if the function works
H_0 = hom_transform_endeffector_py(q_0);
xe0 = [H_0(1:3,4); rotm2quat_v4(H_0(1:3,1:3))]; % better to exact start in point
xeT = [xe0(1:3,1) + [0; 0; -0.5]; rotm2quat_v4( Rz(pi/4)*quat2rotm_v2(xe0(4:7)) )]; % rotm2quat (from matlab) is very precise but slow

%tests;
R_init = quat2rotm_v2(xe0(4:7));
R_target = quat2rotm_v2(xeT(4:7));

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

%% TRAJECTORY 2: Polynomial & ALL SINGULARITY 1: planning in workspace
Q_pos = 1e5*diag([1,1,1,1,1,1]);  % Weight for the position error in the cost function.
Q_m = 1e8;             % Weight for the manipulability error in the cost function.
Q_q = 1e8*diag([1,1,1,1,1,1]);    % Weight for the deviaton of q_sol to q_d
Q_nl = 1e-1 * eye(n);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
q_d = [-0.0349   -2.4613   -3.5569    2.2398    0.0350   -1.2488]';
H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];

[q_init, ~] = inverse_kinematics(param_robot, xe_d, q_d, Q_pos, Q_m, Q_q, Q_nl, 1e-3, 100, ct_ctrl_param);
H_init = hom_transform_endeffector_py(q_init);
xe_init = [H_init(1:3,4); rotm2quat_v4(H_init(1:3,1:3))]; % better to exact start in point

xe0 = [xe_init(1:3,1) + [0; -0.2; 0]; xe_init(4:7)];
xeT = [xe_init(1:3,1) + [0; 0.2; 0]; xe_init(4:7)];

R_init = H_init(1:3,1:3);
R_0 = quat2rotm_v2(xe0(4:7));
R_target = quat2rotm_v2(xeT(4:7));

traj_struct = struct;
traj_struct.q_0 = q_init;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [-1, -1, -1, -1].*ones(n,1);
traj_struct.pose = [xe_init, xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_0, R_target, R_0);
traj_struct.time = [0; T_sim/3; 2*T_sim/3; T_sim];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct.N = 4;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Allsingularity, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%% TRAJECTORY 3: Polynomial & Ellbow SINGULARITY 1: planning in workspace
Q_pos = 1e5*diag([1,1,1,1,1,1]);  % Weight for the position error in the cost function.
Q_m = 1e8;             % Weight for the manipulability error in the cost function.
Q_q = 1e5*diag([1,1,1,1,1,1]);    % Weight for the deviaton of q_sol to q_d
Q_nl = 1e-1 * eye(n);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
q_d = [-0.0182   -0.4209   -2.6141    4.9392    0.0094   -1.3736]';
H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];

[q_init, ~] = inverse_kinematics(param_robot, xe_d, q_d, Q_pos, Q_m, Q_q, Q_nl, 1e-3, 100, ct_ctrl_param);
H_init = hom_transform_endeffector_py(q_init);
xe_init = [H_init(1:3,4); rotm2quat_v4(H_init(1:3,1:3))]; % better to exact start in point

xe0 = [xe_init(1:3,1) + [0; -0.2; 0]; xe_init(4:7)];
xeT = [xe_init(1:3,1) + [0; 0.2; 0]; xe_init(4:7)];

R_init = H_init(1:3,1:3);
R_0 = quat2rotm_v2(xe0(4:7));
R_target = quat2rotm_v2(xeT(4:7));

traj_struct = struct;
traj_struct.q_0 = q_init;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [-1, -1, -1, -1].*ones(n,1);
traj_struct.pose = [xe_init, xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_0, R_target, R_0);
traj_struct.time = [0; T_sim/3; 2*T_sim/3; T_sim];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct.N = 4;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Ellbow Singularity 1, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TRAJECTORY 4: Polynomial & ELLBOW SINGULARITY: planning in workspace

Q_pos = 1e5*eye(m);  % Weight for the position error in the cost function.
Q_m = 0;             % Weight for the manipulability error in the cost function.
Q_q = diag([1,1,1e10,1,1,1]);    % Weight for the deviaton of q_sol to q_d
Q_nl = 1e-1 * eye(n);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
q_d = [0.3636   -1.3788    0    0.1103    0.3635   -0.0042]';
xe0 = [0.376; 0.376; 0.747; rotm2quat_v4(Ry(pi))];

[q_0, ~] = inverse_kinematics(param_robot, xe0, q_d, Q_pos, Q_m, Q_q, Q_nl,  1e-3, 100, ct_ctrl_param);
H_0 = hom_transform_endeffector_py(q_0);
xe0 = [H_0(1:3,4); rotm2quat_v4(H_0(1:3,1:3))]; % better to exact start in point

R_init = quat2rotm_v2(xe0(4:7));
R_target = R_init;

traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [-1, -1, -1, -1].*ones(n,1);
traj_struct.pose = [xe0, xe0 + [0; 0; 0.946-xe0(3); zeros(4,1)], xe0 - [0; 0; 0; zeros(4,1)], xe0 - [0; 0; 0; zeros(4,1)]];
traj_struct.rotation = cat(3, R_init, R_target, R_init, R_init);
traj_struct.time = [0; T_sim/4; 1/2*T_sim; T_sim];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct.N = 4;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'ELLBOW Singularity 2, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TRAJECTORY 5: Polynomial & ELLBOW SINGULARITY: planning in joint space

Q_pos = 1e5*eye(m);  % Weight for the position error in the cost function.
Q_m = 0;             % Weight for the manipulability error in the cost function.
Q_q = 1e10*diag([1,1,1,1,1,1]);    % Weight for the deviaton of q_sol to q_d
Q_nl = 1e-1 * eye(n);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
q_d = [-5.9399    5.2908    0   -5.2936   -5.9398   -6.2802]';
xe0 = [0.3759; 0.3762; 0.7472; rotm2quat_v4(Ry(pi))];

q_0 = inverse_kinematics(param_robot, xe0, q_d, Q_pos, Q_m, Q_q, Q_nl,  1e-2, 100, ct_ctrl_param);

xeT = xe0 + [0; 0; 0.9462-xe0(3); zeros(4,1)];

%Q_pos = 1e5*diag([1e1,1e1,1e-3,1,1,1]);
q_d = [-5.9398    5.2914    0    4.1332    5.9398    3.1417]';
Q_q = 1e10*diag([1,1,1,1,1,1]);
q_T = inverse_kinematics(param_robot, xeT, q_d, Q_pos, Q_m, Q_q, Q_nl,  1e-2, 100, ct_ctrl_param);

traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [q_0, q_T, q_0, q_0];
traj_struct.pose = [-1, -1, -1, -1].*ones(7,1);
traj_struct.rotation = cat(3, eye(3), eye(3), eye(3), eye(3));
traj_struct.time = [0; T_sim/4; 1/2*T_sim; T_sim];
traj_struct.traj_type = [traj_mode.polynomial_jointspace];
traj_struct.N = 4;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', 0); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'ELLBOW Singularity 2, Polynomial, joint space';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TRAJECTORY 6: Differential Filter & SHOULDER SINGULARITY    %%%%%%%%%%

Q_pos = 1e3*eye(m);  % Weight for the position error in the cost function.
Q_m = 1e10;             % Weight for the manipulability error in the cost function.
Q_q = diag([1,1,1e3,1,1,1]);    % Weight for the deviaton of q_sol to q_d
Q_nl = 1e-1 * eye(n);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
q_d = [-1.944, -0.710, -1.500, -2.028, 3.852, -0.977]';
H_d = hom_transform_endeffector_py(q_d);
xe0 = [0; 0; 0.8576; rotm2quat_v4(H_d(1:3,1:3))];

[q_0, ~] = inverse_kinematics(param_robot, xe0, q_d, Q_pos, Q_m, Q_q, Q_nl,  1e-2, 100, ct_ctrl_param);
H_0 = hom_transform_endeffector_py(q_0);
xe0 = [H_0(1:3,4); rotm2quat_v4(H_0(1:3,1:3))]; % better to exact start in point
xeT = [xe0(1:3,1) + [-0.49; 0.49; 0]; rotm2quat_v4( Rz(pi/4)*quat2rotm_v2(xe0(4:7)) )]; % rotm2quat (from matlab) is very precise but slow

R_init = quat2rotm_v2(xe0(4:7));
%R_target = quat2rotm_v2(xeT(4:7));
R_target = R_init;
%xeT = xe0;

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
traj_struct.name = 'Shoulder Singularity, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

%% TRAJECTORY 7: Polynomial & ALL SINGULARITY 1: planning in workspace
Q_pos = 1e5*diag([1,1,1,1,1,1]);  % Weight for the position error in the cost function.
Q_m = 1e8;             % Weight for the manipulability error in the cost function.
Q_q = 1e8*diag([1,1,1,1,1,1]);    % Weight for the deviaton of q_sol to q_d
Q_nl = 1e-1 * eye(n);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
q_d = [1e-3  -0.44  -2.45  -0.251  1e-3  1e-3]';
% q_d = [1e-3, -0.754, -1.76, -0.628, 1e-3, 1e-3]';
H_d = hom_transform_endeffector_py(q_d);
xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];

[q_init, ~] = inverse_kinematics(param_robot, xe_d, q_d, Q_pos, Q_m, Q_q, Q_nl, 1e-3, 100, ct_ctrl_param);
H_init = hom_transform_endeffector_py(q_init);
xe_init = [H_init(1:3,4); rotm2quat_v4(H_init(1:3,1:3))]; % better to exact start in point

xe0 = [xe_init(1:3,1) + [0; -0.5; 0]; xe_init(4:7)];
xeT = [xe_init(1:3,1) + [0; 0.5; 0]; xe_init(4:7)];

R_init = H_init(1:3,1:3);
R_0 = quat2rotm_v2(xe0(4:7));
R_target = quat2rotm_v2(xeT(4:7));

traj_struct = struct;
traj_struct.q_0 = q_init;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = [-1, -1, -1, -1].*ones(n,1);
traj_struct.pose = [xe_init, xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_0, R_target, R_0);
traj_struct.time = [0; T_sim/3; 2*T_sim/3; T_sim];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct.N = 4;
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Shoulder Sing 2, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

function out = kin_fun(xe, q)
    H = hom_transform_endeffector_py(q);
    
    RR = H(1:3,1:3)*quat2rotm_v2(xe(4:7))';
    p_err = H(1:3,4) - xe(1:3);
    q_err = rotm2quat_v4(RR);
    r_err = q_err(2:4);
    %r_err = [RR(3,2) - RR(2,3); RR(1,3) - RR(3,1); RR(2,1) - RR(1,2)];
    out = r_err'*r_err + p_err'*p_err;

    % He = [quat2rotm_v2(xe(4:7)), xe(1:3); 0 0 0 1];
    % out = sum((H - He).^2, 'all');
end










%% OLD DATA
%{
traj_cell = cell(1, 4);
% Trajectory 1: Ruhelage
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.pose = [xe0, xeT, xeT];
traj_struct.rotation = cat(3, R_init, R_target, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_mode.equilibrium];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global); % default settings not used [TODO: Mit extra index arbeiten]tructs ge
traj_struct = create_param_sin_poly(traj_struct, param_global); % default settings not used
traj_struct.name = '1: stabilize equilibrium';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

% Trajectory 3: Polynomial 5th order
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.pose = [xe0, xeT, xe0];
traj_struct.rotation = cat(3, R_init, R_target, R_init);
traj_struct.time = [0; T_sim/2; T_sim];
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct.N = 3;
traj_struct = create_param_diff_filter(traj_struct, param_global); % Param differential filter 5th order trajectory
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = '3: 5th order polynomial';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

% Trajectory 4:
traj_struct = struct;
traj_struct.q_0 = q_0;
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.pose = [xe0, xeT];
traj_struct.rotation = cat(3, R_init, R_target);
traj_struct.time = [0; 2];
traj_struct.traj_type = [traj_mode.sinus];
traj_struct.N = 2;
traj_struct = create_param_diff_filter(traj_struct, param_global); % Param differential filter 5th order trajectory
traj_struct = create_param_sin_poly(traj_struct, param_global, 'T', 2, 'phi', 0); % Param for sinus poly trajectory
traj_struct.name = '4: smooth sinus';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;
%}