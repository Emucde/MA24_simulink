%% Calculate target positions
cnt = 1;

% q_0 = param_robot.q_0_ref;
% H_0 = hom_transform_endeffector_py(q_0);
% xe0 = [H_0(1:3,4); rotm2quat_v4(H_0(1:3,1:3))]; % better to exact start in point
% 
% xeT = [xe0(1:3,1) + [0; 0; -0.15]; rotm2quat_v4( Rz(pi/4)*quat2rotm_v2(xe0(4:7)) )]; % rotm2quat (from matlab) is very precise but slow
% 
% %tests;
% R_init = quat2rotm_v2(xe0(4:7));
% R_target = quat2rotm_v2(xeT(4:7));
% 
% traj_cell = cell(1, 4);
% % Trajectory 1: Ruhelage
% traj_struct = struct;
% traj_struct.q_0 = q_0;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
% traj_struct.pose = [xe0, xeT, xe0];
% traj_struct.rotation = cat(3, R_init, R_target, R_init);
% traj_struct.time = [0; T_sim/2; T_sim];
% traj_struct.traj_type = [traj_mode.equilibrium];
% traj_struct.N = 3;
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
% traj_struct.name = 'Wrist Singularity 1, Equilibrium, Workspace';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;
% 
% % Trajectory 2: Differential filter 5th order
% traj_struct = struct;
% traj_struct.q_0 = q_0;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
% traj_struct.pose = [xe0, xeT, xe0];
% traj_struct.rotation = cat(3, R_init, R_target, R_init);
% traj_struct.time = [0; T_sim/2; T_sim];
% traj_struct.traj_type = [traj_mode.differential_filter];
% traj_struct.N = 3;
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
% traj_struct.name = 'Wrist Singularity 1, Diff filt, Workspace';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;
% 
% % Trajectory 3: Polynomial 5th order
% traj_struct = struct;
% traj_struct.q_0 = q_0;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
% traj_struct.pose = [xe0, xeT, xe0];
% traj_struct.rotation = cat(3, R_init, R_target, R_init);
% traj_struct.time = [0; T_sim/2; T_sim];
% traj_struct.traj_type = [traj_mode.polynomial];
% traj_struct.N = 3;
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
% traj_struct.name = 'Wrist Singularity 1, Polynomial, Workspace';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;
% 
% % Trajectory 4: Sinus
% traj_struct = struct;
% traj_struct.q_0 = q_0;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
% traj_struct.pose = [xe0, xeT, xe0];
% traj_struct.rotation = cat(3, R_init, R_target, R_init);
% traj_struct.time = [0; T_sim/2; T_sim];
% traj_struct.traj_type = [traj_mode.sinus];
% traj_struct.N = 3;
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global, 'T', 5, 'phi', 0); % Param for sinus poly trajectory
% traj_struct.name = 'Wrist Singularity 1, Sinus, Workspace';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;
% 
% % q_0 = [q1 q2 q4 q5 q6 q7]
% q_0 = [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]'; % q3 is per default 0 (not used)
% q_T = [0, -pi/4, 0, -3 * pi/4 + 0.5236, 0, pi/2, pi/4]'; % q3 is per default 0 (not used)
% %H_0 = hom_transform_endeffector_py(q_0);
% R_init = quat2rotm_v2(xe0(4:7));
% 
% % Trajectory 5: 2DOF q2, q4 joint space
% traj_struct = struct;
% traj_struct.q_0 = q_0;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = [q_0, q_T, q_0];
% traj_struct.pose = [-1, -1, -1].*ones(m+1,1);
% traj_struct.rotation = cat(3, R_init, R_init, R_init);
% traj_struct.time = [0; T_sim/2; T_sim];
% traj_struct.traj_type = [traj_mode.polynomial_jointspace];
% traj_struct.N = 3;
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global, 'T', 1, 'phi', 0); % Param for sinus poly trajectory
% traj_struct.name = '2DOF Test Trajectory, Polynomial, joint space';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;
% 
% %% Trajectory 6: Stretch arm out
% q_0 = param_robot.q_0_ref;
% H_0 = hom_transform_endeffector_py(q_0);
% R_init = H_0(1:3,1:3);
% xe0 = [H_0(1:3,4); rotm2quat_v4(R_init)]; % better to exact start in point
% 
% Q_pos = 1*diag([1,1e3,1e3,1e-5,1e5,1e5,1e5]);  % Weight for the position error in the cost function.
% Q_m = 1e8;             % Weight for the manipulability error in the cost function.
% Q_q = 1e5*diag(ones(n_red, 1));    % Weight for the deviaton of q_sol to q_d
% Q_nl = 1e-1 * eye(n_red);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
% q_d = [0.000, 0.781, 0, -0.467, 0.000, 1.248, 0.785]';
% 
% R_J_d = struct( ...
%     'r21', 0, ...
%     'r31', 0, 'r32', 0, ...
%     'r41', 0, 'r42', 0, 'r43', 0, ...
%     'r51', 0, 'r52', 0, 'r53', 0, 'r54', 0, ...
%     'r61', 0, 'r62', 0, 'r63', 0, 'r64', 0, 'r65', 0 ...
% );
% 
% K_J = struct(... 
%     'k21', 0, ...
%     'k31', 0, 'k32', 0, ...
%     'k41', 0, 'k42', 0, 'k43', 0, ...
%     'k51', 0, 'k52', 0, 'k53', 0, 'k54', 0, ...
%     'k61', 0, 'k62', 0, 'k63', 0, 'k64', 0, 'k65', 0 ...
% );
% 
% H_d = hom_transform_endeffector_py(q_d);
% xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];
% 
% param_inv_kin = struct;
% param_inv_kin.xe0 = xe_d;
% param_inv_kin.q_d = q_d(n_indices);
% param_inv_kin.Q_pos = Q_pos;
% param_inv_kin.Q_m = Q_m;
% param_inv_kin.Q_q = Q_q;
% param_inv_kin.Q_nl = Q_nl;
% param_inv_kin.R_J_d = R_J_d; % desired collinearity matrix
% param_inv_kin.K_J = K_J; % weight of collinearity matrix
% param_inv_kin.tolerance = 1e-3;
% param_inv_kin.random_q0_count = 100;
% 
% param_inv_kin.xpos_joint1 = zeros(7,1);
% param_inv_kin.xpos_joint2 = zeros(7,1);
% param_inv_kin.xpos_joint3 = zeros(7,1);
% param_inv_kin.xpos_joint4 = zeros(7,1);
% param_inv_kin.xpos_joint5 = zeros(7,1);
% param_inv_kin.xpos_joint6 = zeros(7,1);
% param_inv_kin.xpos_joint7 = zeros(7,1);
% 
% param_inv_kin.Q_joint1 = eye(7);
% param_inv_kin.Q_joint2 = eye(7);
% param_inv_kin.Q_joint3 = eye(7);
% param_inv_kin.Q_joint4 = eye(7);
% param_inv_kin.Q_joint5 = eye(7);
% param_inv_kin.Q_joint6 = eye(7);
% param_inv_kin.Q_joint7 = eye(7);
% 
% [q_target_red, ~] = inverse_kinematics(param_robot, param_inv_kin, 'not used');
% q_target = q_0;
% q_target(n_indices) = q_target_red;
% H_target = hom_transform_endeffector_py(q_target);
% R_target = H_target(1:3,1:3);
% xeT = [H_target(1:3,4); rotm2quat_v4(R_target)];
% % xeT(1) = xeT(1) + 0.2;
% 
% traj_struct = struct;
% traj_struct.q_0 = q_0;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
% traj_struct.pose = [xe0, xeT, xeT];
% traj_struct.rotation = cat(3, R_init, R_target, R_target);
% traj_struct.time = [0; T_sim/2; T_sim];
% traj_struct.traj_type = [traj_mode.polynomial];
% traj_struct.N = 3;
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
% traj_struct.name = 'Singularity stretched arm out, Polynomial, Workspace';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;
% 
% %% Trajectory 7: Stretch arm in
% traj_struct = struct;
% traj_struct.q_0 = q_target;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
% traj_struct.pose = [xeT, xe0, xe0];
% traj_struct.rotation = cat(3, R_target, R_init, R_init);
% traj_struct.time = [0; T_sim/2; T_sim];
% traj_struct.traj_type = [traj_mode.polynomial];
% traj_struct.N = 3;
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
% traj_struct.name = 'Singularity stretched arm in, Polynomial, Workspace';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;
% 
% %% Trajectory 8: Stretch arm out, other dir, Polynomial
% q_0 = param_robot.q_0_ref;
% H_0 = hom_transform_endeffector_py(q_0);
% R_init = H_0(1:3,1:3);
% xe0 = [H_0(1:3,4); rotm2quat_v4(R_init)]; % better to exact start in point
% 
% Q_pos = 1*diag([1e-10,1e-10,1e5,1e-10,1e-10,1e-10,1e-10]);  % Weight for the position error in the cost function.
% Q_m = 1e8;             % Weight for the manipulability error in the cost function.
% Q_q = 1e3*diag(ones(n_red, 1));    % Weight for the deviaton of q_sol to q_d
% Q_nl = 0 * eye(n_red);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
% R_J_d = struct( ...
%     'r21', 0, ...
%     'r31', 0, 'r32', 0, ...
%     'r41', 0, 'r42', 0, 'r43', 0, ...
%     'r51', 0, 'r52', 0, 'r53', 0, 'r54', 0, ...
%     'r61', 0, 'r62', 0, 'r63', 0, 'r64', 0, 'r65', 0 ...
% );
% 
% K_J = struct(... 
%     'k21', 0, ...
%     'k31', 0, 'k32', 0, ...
%     'k41', 0, 'k42', 0, 'k43', 0, ...
%     'k51', 0, 'k52', 0, 'k53', 0, 'k54', 0, ...
%     'k61', 0, 'k62', 0, 'k63', 0, 'k64', 0, 'k65', 0 ...
% );
% 
% q_d = [0.00437857013686173, -1.46026870497488, 0, -0.526837270635818, -0.00626806216488016, 3.24855497497840, 0.211977023736380]';
% H_d = hom_transform_endeffector_py(q_d);
% %xe_d = [H_0(1:3,4) + [-0.5;0;0]; rotm2quat_v4(H_d(1:3,1:3))];
% xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];
% 
% param_inv_kin = struct;
% param_inv_kin.xe0 = xe_d;
% param_inv_kin.q_d = q_d(n_indices);
% param_inv_kin.Q_pos = Q_pos;
% param_inv_kin.Q_m = Q_m;
% param_inv_kin.Q_q = Q_q;
% param_inv_kin.Q_nl = Q_nl;
% param_inv_kin.K_J = K_J;
% param_inv_kin.R_J_d = R_J_d;
% param_inv_kin.tolerance = 1e-3;
% param_inv_kin.random_q0_count = 100;
% 
% param_inv_kin.xpos_joint1 = zeros(7,1);
% param_inv_kin.xpos_joint2 = zeros(7,1);
% param_inv_kin.xpos_joint3 = zeros(7,1);
% param_inv_kin.xpos_joint4 = zeros(7,1);
% param_inv_kin.xpos_joint5 = zeros(7,1);
% param_inv_kin.xpos_joint6 = zeros(7,1);
% param_inv_kin.xpos_joint7 = zeros(7,1);
% 
% param_inv_kin.Q_joint1 = eye(7);
% param_inv_kin.Q_joint2 = eye(7);
% param_inv_kin.Q_joint3 = eye(7);
% param_inv_kin.Q_joint4 = eye(7);
% param_inv_kin.Q_joint5 = eye(7);
% param_inv_kin.Q_joint6 = eye(7);
% param_inv_kin.Q_joint7 = eye(7);
% 
% [q_target_red, ~] = inverse_kinematics(param_robot, param_inv_kin, 'not used');
% q_target = q_0;
% q_target(n_indices) = q_target_red;
% H_target = hom_transform_endeffector_py(q_target);
% R_target = H_target(1:3,1:3);
% xeT = [H_target(1:3,4); rotm2quat_v4(R_target)];
% % xeT(1) = xeT(1) + 0.2;
% 
% traj_struct = struct;
% traj_struct.q_0 = q_0;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
% traj_struct.pose = [xe0, xeT, xeT];
% traj_struct.rotation = cat(3, R_init, R_target, R_target);
% traj_struct.time = [0; T_sim/2; T_sim];
% traj_struct.traj_type = [traj_mode.polynomial];
% traj_struct.N = 3;
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
% traj_struct.name = 'Stretch arm out, other dir, Polynomial, Workspace';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;
% 
% %% Trajectory 9: Stretch arm in, other dir, Polynomial
% 
% traj_struct = struct;
% traj_struct.q_0 = q_target;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
% traj_struct.pose = [xeT, xe0, xe0];
% traj_struct.rotation = cat(3, R_init, R_target, R_target);
% traj_struct.time = [0; T_sim/2; T_sim];
% traj_struct.traj_type = [traj_mode.polynomial];
% traj_struct.N = 3;
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
% traj_struct.name = 'Stretch arm in, other dir, Polynomial, Workspace';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;
% 
% %% Trajectory 10: Joint Limit singularity, Workspace, Polynomial
% q_0 = param_robot.q_0_ref;
% H_0 = hom_transform_endeffector_py(q_0);
% R_init = H_0(1:3,1:3);
% xe0 = [H_0(1:3,4); rotm2quat_v4(R_init)]; % better to exact start in point
% 
% Q_pos = 1e5*diag([1e3,1e3,1e-5,1,1e5,1e5,1e5]);  % Weight for the position error in the cost function.
% Q_m = 1e8;             % Weight for the manipulability error in the cost function.
% Q_q = 1e-5*diag(ones(n_red, 1));    % Weight for the deviaton of q_sol to q_d
% Q_nl = 0 * eye(n_red);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
% R_J_d = struct( ...
%     'r21', 0, ...
%     'r31', 0, 'r32', 0, ...
%     'r41', 0, 'r42', 0, 'r43', 0, ...
%     'r51', 0, 'r52', 0, 'r53', 0, 'r54', 0, ...
%     'r61', 0, 'r62', 0, 'r63', 0, 'r64', 0, 'r65', 0 ...
% );
% 
% K_J = struct(... 
%     'k21', 0, ...
%     'k31', 0, 'k32', 0, ...
%     'k41', 0, 'k42', 0, 'k43', 0, ...
%     'k51', 0, 'k52', 0, 'k53', 0, 'k54', 0, ...
%     'k61', 0, 'k62', 0, 'k63', 0, 'k64', 0, 'k65', 0 ...
% );
% 
% q_d = [0.177, 0.831, 0, -2.311, 2.480, 3.142, -1.517]';
% H_d = hom_transform_endeffector_py(q_d);
% xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];
% 
% param_inv_kin = struct;
% param_inv_kin.xe0 = xe_d;
% param_inv_kin.q_d = q_d(n_indices);
% param_inv_kin.Q_pos = Q_pos;
% param_inv_kin.Q_m = Q_m;
% param_inv_kin.Q_q = Q_q;
% param_inv_kin.Q_nl = Q_nl;
% param_inv_kin.K_J = K_J;
% param_inv_kin.R_J_d = R_J_d;
% param_inv_kin.tolerance = 1e-3;
% param_inv_kin.random_q0_count = 100;
% 
% param_inv_kin.xpos_joint1 = zeros(7,1);
% param_inv_kin.xpos_joint2 = zeros(7,1);
% param_inv_kin.xpos_joint3 = zeros(7,1);
% param_inv_kin.xpos_joint4 = zeros(7,1);
% param_inv_kin.xpos_joint5 = zeros(7,1);
% param_inv_kin.xpos_joint6 = zeros(7,1);
% param_inv_kin.xpos_joint7 = zeros(7,1);
% 
% param_inv_kin.Q_joint1 = eye(7);
% param_inv_kin.Q_joint2 = eye(7);
% param_inv_kin.Q_joint3 = eye(7);
% param_inv_kin.Q_joint4 = eye(7);
% param_inv_kin.Q_joint5 = eye(7);
% param_inv_kin.Q_joint6 = eye(7);
% param_inv_kin.Q_joint7 = eye(7);
% 
% [q_target_red, ~] = inverse_kinematics(param_robot, param_inv_kin, 'not used');
% q_target = q_0;
% q_target(n_indices) = q_target_red;
% H_target = hom_transform_endeffector_py(q_target);
% R_target = H_target(1:3,1:3);
% xeT = [H_target(1:3,4); rotm2quat_v4(R_target)];
% % xeT(1) = xeT(1) + 0.2;
% 
% traj_struct = struct;
% traj_struct.q_0 = q_0;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = [-1, -1, -1].*ones(n,1);
% traj_struct.pose = [xe0, xeT, xe0];
% traj_struct.rotation = cat(3, R_init, R_target, R_init);
% traj_struct.time = [0; T_sim/2; T_sim];
% traj_struct.traj_type = [traj_mode.polynomial];
% traj_struct.N = 3;
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
% traj_struct.name = 'Joint limit singularity, Polynomial, Workspace';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;

% %% Trajectory 11: ellbow sing, Workspace, Polynomial
% q_0 = param_robot.q_0_ref;
% H_0 = hom_transform_endeffector_py(q_0);
% R_init = H_0(1:3,1:3);
% xe0 = [H_0(1:3,4); rotm2quat_v4(R_init)]; % better to exact start in point
% 
% q_d = [0;-1.51; 0; -2.44; -pi/2; pi/2; 0];
% H_d = hom_transform_endeffector_py(q_d);
% H2 = hom_transform_joint_2_py(q_d);
% H7 = hom_transform_joint_7_py(q_d);
% p_d = H2(1:3, 4);
% %R_d = Rz(-pi/4)*Rx(-pi);
% R_d = H2(1:3,1:3);
% %xe_d = [H_0(1:3,4) + [-0.4;0;0]; rotm2quat_v4(H_d(1:3,1:3))];
% xe_d = [p_d; rotm2quat_v4(R_d)];
% xe_d2 = [p_d+[0;0;0.2]; rotm2quat_v4(R_d*Rx(pi))]; % show in -z of H2
% %xe_d = [H_d(1:3,4); rotm2quat_v4(H_d(1:3,1:3))];
% 
% Q_pos = 0*diag([1;1;1;1e2;1e5;1e5;1e5]);  % Weight for the position error in the cost function.
% Q_m = 1e-5;             % Weight for the manipulability error in the cost function.
% Q_q = 1e-10*diag([1;1;1;1;1;1]);    % Weight for the deviaton of q_sol to q_d
% Q_nl = 0 * eye(n_red);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
% R_J_d = struct( ...
%     'r21', 0, ...
%     'r31', 0, 'r32', 0, ...
%     'r41', 0, 'r42', 0, 'r43', 0, ...
%     'r51', 0, 'r52', 0, 'r53', 0, 'r54', 0, ...
%     'r61', 0, 'r62', 0, 'r63', 0, 'r64', 0, 'r65', 0 ...
% );
% 
% K_J = struct(... 
%     'k21', 0, ...
%     'k31', 0, 'k32', 0, ...
%     'k41', 0, 'k42', 0, 'k43', 0, ...
%     'k51', 0, 'k52', 0, 'k53', 0, 'k54', 0, ...
%     'k61', 0, 'k62', 0, 'k63', 0, 'k64', 0, 'k65', 0 ...
% );
% 
% param_inv_kin = struct;
% param_inv_kin.xe0 = xe_d;
% param_inv_kin.q_d = q_d(n_indices);
% param_inv_kin.Q_pos = Q_pos;
% param_inv_kin.Q_m = Q_m;
% param_inv_kin.Q_q = Q_q;
% param_inv_kin.Q_nl = Q_nl;
% param_inv_kin.K_J = K_J;
% param_inv_kin.R_J_d = R_J_d;
% param_inv_kin.tolerance = 1e-3;
% param_inv_kin.random_q0_count = 100;
% 
% param_inv_kin.xpos_joint1 = zeros(7,1);
% param_inv_kin.xpos_joint2 = xe_d;
% param_inv_kin.xpos_joint3 = zeros(7,1);
% param_inv_kin.xpos_joint4 = zeros(7,1);
% param_inv_kin.xpos_joint5 = zeros(7,1);
% param_inv_kin.xpos_joint6 = zeros(7,1);
% param_inv_kin.xpos_joint7 = xe_d2;
% 
% param_inv_kin.Q_joint1 = zeros(7);
% param_inv_kin.Q_joint2 = 1e8*diag([1e2;1e2;1e-10;0;1;1;1]);
% param_inv_kin.Q_joint3 = zeros(7);
% param_inv_kin.Q_joint4 = zeros(7);
% param_inv_kin.Q_joint5 = zeros(7);
% param_inv_kin.Q_joint6 = zeros(7);
% param_inv_kin.Q_joint7 = 1e8*diag([1e2;1e2;1e-10;0;1;1;1]);
% 
% [q_target_red, ~] = inverse_kinematics(param_robot, param_inv_kin, 'not used');
% q_target = q_0;
% q_target(n_indices) = q_target_red;
% 
% H2_target = hom_transform_joint_2_py(q_target)
% H7_target = hom_transform_joint_7_py(q_target)
% x2T = [H2_target(1:3,4); rotm2quat_v4(H2_target(1:3,1:3))];
% x7T = [H7_target(1:3,4); rotm2quat_v4(H7_target(1:3,1:3))];
% 
% 
% H_target = hom_transform_endeffector_py(q_target);
% R_target = H_target(1:3,1:3);
% xeT = [H_target(1:3,4); rotm2quat_v4(R_target)];
% % xeT(1) = xeT(1) + 0.2;
% 
% pT = H_target * [0;0;0.01;1];
% pT2 = H_target * [0;0;-0.4;1];
% 
% xe1 = xe0;
% xe1(3) = xeT(3);
% xe1(4:7) = xeT(4:7);
% 
% xe2 = [pT(1:3);xeT(4:7)];
% xe3 = [pT2(1:3);xeT(4:7)];
% 
% 
% traj_struct = struct;
% traj_struct.N = 3;
% traj_struct.q_0 = q_0;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = [q_0; q_target; q_target]';%-ones(n,traj_struct.N)
% %traj_struct.pose = [xe0, xeT, [pT(1:3);xeT(4:7)], [pT2(1:3);xeT(4:7)]];
% traj_struct.pose = [xe0, xeT, xeT];
% traj_struct.rotation = cat(3, R_init, R_target, R_target);
% traj_struct.time = [0; T_sim-1; T_sim];
% traj_struct.traj_type = [traj_mode.polynomial_jointspace];
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
% traj_struct.name = 'wrist sing 1 init point, Polynomial, Jointspace';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;
% 
% traj_struct = struct;
% traj_struct.N = 3;
% traj_struct.q_0 = q_target;
% traj_struct.q_0_p = zeros(n,1);
% traj_struct.q_0_pp = zeros(n,1);
% traj_struct.joint_points = -ones(n,traj_struct.N);
% %traj_struct.pose = [xe0, xeT, [pT(1:3);xeT(4:7)], [pT2(1:3);xeT(4:7)]];
% traj_struct.pose = [xeT, xe2, xe3];
% traj_struct.rotation = cat(3, R_target, R_target, R_target);
% traj_struct.time = [0; 2; T_sim];
% traj_struct.traj_type = [traj_mode.polynomial];
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
% traj_struct.name = 'wrist sing 1 trajectory, Polynomial, Workspace';
% traj_cell{cnt} = traj_struct;
% cnt = cnt+1;


%% Trajectory 11: ellbow sing, Workspace, Polynomial
q_0 = param_robot.q_0_ref';
q_0(n_indices) = rand(6,1);
q_0 = min(max(q_0, param_robot.q_limit_lower), param_robot.q_limit_upper);
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