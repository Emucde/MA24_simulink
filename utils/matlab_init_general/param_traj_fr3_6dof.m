%% Calculate target positions
cnt = 1;

Q_pos = 1e0 * eye(m);  % Weight for the position error in the cost function.
Q_m = 0.5;             % Weight for the manipulability error in the cost function.
Q_q = 1e5 * eye(n);    % Weight for the deviaton of q_sol to q_d
Q_nl = 1e-1 * eye(n);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
q_d = param_robot.q_0_ref(n_indices);

xe0 = [0.3921; 0.3921; 0.5211; 0; 1/sqrt(2); 1/sqrt(2); 0];
xeT = [xe0(1:3,1) + [0; 0; -0.5]; quat_R_endeffector_py( Rz(pi/4)*quat2rotm_v2(xe0(4:7)) )]; % rotm2quat (from matlab) is very precise but slow

%q_0 = fsolve(@(q) kin_fun(xe0, q), q_d); % test if the function works
options = optimoptions('fsolve', 'Algorithm', 'levenberg-marquardt', 'MaxFunctionEvaluations', 1000);
q_0 = fsolve(@(q) kin_fun(xe0, q), q_d, options); % test if the function works
%[q_0, ~] = inverse_kinematics(param_robot, xe0, q_d, Q_pos, Q_m, Q_q, Q_nl,  1e-2, 100, ct_ctrl_param);
H_0 = hom_transform_endeffector_py(q_0);
xe0 = [H_0(1:3,4); quat_R_endeffector_py(H_0(1:3,1:3))]; % better to exact start in point

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
traj_struct = create_param_sin_poly(traj_struct, param_global, 'T', 1, 'phi', 0); % Param for sinus poly trajectory
traj_struct.name = 'Wrist Singularity 1, Sinus, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

function out = kin_fun(xe, q)
    H = hom_transform_endeffector_py(q);
    
    RR = H(1:3,1:3)*quat2rotm_v2(xe(4:7))';
    p_err = H(1:3,4) - xe(1:3);
    q_err = quat_R_endeffector_py(RR);
    r_err = q_err(2:4);
    %r_err = [RR(3,2) - RR(2,3); RR(1,3) - RR(3,1); RR(2,1) - RR(1,2)];
    out = r_err'*r_err + p_err'*p_err;

    % He = [quat2rotm_v2(xe(4:7)), xe(1:3); 0 0 0 1];
    % out = sum((H - He).^2, 'all');
end