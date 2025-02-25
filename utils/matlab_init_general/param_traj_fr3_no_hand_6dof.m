%% Calculate target positions
cnt = 1;
random_traj = false;

if(random_traj)
    param_traj_fr3_no_hand_6dof_random_traj;
else

% %% Trajectory 1: Stretch arm out
% disp('------------------------------------');
% fprintf('Trajectory %d\n\n', cnt);
% q_0 = param_robot.q_0_ref;
% H_0 = hom_transform_endeffector_py(q_0);
% R_init = H_0(1:3,1:3);
% xe0 = [H_0(1:3,4); quat_R_endeffector_py(R_init)]; % better to exact start in point

% Q_pos = 1*diag([1,1e3,1e3,1e-5,1e5,1e5,1e5]);  % Weight for the position error in the cost function.
% Q_m = 1e8;             % Weight for the manipulability error in the cost function.
% Q_q = 1e5*diag(ones(n_red, 1));    % Weight for the deviaton of q_sol to q_d
% Q_nl = 1e-1 * eye(n_red);  % Weight of nl_spring_force(q, ct_ctrl_param, param_robot) -> pose should not start near to limits!
% q_d = [0.000, 0.781, 0, -0.467, 0.000, 1.248, 0.785]';

% R_J_d = struct( ...
%     'r21', 0, ...
%     'r31', 0, 'r32', 0, ...
%     'r41', 0, 'r42', 0, 'r43', 0, ...
%     'r51', 0, 'r52', 0, 'r53', 0, 'r54', 0, ...
%     'r61', 0, 'r62', 0, 'r63', 0, 'r64', 0, 'r65', 0 ...
% );

% K_J = struct(... 
%     'k21', 0, ...
%     'k31', 0, 'k32', 0, ...
%     'k41', 0, 'k42', 0, 'k43', 0, ...
%     'k51', 0, 'k52', 0, 'k53', 0, 'k54', 0, ...
%     'k61', 0, 'k62', 0, 'k63', 0, 'k64', 0, 'k65', 0 ...
% );

% H_d = hom_transform_endeffector_py(q_d);
% xe_d = [H_d(1:3,4); quat_R_endeffector_py(H_d(1:3,1:3))];

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

% param_inv_kin.xpos_joint1 = zeros(7,1);
% param_inv_kin.xpos_joint2 = zeros(7,1);
% param_inv_kin.xpos_joint3 = zeros(7,1);
% param_inv_kin.xpos_joint4 = zeros(7,1);
% param_inv_kin.xpos_joint5 = zeros(7,1);
% param_inv_kin.xpos_joint6 = zeros(7,1);
% param_inv_kin.xpos_joint7 = zeros(7,1);

% param_inv_kin.Q_joint1 = eye(7);
% param_inv_kin.Q_joint2 = eye(7);
% param_inv_kin.Q_joint3 = eye(7);
% param_inv_kin.Q_joint4 = eye(7);
% param_inv_kin.Q_joint5 = eye(7);
% param_inv_kin.Q_joint6 = eye(7);
% param_inv_kin.Q_joint7 = eye(7);

% [q_target_red, ~] = inverse_kinematics(param_robot, param_inv_kin, 'not used');
% q_target = q_0;
% q_target(n_indices) = q_target_red;
% H_target = hom_transform_endeffector_py(q_target);
% R_target = H_target(1:3,1:3);
% xeT = [H_target(1:3,4); quat_R_endeffector_py(R_target)];
% % xeT(1) = xeT(1) + 0.2;

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

% %% Trajectory 2: Stretch arm in
% disp('------------------------------------');
% fprintf('Trajectory %d\n\n', cnt);
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

% % traj_struct = struct;
% % traj_struct.N = 5;
% % traj_struct.q_0 = q_0;
% % traj_struct.q_0_p = zeros(n,1);
% % traj_struct.q_0_pp = zeros(n,1);
% % traj_struct.joint_points = [q_0; q_target; q_target; q_0; q_0]';
% % traj_struct.pose = -ones(n,traj_struct.N);
% % traj_struct.rotation = cat(3, -eye(3), -eye(3), -eye(3), -eye(3), -eye(3));
% % traj_struct.time = [0; 4; 5; 9; 10];
% % traj_struct.traj_type = [traj_mode.polynomial_jointspace];
% % traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
% % traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
% % traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
% % traj_struct.name = 'Wrist singularity 1, Polynomial, Jointspace';
% % traj_cell{cnt} = traj_struct;
% % cnt = cnt+1;

%% Trajectory 3: Smooth Singularity, Polynomial
% Define the joint configurations as a cell array
try
    single_sing_pose = readmatrix('./main_c/sorted_output_single.csv');
    N_row = length(single_sing_pose);
    N_points = 2;
    start_index = 466585;
    end_index = 466676;
    stepwidth = round((end_index-start_index)/N_points);
    indices = start_index:stepwidth:end_index;
    start_time = 0;
    end_time = 10;
    time = linspace(start_time, end_time, N_points);
    q = single_sing_pose(indices, :);
catch
    q = {
        [1.000342 -0.606696 0 -1.075555 1.078343 2.629348 1.173911];
        [1.230009 0.828349 0 -1.137286 1.226867 3.405086 1.439463];
        [1.232434 -0.668604 0 -1.217097 1.39176 2.155764 1.466212];
        [0.105523 -0.632875 0 -1.1298 1.039899 2.054842 -1.059827];
        [1.258925 -0.669717 0 -1.038616 0.358037 2.77123 1.476863];
        [1.366694 -0.494639 0 -0.900858 1.423274 2.936039 0.951906];
    };
end

disp('------------------------------------');
fprintf('Trajectory %d\n\n', cnt);

% Preallocate variables for poses and rotation matrices
x = zeros(7, N_points);  % Assume an appropriate size according to your output
R = zeros(3, 3, N_points); % For rotation matrices

for i = 1:N_points
    H = hom_transform_endeffector_py(q(i, :));
    R(:,:,i) = H(1:3, 1:3);
    x(:, i) = [H(1:3, 4); quat_R_endeffector_py(R(:,:,i))];
end

traj_struct = struct;
traj_struct.N = N_points;
traj_struct.q_0 = q(1, :)';
traj_struct.q_0_p = zeros(n,1);
traj_struct.q_0_pp = zeros(n,1);
traj_struct.joint_points = -ones(n,traj_struct.N);
traj_struct.pose = x;
traj_struct.rotation = R;
traj_struct.time = time;
traj_struct.traj_type = [traj_mode.polynomial];
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda (1/s)', -3.5); % Param differential filter 5th order trajectory
traj_struct = create_param_diff_filter(traj_struct, param_global, 'lambda', 0, 'n_order', 6, 'n_input', n, 'diff_filter_jointspace');
traj_struct = create_param_sin_poly(traj_struct, param_global); % Param for sinus poly trajectory
traj_struct.name = 'Multipoint 2, Polynomial, Workspace';
traj_cell{cnt} = traj_struct;
cnt = cnt+1;

end