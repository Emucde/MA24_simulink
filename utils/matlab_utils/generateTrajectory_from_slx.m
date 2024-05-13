function param_trajectory = generateTrajectory_from_slx(slx_file_path, q_0, H_0_init, traj_select_fin, T_sim, T_horizon, param_global)
% This function utilizes a pre-defined Simulink model (`mdl`) to generate
% a trajectory based on initial joint angles, forward kinematics, trajectory
% selection, simulation time, and MPC and global parameter structures.
%
% Inputs:
%   - slx_file_path: path of simulink file
%   - q_0 (nx1): Initial joint angles vector.
%   - H_0_init (4x4): Initial forward kinematics matrix.
%   - traj_select_fin (integer): Trajectory selection (1, 2, or 3).
%   - T_sim (scalar): Simulation time.
%   - T_horizon (scalar): Total horizon time of MPC
%   - param_global (struct): Struct containing global parameters,
%                             including sampling time (Ta).
%
% Outputs:
%   - param_trajectory (struct): Struct containing the generated
%                               trajectory data:
%       - t (1xN): Time vector.
%       - p_d (3xN): Desired position trajectory.
%       - p_d_p (3xN): Desired velocity trajectory.
%       - p_d_pp (3xN): Desired acceleration trajectory.
%       - q_d (4xN): Desired joint positions (unit quaternions).
%       - omega_d (3xN): Desired joint velocities.
%       - omega_d_p (3xN): Desired joint accelerations.

%inputs_sim = createInputDataset(mdl); %save('traj_input_sim.mat', 'inputs_sim')
    tic
    load('traj_input_sim.mat');
    inputs_sim{1}.q.Data = repmat(q_0, 1,1,2); %robot_model.q
    inputs_sim{1}.H.Data = repmat(H_0_init, 1,1,2); %robot_model.H
    inputs_sim{2}.Data   = repmat(uint32(traj_select_fin),1,1,2); % trajectory selection
    inputs_sim{3}.Data   = ones(1,1,2); % start
    inputs_sim{4}.Data   = zeros(1,1,2); % stop
    inputs_sim{5}.Data   = zeros(1,1,2); % reset
    
    simIn = Simulink.SimulationInput(slx_file_path);
    simIn = setExternalInput(simIn,inputs_sim);
    simIn = setModelParameter(simIn, 'StartTime', '0.0', ...
                                     'StopTime', num2str(T_sim+T_horizon),...
                                     'Solver', 'ode4', ...
                                     'SolverType', 'Fixed-step',...
                                     'FixedStep', num2str(param_global.Ta));
    data_sim_traj = sim(simIn);
    
    x_d_out   = data_sim_traj.yout{1}.Values;
    traj_t    = data_sim_traj.tout;
    p_d       = reshape(x_d_out.p_d.Data,       3, [], 1);
    p_d_p     = reshape(x_d_out.p_d_p.Data,     3, [], 1);
    p_d_pp    = reshape(x_d_out.p_d_pp.Data,    3, [], 1);
    q_d       = reshape(x_d_out.q_d.Data,       4, [], 1); %this is a unit quaternion
    q_d_p     = reshape(x_d_out.q_d_p.Data,     4, [], 1); %this is a unit quaternion
    omega_d   = reshape(x_d_out.omega_d.Data,   3, [], 1);
    omega_d_p = reshape(x_d_out.omega_d_p.Data, 3, [], 1);

    % create param for trajectory: append N samples for end of trajectory
    param_trajectory.t =         traj_t;   
    param_trajectory.p_d =       p_d;      
    param_trajectory.p_d_p =     p_d_p;    
    param_trajectory.p_d_pp =    p_d_pp;   
    param_trajectory.q_d =       q_d;
    param_trajectory.q_d_p =     q_d_p;      
    param_trajectory.omega_d =   omega_d;  
    param_trajectory.omega_d_p = omega_d_p;
    disp(['generateTrajectory_from_slx.m: Execution Time for Trajectory Calculation: ', sprintf('%f', toc), 's']);
end