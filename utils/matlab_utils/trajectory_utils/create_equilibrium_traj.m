function [x_d] = create_equilibrium_traj(traj_select, t, param_traj, init_bus_param)
% create_equilibrium_traj - Creates an equilibrium trajectory based on the selected trajectory and time.
% Inputs:
%   traj_select - Index of the trajectory to select.
%   t           - Time vector.
%   param_traj  - Structure containing trajectory parameters.
%   init_bus_param - Initial bus parameters.
% Outputs:
%   x_d         - Structure containing the desired state of the system. 
    start_index = param_traj.start_index(traj_select);
    stop_index  = param_traj.stop_index(traj_select);
    t_val       = param_traj.time(start_index:stop_index);
    i = sum(t >= t_val);

    if( i == length(t_val) )
        i = length(t_val)-1; % TODO
    end

    i = i + start_index;
    
    xeT    = param_traj.pose(:, i);
    R_init = param_traj.rotation(:, :, i-1);
    alpha_offset = 0; % TODO DELETE

    R_target = param_traj.rotation(:, :, i);

    omega_d   = zeros(3,1);
    omega_d_p = zeros(3,1);
    q_d   = quat_R_endeffector_py(R_init);
    q_d_p = zeros(4,1);
    q_d_pp = zeros(4,1);

    [rot_ax_d, alpha_d] = find_rotation_axis(R_init, R_target);

    x_d = init_bus_param.x_d;
    x_d.p_d       = xeT(1:3);
    x_d.p_d_p     = zeros(3,1);
    x_d.p_d_pp    = zeros(3,1);
    x_d.Phi_d     = rotm2rpy(R_init);
    x_d.Phi_d_p   = zeros(3,1);
    x_d.Phi_d_pp  = zeros(3,1);
    x_d.R_d       = R_init;
    x_d.q_d       = q_d;
    x_d.q_d_p     = q_d_p;
    x_d.q_d_pp    = q_d_pp;
    x_d.omega_d   = omega_d;
    x_d.omega_d_p = omega_d_p;
    x_d.alpha_d   = alpha_d;
    x_d.alpha_d_p = 0;
    x_d.alpha_d_pp = 0;
    x_d.rot_ax_d = rot_ax_d;
    x_d.alpha_d_offset = alpha_offset;
    x_d.q_d_rel = quat_R_endeffector_py(R_init);
end