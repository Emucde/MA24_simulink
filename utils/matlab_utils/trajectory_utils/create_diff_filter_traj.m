function [x_d, x_kp1] = create_diff_filter_traj(traj_select, t, x_k, param_traj, init_bus_param)
% create_diff_filter_traj - Creates the differential filter trajectory
%   [x_d, x_kp1] = create_diff_filter_traj(traj_select, t, x_k, param_traj, init_bus_param)
%   This function computes the differential filter trajectory based on the
%   selected trajectory and the current time `t`. It uses the parameters from `param_traj` and initializes the bus parameters from `init_bus_param`.
% Inputs:
%       traj_select: Index of the trajectory to select.
%       t: Current time at which the trajectory is evaluated.
%       x_k: Current state vector.
%       param_traj: Structure containing trajectory parameters.
%       init_bus_param: Structure containing initial bus parameters.
% Outputs:
%       x_d: Structure containing the desired state vector.
%       x_kp1: State vector for the next time step.
    param_diff_filter = param_traj.diff_filter(traj_select).diff_filter;

    start_index = param_traj.start_index(traj_select);
    stop_index  = param_traj.stop_index(traj_select);
    t_val       = param_traj.time(start_index:stop_index);
    i = sum(t >= t_val);

    if( i == length(t_val) )
        i = length(t_val)-1; % der 0te wert ist nicht relevant!
    end

    i = i + start_index; % zeigt schon auf target

    % TODO: Das führt zu einen sprung in der Trajektorie, wenn alpha nicht perfekt erreicht wurde...
    alpha0 = param_traj.alpha(i-1);
    
    xeT    = param_traj.pose(1:3, i);

    alphaT = param_traj.alpha(i);

    if( t == t_val(i-start_index))
        % otherwise we cannot exactly compensate alpha by alpha0.
        % Here errors in size of 1e-15 can lead to sign jumps in quaternions
        x_k(param_diff_filter.p_d_index(4)) = alpha0;
    end

    R_init = param_traj.rotation(:, :, i-1);
    rot_ax = param_traj.rot_ax(:, i);

    alpha_offset = 0; % TODO DELETE

    yT = [xeT; alphaT];

    Phi = param_diff_filter.Phi;
    Gamma = param_diff_filter.Gamma;
    x_kp1 = Phi*x_k + Gamma*yT;
    
    xd   = x_kp1(param_diff_filter.p_d_index);
    dxd  = x_kp1(param_diff_filter.p_d_p_index);
    ddxd = x_kp1(param_diff_filter.p_d_pp_index);
    
    alpha    =   xd(4);
    alpha_p  =  dxd(4);
    alpha_pp = ddxd(4);
    
    skew_ew = skew(rot_ax);
    RR = (eye(3) + sin(alpha-alpha0)*skew_ew + (1-cos(alpha-alpha0))*skew_ew^2);
    
    % R_act    = RR*R_init; % Vormultiplikation (in find_rotation_axis wird Nachmultiplikation für RR verwendet!!)
    R_act    = R_init*RR; % Nachmultiplikation (in find_rotation_axis wird Nachmultiplikation für RR verwendet!!)

    omega_d   = alpha_p*rot_ax;
    omega_d_p = alpha_pp*rot_ax;
    q_d   = quat_R_endeffector_py(R_act);
    [q_d_p, q_d_pp] = quat_deriv(q_d, omega_d, omega_d_p);

    Phi_act = rotm2rpy(R_act);
    % w = T Phi_p -> Phi_p = T^(-1)w = T\w
    % w_p = Tp Phi_p + T Phi_pp -> Phi_pp = T^-1(w_p - Tp Phi_p) = T\(w_p - Tp Phi_p)
    Phi_act_p = T_rpy(Phi_act)\omega_d;
    Phi_act_pp = T_rpy(Phi_act)\(omega_d_p - T_rpy_p(Phi_act, Phi_act_p)*Phi_act_p);

    alpha_d = alpha;
    alpha_d_p = alpha_p;
    alpha_d_pp = alpha_pp;
    rot_ax_d = rot_ax;

    x_d = init_bus_param.x_d;
    x_d.p_d       =   xd(1:3);
    x_d.p_d_p     =  dxd(1:3);
    x_d.p_d_pp    = ddxd(1:3);
    x_d.Phi_d     = Phi_act;
    x_d.Phi_d_p   = Phi_act_p;
    x_d.Phi_d_pp  = Phi_act_pp;
    x_d.R_d       = R_act;
    x_d.q_d       = q_d;
    x_d.q_d_p     = q_d_p;
    x_d.q_d_pp    = q_d_pp;
    x_d.omega_d   = omega_d;
    x_d.omega_d_p = omega_d_p;
    x_d.alpha_d   = alpha_d;
    x_d.alpha_d_p = alpha_d_p;
    x_d.alpha_d_pp = alpha_d_pp;
    x_d.rot_ax_d = rot_ax_d;
    x_d.alpha_d_offset = alpha_offset;
    x_d.q_d_rel = quat_R_endeffector_py(R_init);
end