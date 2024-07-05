function [x_d, x_kp1] = create_diff_filter_traj(traj_select, t, x_k, param_traj, init_bus_param)
    traj_init = param_traj.traj_init;

    start_index = traj_init.start_index(traj_select);
    stop_index  = traj_init.stop_index(traj_select);
    t_val       = traj_init.time(start_index:stop_index);
    i = sum(t >= t_val);

    if( i == length(t_val) )
        i = length(t_val)-1; % TODO
    end

    alpha0 = traj_init.alpha(i);
    
    xeT    = traj_init.pose(1:3, i+1);
    alphaT = traj_init.alpha(i+1);

    if( t == t_val(i))
        % otherwise we cannot exactly compensate alpha by alpha0.
        % Here errors in size of 1e-15 can lead to sign jumps in quaternions
        x_k(param_traj.diff_filter.p_d_index(4)) = alpha0;
    end

    R_init = traj_init.rotation(:, :, i);
    rot_ax = traj_init.rot_ax(:, i+1);

    alpha_offset = 0; % TODO DELETE

    yT = [xeT; alphaT];

    Phi = param_traj.diff_filter.Phi;
    Gamma = param_traj.diff_filter.Gamma;
    x_kp1 = Phi*x_k + Gamma*yT;
    
    xd   = x_kp1(param_traj.diff_filter.p_d_index);
    dxd  = x_kp1(param_traj.diff_filter.p_d_p_index);
    ddxd = x_kp1(param_traj.diff_filter.p_d_pp_index);
    
    alpha    =   xd(4);
    alpha_p  =  dxd(4);
    alpha_pp = ddxd(4);
    
    skew_ew = skew(rot_ax);
    RR = (eye(3) + sin(alpha-alpha0)*skew_ew + (1-cos(alpha-alpha0))*skew_ew^2);
    
    R_act    = RR*R_init; % Vormultiplikation (in find_rotation_axis wird Vormultiplikation für RR verwendet!!)

    omega_d   = alpha_p*rot_ax;
    omega_d_p = alpha_pp*rot_ax;
    q_d   = rotm2quat_v4(R_act);
    [q_d_p, q_d_pp] = quat_deriv(q_d, omega_d, omega_d_p);

    Phi_act = rotm2rpy(R_act);
    Phi_act_p = T_rpy(Phi_act)*omega_d;
    Phi_act_pp =T_rpy_p(Phi_act, Phi_act_p)*omega_d + T_rpy(Phi_act)*omega_d_p;

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
    x_d.q_d_rel = rotm2quat_v4(R_init);
end