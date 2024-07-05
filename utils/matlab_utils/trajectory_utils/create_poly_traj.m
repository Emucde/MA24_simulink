function [x_d] = create_poly_traj(traj_select, t, param_traj, init_bus_param)
    traj_init = param_traj.traj_init;
    
    start_index = traj_init.start_index(traj_select);
    stop_index  = traj_init.stop_index(traj_select);
    t_val       = traj_init.time(start_index:stop_index);
    i = sum(t >= t_val);

    if( i == length(t_val) )
        i = length(t_val)-1; % TODO
        t = t_val(end) - t_val(end-1);
    else
        t = t - t_val(i);
    end
    
    xe0    = traj_init.pose(1:3, i);
    alpha0 = traj_init.alpha(i);

    xeT    = traj_init.pose(1:3, i+1);
    alphaT = traj_init.alpha(i+1);

    R_init = traj_init.rotation(:, :, i);
    rot_ax = traj_init.rot_ax(:, i+1);

    alpha_offset = 0; % TODO DELETE

    y0 = [xe0; alpha0];
    yT = [xeT; alphaT];

    [p_d, p_d_p, p_d_pp] = trajectory_poly(t, y0, yT, t_val(i+1) - t_val(i));
    
    alpha    = p_d(4);
    alpha_p  = p_d_p(4);
    alpha_pp = p_d_pp(4);

    skew_ew  = skew(rot_ax);

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
    x_d.p_d       = p_d(1:3);
    x_d.p_d_p     = p_d_p(1:3);
    x_d.p_d_pp    = p_d_pp(1:3);
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