function [x_d] = create_sinus_traj(x_target, x0_target, t, R_init, rot_ax, rot_alpha_scale, alpha_offset, Phi_init, delta_Phi, param_traj_sin_poly, init_bus_param)

    T = param_traj_sin_poly.T;
    omega = param_traj_sin_poly.omega;
    phi = param_traj_sin_poly.phi;
    if(t > T)
        s    = 1;
        s_p  = 0;
        s_pp = 0;
    else
        [s, s_p, s_pp] = trajectory_poly(t, 0, 1, T);
    end
    xT = x_target(1:3);
    x0 = x0_target(1:3);

    a = (xT - x0)/2; % width of sinus movement

    sin_t = sin(omega*t - phi);
    cos_t = cos(omega*t - phi);

    a_sin_t = a*sin_t;
    a_cos_t = a*cos_t;

    x_ref = x0 + (a + a_sin_t) * s;
    x_ref_p =    a_cos_t * omega * s   + (a + a_sin_t) * s_p;
    x_ref_pp =  -a_sin_t * omega^2 * s + a_cos_t * s_p * omega * 2 + (a + a_sin_t) * s_pp;

    sin_t_01 = s*(1 + sin_t)/2;
    alpha    = s*sin_t_01;
    alpha_p  = s*omega*cos_t/2 + s_p*sin_t_01;
    alpha_pp = -s*omega^2*sin_t + 2*s_p*omega*cos_t + s_pp*sin_t_01;

    skew_ew  = skew(rot_ax);
    R_act    = R_init*(eye(3) + sin(rot_alpha_scale*alpha)*skew_ew + (1-cos(rot_alpha_scale*alpha))*skew_ew^2);

    omega_d   = alpha_p*rot_ax;
    omega_d_p = alpha_pp*rot_ax;
    q_d   = rotation2quaternion(R_act);
    [q_d_p, q_d_pp] = quat_deriv(q_d, omega_d, omega_d_p);

    Phi_act = rotm2rpy(R_act);
    Phi_act_p = T_rpy(Phi_act)*omega_d;
    Phi_act_pp =T_rpy_p(Phi_act, Phi_act_p)*omega_d + T_rpy(Phi_act)*omega_d_p;

    alpha_d = alpha;
    alpha_d_p = alpha_p;
    alpha_d_pp = alpha_pp;
    rot_ax_d = rot_ax;

    x_d = init_bus_param.x_d;
    x_d.p_d       = x_ref;
    x_d.p_d_p     = x_ref_p;
    x_d.p_d_pp    = x_ref_pp;
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
    x_d.q_d_rel = rotation2quaternion(R_init);
end