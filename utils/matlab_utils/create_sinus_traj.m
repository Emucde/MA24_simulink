function [x_d] = create_sinus_traj(x_target, x0_target, t, R_init, rot_ax, rot_alpha_scale, Phi_init, delta_Phi, param_traj_sin_poly)

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
    %q_d   = rotm2quat_v3(R_act);
    [q_d_p, q_d_pp] = quat_deriv(q_d, omega_d, omega_d_p);

    Phi_act    = Phi_init + alpha*delta_Phi;
    Phi_act_p  = alpha_p*delta_Phi;
    Phi_act_pp = alpha_pp*delta_Phi;

    %xd_prev   = x_k(param_traj_filter.p_d_index);
    %alpha_prev = xd_prev(4)*rot_alpha_scale;
    alpha_d = alpha;% - alpha_prev; % relativ gesehen
    alpha_d_p = alpha_p;
    alpha_d_pp = alpha_pp;
    rot_ax_d = rot_ax; % Darf ich nur, da rotax konstant ist!

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
end