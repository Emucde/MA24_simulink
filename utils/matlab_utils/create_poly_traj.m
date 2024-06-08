function [x_d] = create_poly_traj(x_target, alphaT, x0_target, alpha0, T_start, t, R_init, rot_ax, rot_alpha_scale, Phi_init, delta_Phi, param_traj_poly)

    T = param_traj_poly.T;
    if(t-T_start > T)
        p_d =    [x_target(1:3); alphaT]; % abhängig vom akt. target
        p_d_p =  [zeros(3,1); 0];% müssen 0 sein, weil änderungsraten
        p_d_pp = [zeros(3,1); 0];% am Ende sicher 0 sind.
    else
        yT = [x_target(1:3); alphaT]; % poly contains [x,y,z,alpha]
        y0 = [x0_target(1:3); alpha0];
        [p_d, p_d_p, p_d_pp] = trajectory_poly(t-T_start, y0, yT, T);
    end
    
    alpha    = rot_alpha_scale*p_d(4);
    alpha_p  = rot_alpha_scale*p_d_p(4);
    alpha_pp = rot_alpha_scale*p_d_pp(4);

    skew_ew  = skew(rot_ax);

    RR = (eye(3) + sin(alpha)*skew_ew + (1-cos(alpha))*skew_ew^2);
    
    % R_act    = R_init*RR; % Nachmultiplikation
    R_act    = RR*R_init; % Vormultiplikation (in find_rotation_axis wird Vormultiplikation für RR verwendet!!)

    omega_d   = alpha_p*rot_ax;
    omega_d_p = alpha_pp*rot_ax;

    % Korrektes Quaternion wählen, dass zu R_act passt:
    %q_err = [cos(alpha/2); rot_ax*sin(alpha/2)];
    %q_d = quat_mult(rotation2quaternion(R_init), q_err);
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
end