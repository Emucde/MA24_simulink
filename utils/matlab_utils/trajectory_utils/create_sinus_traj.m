function [x_d] = create_sinus_traj(traj_select, t, param_traj, init_bus_param)
    param_sin_poly = param_traj.sin_poly(traj_select).sin_poly;
    start_index = param_traj.start_index(traj_select);

    x0    = param_traj.pose(1:3, start_index);
    xT    = param_traj.pose(1:3, start_index+1);
    alpha0 = param_traj.alpha(start_index);
    alphaT = param_traj.alpha(start_index+1);

    R_init = param_traj.rotation(:, :, start_index);
    rot_ax = param_traj.rot_ax(:, start_index+1);

    T =  param_sin_poly.T;
    omega =  param_sin_poly.omega;
    phi =  param_sin_poly.phi;

    if(t > T)
        s    = 1;
        s_p  = 0;
        s_pp = 0;
    else
        [s, s_p, s_pp] = trajectory_poly(t, 0, 1, T);
    end

    a = (xT - x0)/2; % width of sinus movement

    sin_t = sin(omega*t - phi);
    cos_t = cos(omega*t - phi);

    a_sin_t = a*sin_t;
    a_cos_t = a*cos_t;

    apa_sin_t = (a + a_sin_t);

    x_ref = x0 + apa_sin_t * s;
    x_ref_p =    a_cos_t * omega * s   + apa_sin_t * s_p;
    x_ref_pp =  -a_sin_t * omega^2 * s + a_cos_t * s_p * omega * 2 + apa_sin_t * s_pp;

    b = (alphaT - alpha0)/2;
    b_sin_t = b*sin_t;
    b_cos_t = b*cos_t;

    bpb_sin_t = (b + b_sin_t);

    alpha    =  alpha0 + bpb_sin_t * s;
    alpha_p  =  b_cos_t * omega * s   + bpb_sin_t * s_p;
    alpha_pp = -b_sin_t * omega^2 * s + b_cos_t * s_p * omega * 2 + bpb_sin_t * s_pp;

    RR    = eye(3) + sin(alpha-alpha0)*skew_ew + (1-cos(alpha-alpha0))*skew_ew^2;
    
    % R_act    = RR*R_init; % Vormultiplikation (in find_rotation_axis wird Nachmultiplikation für RR verwendet!!)
    R_act    = R_init*RR; % Nachmultiplikation (in find_rotation_axis wird Nachmultiplikation für RR verwendet!!)

    alpha_d = alpha;
    alpha_d_p = alpha_p;
    alpha_d_pp = alpha_pp;
    rot_ax_d = rot_ax;

    omega_d   = alpha_d_p*rot_ax_d;
    omega_d_p = alpha_d_pp*rot_ax_d;

    %if(t==5)
    %    disp('no')
    %end

    q_d   = quat_R_endeffector_py(R_act);
    [q_d_p, q_d_pp] = quat_deriv(q_d, omega_d, omega_d_p);

    Phi_act = rotm2rpy(R_act);
    % w = T Phi_p -> Phi_p = T^(-1)w = T\w
    % w_p = Tp Phi_p + T Phi_pp -> Phi_pp = T^-1(w_p - Tp Phi_p) = T\(w_p - Tp Phi_p)
    Phi_act_p = T_rpy(Phi_act)\omega_d;
    Phi_act_pp = T_rpy(Phi_act)\(omega_d_p - T_rpy_p(Phi_act, Phi_act_p)*Phi_act_p);

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
    x_d.alpha_d_offset = alpha0;
    x_d.q_d_rel = quat_R_endeffector_py(R_init);
end