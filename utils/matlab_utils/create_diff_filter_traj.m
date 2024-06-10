function [x_d, x_kp1] = create_diff_filter_traj(x_target, x_k, alpha_T, R_init, rot_ax, rot_alpha_scale, alpha_offset, Phi_init, delta_Phi, param_traj_filter)
    
    Phi = param_traj_filter.Phi;
    Gamma = param_traj_filter.Gamma;
    %alpha_T = 1; % die idee ist, dass alpha \in [0,1] ist, da man die Drehung normiert durchführt.
    x_kp1 = Phi*x_k + Gamma*[x_target(1:3);alpha_T];
    
    xd   = x_kp1(param_traj_filter.p_d_index);
    dxd  = x_kp1(param_traj_filter.p_d_p_index);
    ddxd = x_kp1(param_traj_filter.p_d_pp_index);
    
    alpha    =   xd(4)*rot_alpha_scale;
    alpha_p  =  dxd(4)*rot_alpha_scale;
    alpha_pp = ddxd(4)*rot_alpha_scale;
    
    skew_ew = skew(rot_ax);
    RR = (eye(3) + sin(alpha)*skew_ew + (1-cos(alpha))*skew_ew^2);
    
    % R_act    = R_init*RR; % Nachmultiplikation
    R_act    = RR*R_init; % Vormultiplikation (in find_rotation_axis wird Vormultiplikation für RR verwendet!!)

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
    x_d.q_d_rel = rotation2quaternion(R_init);
end