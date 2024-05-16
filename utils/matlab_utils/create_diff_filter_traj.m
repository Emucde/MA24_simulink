function [x_d, x_kp1] = create_diff_filter_traj(x_target, x_k, R_init, rot_ax, rot_alpha_scale, param_traj_filter)
    
    Phi = param_traj_filter.Phi;
    Gamma = param_traj_filter.Gamma;
    alpha_T = 1; % die idee ist, dass alpha \in [0,1] ist, da man die Drehung normiert durchführt.
    x_kp1 = Phi*x_k + Gamma*[x_target(1:3);alpha_T];
    
    xd   = x_kp1(param_traj_filter.p_d_index);
    dxd  = x_kp1(param_traj_filter.p_d_p_index);
    ddxd = x_kp1(param_traj_filter.p_d_pp_index);
    
    alpha    =   xd(4);
    alpha_p  =  dxd(4);
    alpha_pp = ddxd(4);
    
    skew_ew = skew(rot_ax);
    R_act = R_init*(eye(3) + sin(rot_alpha_scale*alpha)*skew_ew + (1-cos(rot_alpha_scale*alpha))*skew_ew^2);    

    omega_d   = alpha_p*rot_ax;
    omega_d_p = alpha_pp*rot_ax;
    q_d   = rotation2quaternion(R_act);
    %q_d   = rotm2quat_v3(R_act);
    q_d_p = quat_deriv(q_d, omega_d);

    x_d.p_d       =   xd(1:3);
    x_d.p_d_p     =  dxd(1:3);
    x_d.p_d_pp    = ddxd(1:3);
    x_d.R_d       = R_act;
    x_d.q_d       = q_d;
    x_d.q_d_p     = q_d_p;
    x_d.omega_d   = omega_d;
    x_d.omega_d_p = omega_d_p;
end