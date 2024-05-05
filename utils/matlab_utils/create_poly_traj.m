function [x_d] = create_poly_traj(x_target, x0_target, T_start, t, R_init, rot_ax, rot_alpha_scale, param_traj_poly)

    T = param_traj_poly.T;
    if(t-T_start > T)
        p_d =    [x_target(1:3); 1];
        p_d_p =  [zeros(3,1); 1];
        p_d_pp = [zeros(3,1); 1];
    else
        yT = [x_target(1:3); 1]; % poly contains [x,y,z,alpha]
        y0 = [x0_target(1:3); 0];
        [p_d, p_d_p, p_d_pp] = trajectory_poly(t-T_start, y0, yT, T);
    end
    
    alpha = p_d(4);
    alpha_p = p_d_p(4);
    alpha_pp = p_d_pp(4);
    skew_ew = skew(rot_ax);
    R_act = R_init*(eye(3) + sin(rot_alpha_scale*alpha)*skew_ew + (1-cos(rot_alpha_scale*alpha))*skew_ew^2);
    
    x_d.p_d = p_d(1:3);
    x_d.p_d_p =  p_d_p(1:3);
    x_d.p_d_pp = p_d_pp(1:3);
    x_d.R_d = R_act;
    x_d.q_d = rotation2quaternion(R_act);
    x_d.omega_d   = alpha_p*rot_ax;
    x_d.omega_d_p = alpha_pp*rot_ax;

end