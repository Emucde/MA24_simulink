function [x_d] = create_poly_traj_jointspace(traj_select, t, param_traj, init_bus_param)  
    start_index = param_traj.start_index(traj_select);
    stop_index  = param_traj.stop_index(traj_select);
    t_val       = param_traj.time(start_index:stop_index);
    i = sum(t >= t_val);

    if( i == length(t_val) )
        i = length(t_val)-1; % TODO
        t = t_val(end) - t_val(end-1);
    else
        t = t - t_val(i);
    end

    i = i + start_index; % zeigt schon auf target
    
    q0    = param_traj.joint_points(:, i-1);
    qT    = param_traj.joint_points(:, i);

    [xd, dxd, ddxd] = trajectory_poly(t, q0, qT, t_val(i-start_index+1) - t_val(i-start_index));
    
    H = hom_transform_endeffector_py(xd);
    J = geo_jacobian_endeffector_py(xd);
    J_p = geo_jacobian_endeffector_p_py(xd, dxd);

    pp_d = [H(1:3,4); quat_endeffector_py(xd)];
    pp_d_p = J*dxd;
    pp_d_pp = J*ddxd + J_p*dxd;

    p_d = pp_d(1:3);
    p_d_p = pp_d_p(1:3);
    p_d_pp = pp_d_pp(1:3);

    omega_d = pp_d_p(4:6);
    omega_d_p = pp_d_pp(4:6);
    
    q_d   = pp_d(4:7);
    [q_d_p, q_d_pp] = quat_deriv(q_d, omega_d, omega_d_p);

    Phi_act = rotm2rpy(H(1:3,1:3));
    % w = T Phi_p -> Phi_p = T^(-1)w = T\w
    % w_p = Tp Phi_p + T Phi_pp -> Phi_pp = T^-1(w_p - Tp Phi_p) = T\(w_p - Tp Phi_p)
    Phi_act_p = T_rpy(Phi_act)\omega_d;
    Phi_act_pp = T_rpy(Phi_act)\(omega_d_p - T_rpy_p(Phi_act, Phi_act_p)*Phi_act_p);

    alpha_d = 0;
    alpha_d_p = 0;
    alpha_d_pp = 0;
    rot_ax_d = zeros(3,1);

    x_d = init_bus_param.x_d;
    x_d.p_d       = p_d(1:3);
    x_d.p_d_p     = p_d_p(1:3);
    x_d.p_d_pp    = p_d_pp(1:3);
    x_d.Phi_d     = Phi_act;
    x_d.Phi_d_p   = Phi_act_p;
    x_d.Phi_d_pp  = Phi_act_pp;
    x_d.R_d       = H(1:3,1:3);
    x_d.q_d       = q_d;
    x_d.q_d_p     = q_d_p;
    x_d.q_d_pp    = q_d_pp;
    x_d.omega_d   = omega_d;
    x_d.omega_d_p = omega_d_p;
    x_d.alpha_d   = alpha_d;
    x_d.alpha_d_p = alpha_d_p;
    x_d.alpha_d_pp = alpha_d_pp;
    x_d.rot_ax_d = rot_ax_d;
    x_d.alpha_d_offset = 0;
    x_d.q_d_rel = q_d;
end