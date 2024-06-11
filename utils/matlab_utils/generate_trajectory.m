function [param_trajectory] = generate_trajectory(t, modus, param_init_pose, param_traj_filter, param_traj_poly, param_traj_sin_poly, param_traj_allg)

    xe0 = param_init_pose.xe0;
    xeT = param_init_pose.xeT;
    alpha0 = param_init_pose.alpha0;
    alphaT = param_init_pose.alphaT;
    R_init = param_init_pose.R_init;
    R_target = param_init_pose.R_target;
    rot_ax = param_init_pose.rot_ax;
    rot_alpha_scale = param_init_pose.rot_alpha_scale;
    T_start = param_init_pose.T_start;
    Phi_init = param_init_pose.Phi_init;
    Phi_target = param_init_pose.Phi_target;
    delta_Phi = param_init_pose.delta_Phi;

    N = length(t);
    p_d       = zeros(3, N);
    p_d_p     = zeros(3, N);
    p_d_pp    = zeros(3, N);
    Phi_d     = zeros(3, N);
    Phi_d_p   = zeros(3, N);
    Phi_d_pp  = zeros(3, N);
    R_d       = zeros(3, 3, N);
    q_d       = zeros(4, N);
    q_d_p     = zeros(4, N);
    q_d_pp    = zeros(4, N);
    omega_d   = zeros(3, N);
    omega_d_p = zeros(3, N);
    alpha_d   = zeros(1, N);
    alpha_d_p = zeros(1, N);
    alpha_d_pp= zeros(1, N);
    rot_ax_d  = zeros(3, N);
    alpha_d_offset = zeros(1, N);
    q_d_rel = zeros(4, N);
    
   

    flag=0;

    % [TODO: ineffizient - sollte in einer Schleife gemacht werden]

    if(modus == 1) % stabilize equilibrium
        for i=1:N
            x_d = create_equilibrium_traj(xeT, R_init, Phi_target);
            p_d(:,i)       = x_d.p_d;
            p_d_p(:,i)     = x_d.p_d_p;
            p_d_pp(:,i)    = x_d.p_d_pp;
            Phi_d(:,i)     = x_d.Phi_d;
            Phi_d_p(:,i)   = x_d.Phi_d_p;
            Phi_d_pp(:,i)  = x_d.Phi_d_pp;
            R_d(:,:,i)     = x_d.R_d;
            q_d(:,i)       = x_d.q_d;
            q_d_p(:,i)     = x_d.q_d_p;
            q_d_pp(:,i)    = x_d.q_d_pp;
            omega_d(:,i)   = x_d.omega_d;
            omega_d_p(:,i) = x_d.omega_d_p;
            alpha_d(:,i)   = x_d.alpha_d;
            alpha_d_p(:,i) = x_d.alpha_d_p;
            alpha_d_pp(:,i)= x_d.alpha_d_pp;
            rot_ax_d(:,i)  = x_d.rot_ax_d;
            alpha_d_offset(:,i) = x_d.alpha_d_offset;
            q_d_rel(:,i) = x_d.q_d_rel;
            if(t(i) >= T_start && flag == 0)
                temp = xe0;
                xe0 = xeT;
                xeT = temp;
                flag = 1;
            end
        end
    elseif(modus == 2) % 5th order differential filter
        x_k = [xe0(1);0;0;0;0;0; xe0(2);0;0;0;0;0; xe0(3);0;0;0;0;0; 0;0;0;0;0;0];
        t_offset = 0;
        alpha_offset = 0;
        T_switch = param_traj_allg.T_switch;
        for i=1:N
            [x_d, x_kp1] = create_diff_filter_traj(xeT, x_k, alphaT, R_init, rot_ax, rot_alpha_scale, alpha_offset, Phi_init, delta_Phi, param_traj_filter);
            p_d(:,i)       = x_d.p_d;
            p_d_p(:,i)     = x_d.p_d_p;
            p_d_pp(:,i)    = x_d.p_d_pp;
            Phi_d(:,i)     = x_d.Phi_d;
            Phi_d_p(:,i)   = x_d.Phi_d_p;
            Phi_d_pp(:,i)  = x_d.Phi_d_pp;
            R_d(:,:,i)     = x_d.R_d;
            q_d(:,i)       = x_d.q_d;
            q_d_p(:,i)     = x_d.q_d_p;
            q_d_pp(:,i)    = x_d.q_d_pp;
            omega_d(:,i)   = x_d.omega_d;
            omega_d_p(:,i) = x_d.omega_d_p;
            alpha_d(:,i)   = x_d.alpha_d;
            alpha_d_p(:,i) = x_d.alpha_d_p;
            alpha_d_pp(:,i)= x_d.alpha_d_pp;
            rot_ax_d(:,i)  = x_d.rot_ax_d;
            alpha_d_offset(:,i) = x_d.alpha_d_offset;
            q_d_rel(:,i) = x_d.q_d_rel;

            x_k = x_kp1;

            if(t(i) >= (t_offset + T_switch) && flag == 0)
                temp = xe0;
                xe0 = xeT;
                xeT = temp;
                temp = R_init;
                R_init = R_target;
                R_target = temp;
                [rot_ax, rot_alpha_scale] = find_rotation_axis(R_init, R_target);
                alphaT = 1-param_init_pose.alphaT;
                t_offset = t_offset + T_switch;
                flag = 1;
            end
        end
    elseif(modus == 3) % 5th order polynomial
        T_start_end = T_start;
        T_start = 0;
        alpha_offset = 0;
        for i=1:N
            [x_d] = create_poly_traj(xeT, alphaT, xe0, alpha0, alpha_offset, T_start, t(i), R_init, rot_ax, rot_alpha_scale, Phi_init, delta_Phi, param_traj_poly);
            p_d(:,i)       = x_d.p_d;
            p_d_p(:,i)     = x_d.p_d_p;
            p_d_pp(:,i)    = x_d.p_d_pp;
            Phi_d(:,i)     = x_d.Phi_d;
            Phi_d_p(:,i)   = x_d.Phi_d_p;
            Phi_d_pp(:,i)  = x_d.Phi_d_pp;
            R_d(:,:,i)     = x_d.R_d;
            q_d(:,i)       = x_d.q_d;
            q_d_p(:,i)     = x_d.q_d_p;
            q_d_pp(:,i)    = x_d.q_d_pp;
            omega_d(:,i)   = x_d.omega_d;
            omega_d_p(:,i) = x_d.omega_d_p;
            alpha_d(:,i)   = x_d.alpha_d;
            alpha_d_p(:,i) = x_d.alpha_d_p;
            alpha_d_pp(:,i)= x_d.alpha_d_pp;
            rot_ax_d(:,i)  = x_d.rot_ax_d;
            alpha_d_offset(:,i) = x_d.alpha_d_offset;
            q_d_rel(:,i) = x_d.q_d_rel;

            if(t(i) >= T_start_end && flag == 0)
                temp = xe0;
                xe0 = xeT;
                xeT = temp;
                alphaT = 1-param_init_pose.alphaT;
                alpha0 = 1-param_init_pose.alpha0;
                T_start = T_start_end;
                flag = 1;
            end
        end
    elseif(modus == 4) % smooth sinus [ Orientation: TODO ]
        alpha_offset = 0;
        for i=1:N
            x_d = create_sinus_traj(xeT, xe0, t(i), R_init, rot_ax, rot_alpha_scale, alpha_offset, Phi_init, delta_Phi, param_traj_sin_poly);
            p_d(:,i)       = x_d.p_d;
            p_d_p(:,i)     = x_d.p_d_p;
            p_d_pp(:,i)    = x_d.p_d_pp;
            Phi_d(:,i)     = x_d.Phi_d;
            Phi_d_p(:,i)   = x_d.Phi_d_p;
            Phi_d_pp(:,i)  = x_d.Phi_d_pp;
            R_d(:,:,i)     = x_d.R_d;
            q_d(:,i)       = x_d.q_d;
            q_d_p(:,i)     = x_d.q_d_p;
            q_d_pp(:,i)    = x_d.q_d_pp;
            omega_d(:,i)   = x_d.omega_d;
            omega_d_p(:,i) = x_d.omega_d_p;
            alpha_d(:,i)   = x_d.alpha_d;
            alpha_d_p(:,i) = x_d.alpha_d_p;
            alpha_d_pp(:,i)= x_d.alpha_d_pp;
            rot_ax_d(:,i)  = x_d.rot_ax_d;
            alpha_d_offset(:,i) = x_d.alpha_d_offset;
            q_d_rel(:,i) = x_d.q_d_rel;
        end
    else
        error('modus have to be 1 (stabilize equilibrium), 2 (5th order differential filter), 3 (5th order polynomial) or 4 (smooth sinus)');
    end

    param_trajectory = struct;
    param_trajectory.t         = reshape(t, N, 1);
    param_trajectory.p_d       = p_d;
    param_trajectory.p_d_p     = p_d_p;
    param_trajectory.p_d_pp    = p_d_pp;
    param_trajectory.Phi_d     = Phi_d;
    param_trajectory.Phi_d_p   = Phi_d_p;
    param_trajectory.Phi_d_pp  = Phi_d_pp;
    param_trajectory.R_d       = R_d;
    param_trajectory.q_d       = q_d;
    param_trajectory.q_d_p     = q_d_p;
    param_trajectory.q_d_pp    = q_d_pp;
    param_trajectory.omega_d   = omega_d;
    param_trajectory.omega_d_p = omega_d_p;
    param_trajectory.alpha_d   = alpha_d;
    param_trajectory.alpha_d_p = alpha_d_p;
    param_trajectory.alpha_d_pp= alpha_d_pp;
    param_trajectory.rot_ax_d  = rot_ax_d;
    param_trajectory.alpha_d_offset = alpha_d_offset;
    param_trajectory.q_d_rel = q_d_rel;

end