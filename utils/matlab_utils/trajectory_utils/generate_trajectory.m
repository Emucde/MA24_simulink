function [param_trajectory] = generate_trajectory(t, modus, param_traj, init_bus_param)

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

    traj_init = param_traj.traj_init;
    N_traj = traj_init.N_traj;
    
    x_d_cell = cell(1, N);
    %for i = 1:N_traj
    i = modus;
        for j=1:N
            traj_type = traj_init.traj_type(i);
            if(traj_type == 1)
                x_d_cell{j} = create_equilibrium_traj(i, t(j), param_traj, init_bus_param);
            elseif(traj_type == 2)
                if(j == 1)
                    start_index = traj_init.start_index(i);
                    x_k = [ traj_init.pose(1,start_index); 0; 0; 0; 0; 0; ...
                            traj_init.pose(2,start_index); 0; 0; 0; 0; 0; ...
                            traj_init.pose(3,start_index); 0; 0; 0; 0; 0; ...
                            traj_init.alpha(start_index); 0; 0; 0; 0; 0 ];
                end
                [x_d_cell{j}, x_kp1] = create_diff_filter_traj(i, t(j), x_k, param_traj, init_bus_param);
                x_k = x_kp1;
            elseif(traj_type == 3)
                x_d_cell{j} = create_poly_traj(i, t(j), param_traj, init_bus_param);
            elseif(traj_type == 4)
                x_d_cell{j} = create_sinus_traj(i, t(j), param_traj, init_bus_param);
            else
                error('traj_type have to be 1 (stabilize equilibrium), 2 (5th order differential filter), 3 (5th order polynomial) or 4 (smooth sinus)');
            end
        end
    %end

    %{
    flag=0;

    x_d_cell = cell(1, N);
    if(modus == 1) % stabilize equilibrium´
        for i=1:N
            x_d_cell{i} = create_equilibrium_traj(param_traj, init_bus_param);
            if(t(i) >= param_traj.pose.T_start && flag == 0)
                temp = param_traj.pose.xe0;
                param_traj.pose.xe0 = param_traj.pose.xeT;
                param_traj.pose.xeT = temp;
                flag = 1;
            end
        end
    elseif(modus == 2) % 5th order differential filter
        x_k = [ param_traj.pose.xe0(1); 0; 0; 0; 0; 0; ...
                param_traj.pose.xe0(2); 0; 0; 0; 0; 0; ...
                param_traj.pose.xe0(3); 0; 0; 0; 0; 0; ...
                param_traj.pose.alpha0; 0; 0; 0; 0; 0 ];
        t_offset = 0;
        for i=1:N
            [x_d_cell{i}, x_kp1] = create_diff_filter_traj(x_k, param_traj, init_bus_param);
            x_k = x_kp1;

            if(t(i) >= (t_offset + param_traj.diff_filter.T_switch) && flag == 0)
                temp = param_traj.pose.xe0;
                param_traj.pose.xe0 = param_traj.pose.xeT;
                param_traj.pose.xeT = temp;

                param_traj.pose.alphaT = 1-param_traj.pose.alphaT;
                t_offset = t_offset + param_traj.diff_filter.T_switch;
                flag = 1;
            end
        end
    elseif(modus == 3) % 5th order polynomial
        T_start_end = param_traj.pose.T_start;
        param_traj.pose.T_start = 0;
        for i=1:N
            x_d_cell{i} = create_poly_traj(t(i), param_traj, init_bus_param);

            if(t(i) >= T_start_end && flag == 0)
                temp = param_traj.pose.xe0;
                param_traj.pose.xe0 = param_traj.pose.xeT;
                param_traj.pose.xeT = temp;
                param_traj.pose.alphaT = 1-param_traj.pose.alphaT;
                param_traj.pose.alpha0 = 1-param_traj.pose.alpha0;
                param_traj.pose.T_start = T_start_end;
                flag = 1;
            end
        end
    elseif(modus == 4) % smooth sinus [ Orientation: TODO ]
        for i=1:N
            x_d_cell{i} = create_sinus_traj(t(i), param_traj, init_bus_param);
        end
    else
        error('modus have to be 1 (stabilize equilibrium), 2 (5th order differential filter), 3 (5th order polynomial) or 4 (smooth sinus)');
    end
%}
    for i=1:N
        x_d = x_d_cell{i};
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