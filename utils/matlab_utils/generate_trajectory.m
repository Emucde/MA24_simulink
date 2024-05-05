function [param_trajectory] = generate_trajectory(t, modus, xe0, xeT, R_init, rot_ax, rot_alpha_scale, T_start, param_traj_filter, param_traj_poly, param_traj_sin_poly, param_traj_allg)

    N = length(t);
    p_d       = zeros(3, N);
    p_d_p     = zeros(3, N);
    p_d_pp    = zeros(3, N);
    R_d       = zeros(3, 3, N);
    q_d       = zeros(4, N);
    omega_d   = zeros(3, N);
    omega_d_p = zeros(3, N);

    flag=0;

    if(modus == 1) % stabilize equilibrium
        for i=1:N
            x_d = create_equilibrium_traj(xeT);
            p_d(:,i)       = x_d.p_d;
            p_d_p(:,i)     = x_d.p_d_p;
            p_d_pp(:,i)    = x_d.p_d_pp;
            R_d(:,:,i)     = x_d.R_d;
            q_d(:,i)       = x_d.q_d;
            omega_d(:,i)   = x_d.omega_d;
            omega_d_p(:,i) = x_d.omega_d_p;
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
        T_switch = param_traj_allg.T_switch;
        for i=1:N
            [x_d, x_kp1] = create_diff_filter_traj(xeT, x_k, R_init, rot_ax, rot_alpha_scale, param_traj_filter);
            p_d(:,i)       = x_d.p_d;
            p_d_p(:,i)     = x_d.p_d_p;
            p_d_pp(:,i)    = x_d.p_d_pp;
            R_d(:,:,i)     = x_d.R_d;
            q_d(:,i)       = x_d.q_d;
            omega_d(:,i)   = x_d.omega_d;
            omega_d_p(:,i) = x_d.omega_d_p;
            x_k = x_kp1;

            if(t(i) >= (t_offset + T_switch))
                temp = xe0;
                xe0 = xeT;
                xeT = temp;

                t_offset = t_offset + T_switch;
            end
        end
    elseif(modus == 3) % 5th order polynomial
        T_start_end = T_start;
        T_start = 0;
        for i=1:N
            x_d = create_poly_traj(xeT, xe0, T_start, t(i), R_init, rot_ax, rot_alpha_scale, param_traj_poly);
            p_d(:,i)       = x_d.p_d;
            p_d_p(:,i)     = x_d.p_d_p;
            p_d_pp(:,i)    = x_d.p_d_pp;
            R_d(:,:,i)     = x_d.R_d;
            q_d(:,i)       = x_d.q_d;
            omega_d(:,i)   = x_d.omega_d;
            omega_d_p(:,i) = x_d.omega_d_p;
            if(t(i) >= T_start_end && flag == 0)
                temp = xe0;
                xe0 = xeT;
                xeT = temp;
                T_start = T_start_end;
                flag = 1;
            end
        end
    elseif(modus == 4) % smooth sinus
        for i=1:N
            x_d = create_sinus_traj(xeT, xe0, t(i), R_init, param_traj_sin_poly);
            p_d(:,i)       = x_d.p_d;
            p_d_p(:,i)     = x_d.p_d_p;
            p_d_pp(:,i)    = x_d.p_d_pp;
            R_d(:,:,i)     = x_d.R_d;
            q_d(:,i)       = x_d.q_d;
            omega_d(:,i)   = x_d.omega_d;
            omega_d_p(:,i) = x_d.omega_d_p;
        end
    else
        error('modus have to be 1 (stabilize equilibrium), 2 (5th order differential filter), 3 (5th order polynomial) or 4 (smooth sinus)');
    end

    % result = arrayfun(@(time) struct2cell(create_sinus_traj(xeT, xe0, time, param_traj_sin_poly)), t, 'UniformOutput', false);
    % result = cell2mat([result{:}]);
    % param_trajectory = struct;
    % param_trajectory.t         = result(1, :);
    % param_trajectory.p_d       = result( 0 + (1:3), :);
    % param_trajectory.p_d_p     = result( 3 + (1:3), :);
    % param_trajectory.p_d_pp    = result( 6 + (1:3), :);
    % param_trajectory.q_d       = result( 9 + (1:4), :);
    % param_trajectory.omega_d   = result( 13 + (1:3), :);
    % param_trajectory.omega_d_p = result( 16 + (1:3), :);

    param_trajectory.t         = reshape(t, N, 1);
    param_trajectory.p_d       = p_d;
    param_trajectory.p_d_p     = p_d_p;
    param_trajectory.p_d_pp    = p_d_pp;
    param_trajectory.R_d       = R_d;
    param_trajectory.q_d       = q_d;
    param_trajectory.omega_d   = omega_d;
    param_trajectory.omega_d_p = omega_d_p;

end