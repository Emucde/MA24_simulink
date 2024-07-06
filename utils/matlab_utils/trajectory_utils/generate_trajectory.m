function [param_traj_data] = generate_trajectory(param_traj, init_bus_param, traj_settings)
    N_traj = param_traj.N_traj;
    N_data = traj_settings.N_data;
    t = traj_settings.t;
    traj_mode = traj_settings.traj_mode;
    param_traj_data = param_traj_data_fun(traj_settings, 'init');

    for i = 1:N_traj
        tic;
        traj_type = param_traj.traj_type(i);

        for j=1:N_data
            if(traj_type == traj_mode.equilibrium)
                x_d = create_equilibrium_traj(traj_type, t(j), param_traj, init_bus_param);
            elseif(traj_type == traj_mode.differential_filter)
                if(j == 1)
                    start_index = param_traj.start_index(i);
                    x_k = [ param_traj.pose(1,start_index); 0; 0; 0; 0; 0; ...
                            param_traj.pose(2,start_index); 0; 0; 0; 0; 0; ...
                            param_traj.pose(3,start_index); 0; 0; 0; 0; 0; ...
                            param_traj.alpha(start_index); 0; 0; 0; 0; 0 ];
                end
                [x_d, x_kp1] = create_diff_filter_traj(traj_type, t(j), x_k, param_traj, init_bus_param);
                x_k = x_kp1;
            elseif(traj_type == traj_mode.polynomial)
                x_d = create_poly_traj(traj_type, t(j), param_traj, init_bus_param);
            elseif(traj_type == traj_mode.sinus)
                x_d = create_sinus_traj(traj_type, t(j), param_traj, init_bus_param);
            else
                error('traj_type have to be 1 (stabilize equilibrium), 2 (5th order differential filter), 3 (5th order polynomial) or 4 (smooth sinus)');
            end

            param_traj_data.p_d(           :, j,    i) = x_d.p_d;
            param_traj_data.p_d_p(         :, j,    i) = x_d.p_d_p;
            param_traj_data.p_d_pp(        :, j,    i) = x_d.p_d_pp;
            param_traj_data.Phi_d(         :, j,    i) = x_d.Phi_d;
            param_traj_data.Phi_d_p(       :, j,    i) = x_d.Phi_d_p;
            param_traj_data.Phi_d_pp(      :, j,    i) = x_d.Phi_d_pp;
            param_traj_data.R_d(           :, :, j, i) = x_d.R_d;
            param_traj_data.q_d(           :, j,    i) = x_d.q_d;
            param_traj_data.q_d_p(         :, j,    i) = x_d.q_d_p;
            param_traj_data.q_d_pp(        :, j,    i) = x_d.q_d_pp;
            param_traj_data.omega_d(       :, j,    i) = x_d.omega_d;
            param_traj_data.omega_d_p(     :, j,    i) = x_d.omega_d_p;
            param_traj_data.alpha_d(       :, j,    i) = x_d.alpha_d;
            param_traj_data.alpha_d_p(     :, j,    i) = x_d.alpha_d_p;
            param_traj_data.alpha_d_pp(    :, j,    i) = x_d.alpha_d_pp;
            param_traj_data.rot_ax_d(      :, j,    i) = x_d.rot_ax_d;
            param_traj_data.alpha_d_offset(:, j,    i) = x_d.alpha_d_offset;
            param_traj_data.q_d_rel(       :, j,    i) = x_d.q_d_rel;
        end
        disp(['create_trajectories.m: Execution Time for Trajectory Calculation: ', sprintf('%f', toc), 's']);
    end
end