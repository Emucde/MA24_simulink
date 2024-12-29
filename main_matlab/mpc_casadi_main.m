% Init scripts
if(~exist('parameter_str', 'var')) % otherwise parameter_str is already defined from parameters_xdof
    %parameter_str = "parameters_2dof"; % default value
    parameter_str = "parameters_7dof"; % default value
end

fprintf('Start Execution of ''mpc_casadi_main.m''\n\n');

run(parameter_str); init_casadi; import casadi.*;

%dbstop if error
%% GLOBAL SETTINGS FOR MPC

% show plot functions
print_init_guess_cost_functions = false;
plot_init_guess                 = false; % plot init guess
plot_null_simu                  = false; % plot system simulation for x0 = 0, u0 = ID(x0)
convert_maple_to_casadi         = false; % convert maple functions into casadi functions
fullsimu                        = false; % make full mpc simulation and plot results
traj_select_mpc                 = 1; % (1: equilibrium, 2: 5th order diff filt, 3: 5th order poly, 4: smooth sinus)
create_init_guess_for_all_traj  = ~true; % create init guess for all trajectories
compile_sfun                    = false; % needed for simulink s-function, filename: "s_function_"+casadi_func_name
compile_matlab_sfunction        = false; % only needed for matlab MPC simu, filename: "casadi_func_name
compile_all_mpc_sfunctions      = false;
generate_realtime_udp_c_fun     = true; % create a c function for realtime udp communication
reload_parameters_m             = true; % reload parameters.m at the end (clears all variables!)
remove_sourcefiles              = false; % remove source files after compilation

% TESTING:
%compile_sfun = false;
%create_init_guess_for_all_traj = false;
%reload_parameters_m = false;

% Compile Mode:
% compile_mode = 1 | nlpsol-sfun | fast compile time | very accurate,          | sometimes slower exec
% compile_mode = 2 | opti-sfun   | slow compile time | sometimes less accurate | sometimes faster exec

MPC='MPC01';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version = 'v1'; % (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v4_kin_ref_dev )
param_casadi_fun_name.(MPC).Ts      = 5e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

MPC='MPC6';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version = 'v3_quat'; % (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v4_kin_ref_dev )
param_casadi_fun_name.(MPC).Ts      = 5e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

MPC='MPC7';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v3_rpy'; % (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v4_kin_ref_dev )
param_casadi_fun_name.(MPC).Ts      = 5e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 1; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

MPC='MPC8';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v4_kin_int'; % (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v4_kin_ref_dev )
param_casadi_fun_name.(MPC).Ts      = 5e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

MPC='MPC9';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v4_kin_int_refsys'; % (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v4_kin_ref_dev )
param_casadi_fun_name.(MPC).Ts      = 5e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

MPC='MPC10'; % TODO: qp wird nicht geprüft, ob limits eingehalten werden
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v5_kin_dev'; % (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v4_kin_ref_dev )
param_casadi_fun_name.(MPC).Ts      = 5e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
param_casadi_fun_name.(MPC).int_method = 'RK4'; % is ignored here

MPC='MPC11';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v6_kin_int_path_following'; % (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v6_kin_int_path_following )
param_casadi_fun_name.(MPC).Ts      = 5e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

MPC='MPC12';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v7_kin_int_planner'; % (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v6_kin_int_path_following )
param_casadi_fun_name.(MPC).Ts      = 5e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4  | SSPRK3 | Euler)

MPC='MPC13';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v8_kin_dev_planner'; % (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v6_kin_int_path_following )
param_casadi_fun_name.(MPC).Ts      = 10e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4  | SSPRK3 | Euler)

MPC='MPC14';
param_casadi_fun_name.(MPC).name    = MPC;
param_casadi_fun_name.(MPC).variant = 'nlpsol';
param_casadi_fun_name.(MPC).solver  = 'qrqp'; % (qrqp (sqp) | qpoases | ipopt)
param_casadi_fun_name.(MPC).version  = 'v6_kin_dev_path_following'; % (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v6_kin_int_path_following )
param_casadi_fun_name.(MPC).Ts      = 5e-3;
param_casadi_fun_name.(MPC).rk_iter = 1;
param_casadi_fun_name.(MPC).N_MPC   = 5;
param_casadi_fun_name.(MPC).compile_mode = 2; %1: nlpsol-sfun, 2: opti-sfun
param_casadi_fun_name.(MPC).fixed_parameter = false; % Weights and limits (true: fixed, false: as parameter inputs)
param_casadi_fun_name.(MPC).int_method = 'Euler'; % (RK4 | SSPRK3 | Euler)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param_casadi_fun_struct = param_casadi_fun_name.MPC8;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(compile_all_mpc_sfunctions)
    param_casadi_fun_struct_list = struct2cell(param_casadi_fun_name);
else
    param_casadi_fun_struct_list = {param_casadi_fun_struct};
end

for mpc_idx = 1 : length(param_casadi_fun_struct_list)
    param_casadi_fun_struct = param_casadi_fun_struct_list{mpc_idx};

    casadi_func_name = param_casadi_fun_struct.name;
    Ts_MPC           = param_casadi_fun_struct.Ts     ; % MUSS TRAJEKTORIE OFFLINE BERECHNEN DAMIT DAS von param_global.Ta ABWEICHEN DARF
    rk_iter          = param_casadi_fun_struct.rk_iter; % intermediate steps for runge kutta (rk_iter = 1 means no intermediate steps): DT = T_horizon_MPC/N_MPC/rk_iter;
    N_MPC            = param_casadi_fun_struct.N_MPC  ; % Number of control intervals - so lang prädiziert man N=0: keine Prädiktion
    T_horizon_MPC    = Ts_MPC*N_MPC;                    % Time horizon
    N_step_MPC       = round(Ts_MPC/param_global.Ta);   % sampling steps for trajectory
    MPC_variant      = param_casadi_fun_struct.variant;
    MPC_solver       = param_casadi_fun_struct.solver;
    MPC_version      = param_casadi_fun_struct.version;
    compile_mode     = param_casadi_fun_struct.compile_mode;
    int_method       = param_casadi_fun_struct.int_method;
    fixed_parameter  = param_casadi_fun_struct.fixed_parameter;
    weights_and_limits_as_parameter = ~fixed_parameter; %[todo, replace]

    disp(['mpc_casadi_main.m: Selected MPC: ', casadi_func_name]);
    fprintf('--------------------------------------------------------------------\n\n');

    % checks
    if mod(Ts_MPC, param_global.Ta) ~= 0
        error('mpc_casadi_main.m: Error: Result is not an integer.');
    end

    %% Convert Maple Functions to casadi functions
    if(convert_maple_to_casadi)
        %rmpath('./maple/maple_generated/fr3_7dof');
        %addpath('./maple/maple_generated/7_dof_system_fr3_MW19');
        %maple_fun_arr = {"inertia_matrix.m", "coriolis_matrix.m", ...
        %    "gravitational_forces.m", "forward_kinematics_endeffector.m"...
        %    "hom_transform_endeffector.m", "geo_jacobian_endeffector.m", ...
        %    "geo_jacobian_endeffector_p.m", "coriolis_rnea.m"};

        maple_fun_arr = {"Gamma_kin.m", "Gamma_kin_aperiod.m", "Phi_kin.m", "Phi_kin_aperiod.m"};
        maple_path = "maple/maple_generated/fr3_7dof/";
        %maple_path = "maple/maple_generated/7_dof_system_fr3_MW19/";
        create_casadi_functions('SX', maple_path, maple_fun_arr); %  script for matlab to casadi conversion
        return
    end
    %% path for init guess
    mpc_settings_path                = [s_fun_path, '/mpc_settings/'];%todo:  UP
    mpc_settings_struct_name         = "param_"+casadi_func_name;

    param_trajectory = param_traj_data_fun(traj_settings, 'get', traj_select_mpc, param_traj_data);

    %% OPT PROBLEM
    %[TODO: oben init]
    input_dir = [s_fun_path, '/casadi_functions/'];
    output_dir = s_fun_path; % needed for compile sfunction in nlpsol_opt_problem_SX_v2.m
    casadi_fun_c_header_str  = [casadi_func_name, '.c'];
    casadi_fun_h_header_str  = [casadi_func_name, '.h'];
    s_func_name              = ['s_function_', casadi_fun_c_header_str]; % final name for Simulink s-function
    s_fun_c_file_path        = [s_fun_path, '/', s_func_name];
    casadi_fun_h_header_path = [s_fun_path, '/', casadi_func_name, '.h'];
    casadi_fun_c_header_path = [s_fun_path, '/', casadi_func_name, '.c'];

    substr = '_matlab';
    MPC_matlab_name = [casadi_func_name, substr];

    %% DEFINE OPTIMIZATION PROBLEM
    if(strcmp(MPC_variant, 'opti'))
        error('mpc_casadi_main.m: Error: opti stack version currently not working!');
        opti_opt_problem;
    elseif(strcmp(MPC_variant, 'nlpsol'))
        nlpsol_opt_problem_SX_v2;
    else
        error(['mpc_casadi_main.m: Error: Variant = ', MPC_variant, ' is not valid. Should be (opti | nlpsol)']);
    end

    %% Test MPC (fast)

    if(plot_init_guess)
        HH_e_test = arrayfun(@(u) hom_transform_endeffector(x_init_guess(1:n,u), param_robot), 1:N_MPC+1, UniformOutput=false);
        HH_e_test_arr = cell2mat(HH_e_test);
        p_e_test_arr = HH_e_test_arr(1:2,4:4:(N_MPC+1)*4)';

        subplot(3,1,1)
        plot((0:N_MPC-1)*Ts_MPC, u_init_guess)
        xlabel('time (s)');
        ylabel('tau (Nm)')
        
        subplot(3,1,2);
        plot((0:N_MPC)*Ts_MPC, p_e_test_arr(:,1))
        xlabel('time (s)');
        ylabel('x pos (m)')
        
        subplot(3,1,3);
        plot((0:N_MPC)*Ts_MPC, p_e_test_arr(:,2))
        xlabel('time (s)');
        ylabel('y pos (m)')
    end

    %% Pre Simulation
    % TODO: S-funktion slx file und inputs und outputs angeben. Wird aber nicht
    % viel schneller als die eigentliche Simulation sein.
    %disp(['Bevor "if(fullsimu)" Rechenzeit: ', sprintf('%f', toc), ' s']);
    if(fullsimu)

        if(exist(MPC_matlab_name, 'file') == 3)
            matlab_sfun = str2func(MPC_matlab_name);
        else
            error(['mpc_casadi_main.m: Error: no matlab s-fun ', MPC_matlab_name,' exists!'])
        end
        
        %load("./s_functions/trajectory_data/"+"param_"+casadi_func_name+"_traj_data.mat")
        %param_trajectory = eval("param_"+casadi_func_name+"_traj_data");
        %load('traj_data.mat');

        N_traj          = length(param_trajectory.t);
        N_traj_original = N_traj - N_MPC*N_step_MPC;
        p_e_act_arr     = zeros(m, N_traj_original);
        u_k_new_arr     = zeros(n, N_traj_original);
        x_k_new_arr     = zeros(2*n, N_traj_original);
        x_k_new         = x_0_0;
        u_k_act         = u_k_0;

        % set additional needed N samples as last sample of trajectory
        % already done (see "parameter.m"):
        % traj_data = [param_trajectory.p_d; repmat(param_trajectory.p_d(end,:), N,1)];

        if(weights_and_limits_as_parameter)
            f_opt2 = @(mpc_init_reference_values, init_guess_0) f_opt(mpc_init_reference_values, init_guess_0, param_weight_init_cell);
        else
            f_opt2 = f_opt;
        end

        yt_d    = param_trajectory.p_d(:, :, traj_select_mpc);
        yt_d_p  = param_trajectory.p_d_p(:, :, traj_select_mpc);
        yt_d_pp = param_trajectory.p_d_pp(:, :, traj_select_mpc);

        if(strcmp(MPC_version, 'v3_rpy'))
            yr_d    = param_trajectory.Phi_d(:, :, traj_select_mpc);
            yr_d_p  = param_trajectory.Phi_d_p(:, :, traj_select_mpc);
            yr_d_pp = param_trajectory.Phi_d_pp(:, :, traj_select_mpc);
        elseif(strcmp(MPC_version, 'v3_quat'))
            yr_d   = param_trajectory.q_d(:, :, traj_select_mpc);
            yr_d_p = param_trajectory.omega_d(:, :, traj_select_mpc);
            yr_d_pp= param_trajectory.omega_d_p(:, :, traj_select_mpc);
        else
            error(['mpc_casadi_main.m: Error: MPC_version = ', MPC_version, ' is not valid. Should be (v3_rpy | v3_quat)']);
        end
        
        tic
        for i=1:1:N_traj_original
            xk = x_k_new;
            zk = zeros(n,1);

            idx = i : N_step_MPC : i + (N_MPC) * N_step_MPC;

            y_d_0   = [yt_d(   :,idx); yr_d(   :,idx)];
            y_d_p_0 = [yt_d_p( :,idx); yr_d_p( :,idx)];
            y_d_pp_0= [yt_d_pp(:,idx); yr_d_pp(:,idx)];

            mpc_init_reference_values = [xk(:); zk(:); y_d_0(:); y_d_p_0(:); y_d_pp_0(:)];
            try
                [u_opt, xx_full_opt_sol] = f_opt(mpc_init_reference_values, init_guess_0);
                % xx_full_opt_sol = {u, x, z, alpha};
                % xx_full_opt_sol(1:n) is u
                % xx_full_opt_sol(n+1:2*n) is tilde x0 = xk
                % xx_full_opt_sol(2*n+1:3*n) is d/dt tilde x0 = d/dt xk
                % xx_full_opt_sol(3*n+1:4*n) is tilde x1
                x_k_new = full( xx_full_opt_sol(:,3*n+1:4*n) ); % = tilde x1 = xk+1
            catch ME
                disp("error at " + (i-1)*param_global.Ta + " s ( i="+i+")")
                disp(ME.message);
            end
        
            HH_e_act = hom_transform_endeffector_py(x_k_new);
            p_e_act  = HH_e_act(1:2,4);

            u_k_new     = full(u_opt);
            x_k_new_sim = full(sim(x_init_guess(:,1), u_k_new));
            x_k_new     = x_k_new_sim(:,1);
            %x_k_new = x_init_guess(:,2); % ist nicht gleich wie obere zeile???? Fehler 10^-6

            p_e_act_arr(:, i) = p_e_act;
            u_k_new_arr(:, i) = u_k_new;
            x_k_new_arr(:, i) = x_k_new;
        end
        disp(['mpc_casadi_main.m: full simu execution time: ', num2str(toc), ' s'])

        figure;
        subplot(3,1,1);
        plot(Ts_MPC*(1:N_traj_original), u_k_new_arr);
        title('u');
        subplot(3,1,2);
        plot(Ts_MPC*(1:N_traj_original), x_k_new_arr);
        title('x');
        subplot(3,1,3);
        plot(Ts_MPC*(1:N_traj_original), p_e_act_arr);
        title('p_e');
        
    end

    % save mpc param: it is important to do it here because it is important for the trajectory
    % and init guess calculation!
    param_MPC = struct( ...
    'N',                     N_MPC, ...
    'N_step',                N_step_MPC, ...
    'Ts',                    Ts_MPC, ...
    'T_horizon',             T_horizon_MPC, ...
    'rk_iter',               rk_iter, ...
    'variant',               MPC_variant, ...
    'solver',                MPC_solver, ...
    'version',               MPC_version, ...
    'name',                  param_casadi_fun_struct.name, ...
    'int_method',            int_method, ...
    'fixed_parameter',       fixed_parameter, ...
    'traj_data_per_horizon', N_MPC+1, ...
    'traj_indices',          MPC_traj_indices, ...
    'int_times',             (MPC_traj_indices(2:end) - MPC_traj_indices(1:end-1))*param_global.Ta ...
    );

    param_MPC_settings = param_MPC;

    mergestructs = @(x,y) cell2struct([struct2cell(x);struct2cell(y)],[fieldnames(x);fieldnames(y)]);
    param_MPC = mergestructs(param_MPC, sol_indices); % wird in create_init_guess ueberschrieben... [TODO]

    eval(mpc_settings_struct_name+"=param_MPC;"); % set new struct name
    save(""+mpc_settings_path+mpc_settings_struct_name+'.mat', mpc_settings_struct_name);

    % Im unterschied zu 'overwrite_offline_traj_forced' in parameters_7dof.m werden hier nur
    % die init guess für die gerade zu kompilierende MPC erstellt. Update: Wenn der neue Prädiktionshorizont
    % länger ist als die Trajektorie, dann muss die Trajektorie verlängert werden und auch alle init guess
    % erstellt werden. Es müssen aber nur die Init guess für die aktuelle MPC erstellt werden, da alle anderen
    % Init guess für die anderen MPCS ja durch eine Trajektorienverlängerung unverändert bleiben.

    if(ceil(1 + (T_horizon_MPC + param_global.T_sim) / param_global.Ta) > traj_settings.N_data )
        overwrite_offline_traj_forced = true;
        create_trajectories;
    elseif(create_init_guess_for_all_traj)
        fprintf(['mpc_casadi_main.m: Creating initial guess for ', casadi_func_name, ' for all trajectories:\n']);
        files = struct;
        files.name = ['param_', casadi_func_name, '.mat'];
        overwrite_init_guess = true;
        create_mpc_init_guess;
    end

    %% COMPILE matlab s_function (can be used as normal function in matlab)
    if(compile_matlab_sfunction)
        % re-define same casadi function with new name
        % Da der Name von f_opt der matlab name ist und im Objekt gespeichert ist verwendet 
        % die Funktion casadi_fun_to_mex ebenfalls diesen Namen und er muss daher nicht
        % übergeben werden.
        %f_opt = Function(MPC_matlab_name, input_vars_MX, output_vars_MX);
        casadi_fun_to_mex(f_opt, [output_dir, 'maple_msfun'], MPC_matlab_name, '-O2');
        disp(['Compile time for matlab s-function: ', num2str(toc), ' s']);
    end

    %% Create c and header file for external usage of mpcs in c
    if(generate_realtime_udp_c_fun)
        mpc_c_sourcefile_path = [s_fun_path, '/mpc_c_sourcefiles/'];
        fprintf(['mpc_casadi_main.m: Creating headers for C: \n\nOutput folder for headers: ', mpc_c_sourcefile_path, '\n\n']);
        generate_casadi_types([mpc_c_sourcefile_path, 'casadi_types.h']);
        generate_mpc_config_typedef([mpc_c_sourcefile_path, 'mpc_config.h'], 'mpc_config');
        calc_udp_cfun_addresses(f_opt, f_opt_input_cell, f_opt_output_cell, mpc_c_sourcefile_path);
        generate_mpc_param_realtime_udp_c_fun(param_weight, param_MPC_settings, casadi_func_name, mpc_c_sourcefile_path, s_fun_path)

        fprintf('--------------------------------------------------------------------\n\n');
    end
end

if(reload_parameters_m)
    fprintf("mpc_casadi_main.m: Reload parameters.m:\n\n");
    mpc_casadi_main_state = 'running';
    run(parameter_str);
    cd ..
    cd MA24_simulink
    
    mpc_casadi_main_state = 'done';
end

fprintf('\nExecution of ''mpc_casadi_main.m'' finished\n');
fprintf('--------------------------------------------------------------------\n');