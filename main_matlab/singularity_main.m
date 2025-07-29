% This script is used to find singularities of a 7-DOF robot arm using optimization techniques.
% It sets up the parameters, defines the cost function, and uses casadi to find random singularities.
% The results are saved in a CSV file for further analysis.

if(~exist('parameter_str', 'var'))
    parameters_7dof;
end

n = param_robot.n_DOF; % Define the number of degrees of freedom.
input_dir = [s_fun_path, '/casadi_functions/'];
traj_select_mpc = 1;

% Set the initial guess for the optimization problem.
% This is a random initialization within the limits of the robot's joint positions.
% q_0 = param_traj.q_0(:, traj_select_mpc)+rand(n,1);
q_0 = rand(n,1);
q_1 = q_0 + ones(n,1)*0.1;
q_0 = min(max(q_0, param_robot.q_limit_lower*0.5), param_robot.q_limit_upper*0.5);
q_1 = min(max(q_1, param_robot.q_limit_lower*0.5), param_robot.q_limit_upper*0.5);
q_0(param_robot.n_indices_fixed) = 0;
q_1(param_robot.n_indices_fixed) = 0;

H_0 = hom_transform_endeffector_py(q_0);

p_d_0 = H_0(1:3, 4);
R_d_0 = H_0(1:3, 1:3);
q_d_0 = quat_R_endeffector_py(H_0(1:3, 1:3));

% Parameters for the optimization problem.
MPC_solver = 'ipopt';
use_jit = false;

% Define the casadi optimization problem.
multiple_singular_solutions = false;
if(multiple_singular_solutions)
    opt_problem_find_multiple_singular_solutions;
else
    opt_problem_find_single_singular_solutions;
end
gen_opt_problem_test; % Generate the optimization problem

% Compile the casadi function to a mex file for performance.
coptimflags = '-Ofast -march=native -flto'; % Optimization flag for compilation

build_mex = false;
if(build_mex)
    casadi_fun_to_mex(f_opt, [s_fun_path, '/mpc_c_sourcefiles'], [s_fun_path, '/matlab_functions'], casadi_func_name, coptimflags, MPC_solver, false);
end

% Run the optimization problem to find two different singular solutions that have a small distance in joint space.
if(multiple_singular_solutions)
    % Calculate the initial guess for the optimization problem.

    N_min = 10000;
    qq_min = repmat({[q_0, q_1]}, 1, N_min);
    w_min = 100*ones(N_min, 1);
    delta_H_min = 100*ones(N_min, 1);
    delta_q_min = 100*ones(N_min, 1);
    krit_min = 100*ones(N_min, 1);

    Qt_1_ref = 0*diag([1, 1, 1]);
    Qr_1_ref = 0*diag([1, 1, 1]);
    Qt_2_ref = 0*diag([1, 1, 1]);
    Qr_2_ref = 0*diag([1, 1, 1]);
    Qt_3_ref = 1e8*diag([1, 1, 1]);
    Qr_3_ref = 1e8*diag([1, 1, 1]);
    Q4_ref = 1e1*eye(n_red); %(q1)^2
    Q5_ref = 1e1*eye(n_red);
    q1_manip_ref = 1e4;
    q2_manip_ref = 1e4;
    dq_eps_ref = 1e2;
    q_eps_ref = 1e4;
    q_scale_ref = 0.8;

    mpc_init_reference_values = [y_d_0(:); Qt_1_ref(:); Qr_1_ref(:); Qt_2_ref(:); Qr_2_ref(:); Qt_3_ref(:); Qr_3_ref(:); Q4_ref(:); Q5_ref(:); q1_manip_ref; q2_manip_ref; dq_eps_ref; q_eps_ref];
    init_guess_0 = [x_init_guess_0(:); epsilon_max; lam_x_init_guess_0(:); lam_g_init_guess_0(:)];
    qq_n_sol = zeros(n, 2);
    q_min = zeros(n, 1);
    min_idx=1;
    tic

    base_scale = 0.3+0.5*rand(1);

    % Loop to find multiple singular solutions based on the initial guess.
    % The loop runs for a large number of iterations to explore the solution space.
    % It generates random values for the parameters and checks if the new solution is better than the previous ones.
    % If a better solution is found, it updates the minimum values and prints the current iteration details.
    % The results are saved in a CSV file for further analysis.
    for i=1:600000
        q1_manip_ref = 1e6 + 1e4*rand(1);
        q2_manip_ref = 1e6 + 1e4*rand(1);
        dq_eps_ref = 1e4 + 1e3*rand(1);
        q_eps_ref = 1e4 + 1e3*rand(1);
        q_scale_ref = base_scale*rand(1);

        Q4_ref = 1e3*eye(n_red)+ 1e3*rand(1); %(q1)^2
        Q5_ref = 1e3*eye(n_red)+ 1e3*rand(1); %(q2)^2
        mpc_init_reference_values = [y_d_0(:); Qt_1_ref(:); Qr_1_ref(:); Qt_2_ref(:); Qr_2_ref(:); Qt_3_ref(:); Qr_3_ref(:); Q4_ref(:); Q5_ref(:); q1_manip_ref; q2_manip_ref; dq_eps_ref; q_eps_ref; q_scale_ref];

        q_0 = 2*pi*rand(n, 1);
        q_1 = 2*pi*rand(n, 1);
        q_0 = min(max(q_0, param_robot.q_limit_lower*q_scale_ref), param_robot.q_limit_upper*q_scale_ref);
        q_1 = min(max(q_1, param_robot.q_limit_lower*q_scale_ref), param_robot.q_limit_upper*q_scale_ref);
        q_0(param_robot.n_indices_fixed) = 0;
        q_1(param_robot.n_indices_fixed) = 0;

        H_0 = hom_transform_endeffector_py(q_0);
        p_d_0 = H_0(1:3, 4);
        q_d_0 = quat_R_endeffector_py(H_0(1:3, 1:3));
        
        y_d_0 = [p_d_0; q_d_0];
        mpc_init_reference_values(1:7) = y_d_0;

        x_init_guess_0 = [q_0(n_indices) q_1(n_indices)];
        init_guess_0(1:2*n_red) = x_init_guess_0(:);

        xsol = find_singularities(mpc_init_reference_values, init_guess_0); % Dont run this without args otherwise matlab crashes!!!
        qq_sol = reshape(full(xsol), n_red, 2);
        qq_n_sol(n_indices, :) = qq_sol;

        H1=hom_transform_endeffector_py(qq_n_sol(:,1));
        H2=hom_transform_endeffector_py(qq_n_sol(:,2));
        JJ1 = geo_jacobian_endeffector_py(qq_n_sol(:,1));
        JJ2 = geo_jacobian_endeffector_py(qq_n_sol(:,2));

        JJ1 = JJ1(:, n_indices);
        JJ2 = JJ2(:, n_indices);

        w1 = sqrt(det(JJ1 * JJ1'));
        w2 = sqrt(det(JJ2 * JJ2'));
        delta_q = norm(qq_n_sol(:,1)-qq_n_sol(:,2), 2);
        w = (w1 + w2) / 2;
        delta_H = norm(H1-H2, 2);
        krit_min_new = (1/delta_q + delta_H + 1e5*w) / 3;
        indices = find(krit_min_new < krit_min);

        if(~isempty(indices))
            [~, max_idx] = max(krit_min);
            qq_min{max_idx} = qq_n_sol;
            w_min(max_idx) = w;
            delta_H_min(max_idx) = delta_H;
            delta_q_min(max_idx) = delta_q;
            krit_min(max_idx) = krit_min_new;
            [~, min_idx] = min(krit_min);
            q_min(n_indices) = qq_min{min_idx}(n_indices,1+round(rand(1)));
            disp(['Iteration: ', num2str(i), ', krit_min: ', num2str(krit_min_new), ', w: ', num2str(w), ', delta_H: ', num2str(delta_H), ', delta_q: ', num2str(delta_q)]);
        end
    end


    % display all solutions
    for i=1:N_min
        fprintf('Solution %d:\n\n', i);
        disp('H1:');
        disp(hom_transform_endeffector_py(qq_min{i}(:,1)));
        disp('H2:');
        disp(hom_transform_endeffector_py(qq_min{i}(:,2)));
        JJ1 = geo_jacobian_endeffector_py(qq_min{i}(:,1));
        JJ2 = geo_jacobian_endeffector_py(qq_min{i}(:,2));
        w1 = sqrt(det(JJ1 * JJ1'));
        w2 = sqrt(det(JJ2 * JJ2'));
        fprintf('q1=[');fprintf('%f ', qq_min{i}(1:end-1,1));fprintf('%f];\n\n', qq_min{i}(end,1));
        fprintf('q2=[');fprintf('%f ', qq_min{i}(1:end-1,2));fprintf('%f];\n\n', qq_min{i}(end,2));
        fprintf('w1 = %f, w2 = %f\n\n', w1, w2);
        fprintf('||q1 - q2|| = %f\n\n', norm(qq_min{i}(:,1) - qq_min{i}(:,2), 2));
        fprintf('------------------------------------------------------------------------------------\n\n');
    end

    % print all solutions
    cnt=1;
    for i = 1:numel(qq_min)
        qq = qq_min{i};
        for j=0:1
            qpos_str = sprintf('%g ', qq(1:end-1, 1+j)); % Format all but the last element
            qpos_str = [qpos_str, sprintf('%f', qq(end, 1+j))]; % Append the last element with '%f' format

            fprintf('  <keyframe>');
            fprintf('<key name="q%d" qpos="%s" ctrl="0 0 0 -1.57079 0 1.57079 -0.7853"/>', cnt, qpos_str);
            fprintf('</keyframe>\n');
            cnt=cnt+1;
        end
    end
    toc;

    % save result into csv file
    current_time = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');
    fileID = fopen(sprintf('singularity_solutions_%s.csv', char(current_time)), 'w');
    for i=1:N_min
        qq = qq_min{i};
        qq = qq(:);
        fprintf(fileID, '%f, ', qq(1:end-1,1));fprintf(fileID, '%f\n', qq(end,1));
    end
    fclose(fileID);
else % Find a single singular solution
    % Calculate the initial guess for the optimization problem.
    N_min = 1000000;
    qq_min = repmat({[q_0_ref]}, 1, N_min);
    w_min = 100*ones(N_min, 1);

    q_manip_ref = 1e8;
    q_scale_ref = 0.8;
    R_x_ref = 1*eye(n_red);
    mpc_init_reference_values = [R_x_ref(:); q_manip_ref; q_scale_ref];

    init_guess_0 = [x_init_guess_0(:); lam_x_init_guess_0(:); lam_g_init_guess_0(:)];
    qq_n_sol = zeros(n, 1);
    q_min = zeros(n, 1);
    min_idx=1;
    tic

    krit_min = 100*ones(N_min, 1);

    base_scale = 0.3+0.5*rand(1);

    % Loop to find multiple singular solutions based on the initial guess.
    % The loop runs for a large number of iterations to explore the solution space.
    % It generates random values for the parameters and checks if the new solution is better than the previous ones.
    % If a better solution is found, it updates the minimum values and prints the current iteration details.
    % The results are saved in a CSV file for further analysis.
    for i=1:1000000
        q_manip_ref = 1e8 + 1e2*rand(1);
        q_scale_ref = base_scale*rand(1);
        R_x_ref = 1e0*rand(1)*eye(n_red);

        mpc_init_reference_values = [R_x_ref(:); q_manip_ref; q_scale_ref];

        q_0 = 2*pi*rand(n, 1);
        q_0 = min(max(q_0, param_robot.q_limit_lower*q_scale_ref), param_robot.q_limit_upper*q_scale_ref);
        q_0(param_robot.n_indices_fixed) = 0;

        x_init_guess_0 = [q_0(n_indices)];
        init_guess_0(1:n_red) = x_init_guess_0(:);

        xsol = find_singularities(mpc_init_reference_values, init_guess_0); % Dont run this without args otherwise matlab crashes!!!
        qq_sol = full(xsol);
        qq_n_sol(n_indices, :) = qq_sol;

        JJ1 = geo_jacobian_endeffector_py(qq_n_sol);
        JJ1_red = JJ1(:, n_indices);

        w = sqrt(det(JJ1_red * JJ1_red'));

        if(w > 1e-5)
            continue
        end

        krit_min_new = w;
        indices = find(krit_min_new < krit_min);

        if(~isempty(indices))
            [~, max_idx] = max(krit_min);
            qq_min{max_idx} = qq_n_sol;
            w_min(max_idx) = w;
            krit_min(max_idx) = krit_min_new;
            [~, min_idx] = min(krit_min);
            q_min(n_indices) = qq_min{min_idx}(n_indices,1);
            disp(['Iteration: ', num2str(i), ', krit_min: ', num2str(krit_min_new), ', w: ', num2str(w)]);
        end
    end

    current_time = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');
    fileID = fopen(sprintf('singularity_solutions_single_%s.csv', char(current_time)), 'w');
    for i=1:N_min
        qq = qq_min{i};
        qq = qq(:);
        fprintf(fileID, '%f, ', qq(1:end-1,1));fprintf(fileID, '%f\n', qq(end,1));
    end
    fclose(fileID);
end