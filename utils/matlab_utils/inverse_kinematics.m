function [best_q, best_f_val] = inverse_kinematics(param_robot, param_inv_kin, ct_ctrl_param)
    % This function solves the inverse kinematics for a 9DOF system with the
    % goal of placing the joint angles q0(1:2) of the linear axes as close to
    % the center of the workspace as possible (also depending on q_d).
    % Once sufficiently accurate inverse kinematic joint angles are
    % found based on the specified tolerance, the iterations are terminated,
    % and the outputs are generated.
    %
    % Inputs: 
    % - param_robot: a struct containing the robot parameters - xe0: a
    %                6x1 vector representing the desired end-effector position
    %                and orientation
    %  - param_inv_kin: a struct containing the inverse kin parameters:
    %     - q_d: a 7x1 vector representing the desired q position, if not necessary
    %            set it to (lb+ub)/2 = q_n. By using that parameter it is possible
    %            to get more useful solutions.
    %     - Q_pos: a 6x6 matrix representing a weighting factor for the end-effector
    %              position error
    %     - Q_m: a scalar representing a weighting factor for
    %            the manipulability measure
    %     - Q_q: a 7x7 matrix representing a
    %            weighting factor for the joint angles
    %     - Q_nl: a 7x7 matrix
    %             representing a weighting factor for the nonlinear spring occured by
    %             each joint angle (increase this weight to avoid starting in limits,
    %             which lead to instabilites due to the huge nonlinear spring
    %             forces, not used for non redundant robots)
    %     - Q_collin: collinearity weightin matrix of J'J/(|Jj||Ji|)
    %     - tolerance: a scalar representing the tolerance for the position error
    %           between the calculated end-effector position and the desired
    %           end-effector position
    %      - random_q0_count: amount of random q0 startvalues of fmincon
    % - ct_ctrl_param: struct defined by ct controller in parameters.m
    %
    % Outputs:
    % - best_q: a 7x1 vector representing the joint angles that maximize the
    %           cost function. To reduce the influence of the start value
    %           q_0_init of fmincon, only the best solution of all random
    %           values q_0_init is returned.
    % - best_f_val: lowest value out of all cost function produced by the
    %               random values q_0_init.

        xe0 = param_inv_kin.xe0;
        q_d = param_inv_kin.q_d;
        Q1 = param_inv_kin.Q_pos;
        Q2 = param_inv_kin.Q_m;
        Q3 = param_inv_kin.Q_q;
        Q4 = param_inv_kin.Q_nl;
        K_J_struct = param_inv_kin.K_J;
        R_J_d_struct = param_inv_kin.R_J_d;
        [R_J_d, K_J] = set_collin_matrices(R_J_d_struct, K_J_struct, param_robot);
        tolerance = param_inv_kin.tolerance;
        random_q0_count = param_inv_kin.random_q0_count;

        %R_target = quat2rotm_v2(xe0(4:7));
        %xe0 = [xe0(1:3); rotm2eul(R_target, 'XYZ')'];
        
        n_indices = param_robot.n_indices;
        n = param_robot.n_red; % Define the number of degrees of freedom.

        % Define the constraints.
        A = []; % Define the matrix for the linear inequality constraints.
        b = []; % Define the vector for the linear inequality constraints.
        Aeq = []; % Define the matrix for the linear equality constraints.
        beq = []; % Define the vector for the linear equality constraints.
        lb = param_robot.q_limit_lower(n_indices); % Define the lower bounds for the joint angles.
        ub = param_robot.q_limit_upper(n_indices); % Define the upper bounds for the joint angles.
        
        % Define the cost function as a combination of position error and manipulability error.
        %if(n > 6)
        %    nl_spring = @(q) nl_spring_force(q, ct_ctrl_param, param_robot);
        %else
            nl_spring = @(q) zeros(n,1);
        %endkin_fun_err
        %x_pos_err = @(q) kin_fun(q) - xe0;
        x_pos_err = @(q) kin_fun_err_reduced(xe0, q, param_robot);
        manip_red = @(q) calc_manipulability_reduced(q, param_robot);
        R_J = @(q) calc_collinearity_reduced(q, param_robot);
        fun = @(q) 1/2*dot(x_pos_err(q), Q1*x_pos_err(q)) ... 
                 + 1/2*Q2*(manip_red(q))^2 ...
                 + 1/2*dot(q-q_d, Q3*(q-q_d)) ...
                 + 1/2*dot(nl_spring(q), Q4*nl_spring(q)) ...
                 + 1/2*sum( 1/2 * K_J .* (abs(R_J(q)) - R_J_d).^2, 'all');
    
        % Define the optimization options.
        options = optimoptions('fmincon','Display','off','Algorithm','interior-point'); % Define the options for the optimization algorithm.
        %options = optimoptions('fmincon','Display','off','Algorithm','sqp'); % Define the options for the optimization algorithm.

        % Find the maximum manipulability.
        fprintf("\nStart Inverse Kinematik Search:\n");
        break_flag = 0;
        best_f_val = inf; % Initialize the maximum manipulability value.
        best_q = zeros(n,1); % Initialize the joint angles corresponding to the maximum manipulability.
        for i=0:1:random_q0_count
            % Generate a random vector of the same size as the lower bounds vector.
            randomVector = rand(size(lb));
            q_0 = lb + randomVector .* (ub - lb); % Scale and shift the random vector to fit between the bounds and use it as the initial joint angles.
            
            % Minimize the cost function subject to the constraints to find the optimal joint angles.
            [q_opt, f_val] = fmincon(fun,q_0,A,b,Aeq,beq,lb,ub,[],options);
            
            % store best result
            if(f_val < best_f_val) % Check if the manipulability for the optimal joint angles is greater than the current maximum manipulability.
                best_f_val = f_val; % Update the maximum manipulability value.
                best_q = q_opt; % Update the joint angles corresponding to the maximum manipulability.

                xe0_err_act = x_pos_err(best_q);
                xe0_err_act(4) = 0; % ignore scalar part
                
                if(norm(xe0_err_act) < tolerance) % Check if the position error between the calculated end-effector position and the desired end-effector position is within a tolerance.
                    fprintf(">>Solution found!<<\n"); % Print a message indicating that the desired end-effector position is found.
                    break_flag = 1;
                    break; % Exit the loop.
                end
            end
        end
    
        % Print the results.
        current_err = norm(xe0_err_act);
        if(break_flag == 0)
            fprintf(2, "\n! Warning ! Tolerance too small, no solution found!\n\n");
            fprintf(2, 'Pose Error:\n\t(current err) %d > %d (tolerance)\n', current_err, tolerance);
        else
            fprintf('Pose Error:\n\t(current err) %d < %d (tolerance)\n', current_err, tolerance);
        end
        
    
        manipulability = manip_red(best_q); % Calculate and print the manipulability for the joint angles corresponding to the maximum manipulability.
        decimalPlaces = 4;
        fprintf('Manipulability for the optimal joint angles:\n\t%.*g\n', decimalPlaces, manipulability);
    
        xe0_opt_act = kin_fun_reduced(best_q, param_robot)';
        xe0_opt1 = xe0_opt_act;
        xe0_opt2 = [xe0_opt_act(1:3) -xe0_opt_act(4:7)];
        

        % zu jeder rotation gibt es zwei quaternionen...
        if(norm(xe0 - xe0_opt1) < norm(xe0 - xe0_opt2))
            xe0_opt = xe0_opt1;
        else
            xe0_opt = xe0_opt2;
        end

        decimalPlaces = 3;
        fprintf("Joint angles corresponding to the minimum manipulability:\n\t[%s]'\n", strjoin(arrayfun(@(x) sprintf('%.*f', decimalPlaces, x), best_q, 'UniformOutput', false), ', '));
        
        decimalPlaces = 3;
        fprintf("Optimal end-effector position:\n\t[%s]'\n", strjoin(arrayfun(@(x) sprintf('%.*f', decimalPlaces, x), xe0_opt, 'UniformOutput', false), ', '));
        decimalPlaces = 3;
        fprintf("Desired end-effector position:\n\t[%s]'\n", strjoin(arrayfun(@(x) sprintf('%.*f', decimalPlaces, x), xe0, 'UniformOutput', false), ', '));
    end
    
    function y_W_E = forward_kinematics(q)
        H = hom_transform_endeffector_py(q);
        R_W_E = H(1:3,1:3);
        y_t_W_E = H(1:3,4);
        y_W_E = [y_t_W_E; rotm2eul(R_W_E, 'XYZ')'];
    end
    
    function val_out = norm_pi(val_in)
        val_in(val_in > pi) = val_in(val_in > pi) - 2*pi;
        val_out = val_in;
    end
    
    function [w] = calc_manipulability(q)
        J = geo_jacobian_endeffector_py(q);
        w = abs(sqrt(det(J*J')));
    end