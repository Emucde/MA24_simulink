function [best_q, best_f_val] = inverse_kinematics(param_robot, xe0, q_d, Q1, Q2, Q3, Q4, tolerance, random_q0_count, ct_ctrl_param)
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
    % - q_d: a 7x1 vector representing the desired q position, if not necessary
    %        set it to (lb+ub)/2 = q_n. By using that parameter it is possible
    %        to get more useful solutions.
    % - Q1: a 6x6 matrix representing a weighting factor for the end-effector
    %       position error
    % - Q2: a scalar representing a weighting factor for
    %       the manipulability measure
    % - Q3: a 7x7 matrix representing a
    %       weighting factor for the joint angles
    % - Q4: a 7x7 matrix
    %       representing a weighting factor for the nonlinear spring occured by
    %       each joint angle (increase this weight to avoid starting in limits,
    %       which lead to instabilites due to the huge nonlinear spring
    %       forces.)
    % - tolerance: a scalar representing the tolerance for the position error
    %              between the calculated end-effector position and the desired
    %              end-effector position
    % - random_q0_count: amount of random q0 startvalues of fmincon
    % - ct_ctrl_param: struct defined by ct controller in parameters.m
    %
    % Outputs:
    % - best_q: a 7x1 vector representing the joint angles that maximize the
    %           cost function. To reduce the influence of the start value
    %           q_0_init of fmincon, only the best solution of all random
    %           values q_0_init is returned.
    % - best_f_val: lowest value out of all cost function produced by the
    %               random values q_0_init.
        
        n = param_robot.n_DOF; % Define the number of degrees of freedom.

        % Define the constraints.
        A = []; % Define the matrix for the linear inequality constraints.
        b = []; % Define the vector for the linear inequality constraints.
        Aeq = []; % Define the matrix for the linear equality constraints.
        beq = []; % Define the vector for the linear equality constraints.
        lb = param_robot.q_limit_lower; % Define the lower bounds for the joint angles.
        ub = param_robot.q_limit_upper; % Define the upper bounds for the joint angles.
        
        % Define the cost function as a combination of position error and manipulability error.
        nl_spring = @(q) nl_spring_force(q, ct_ctrl_param, param_robot);
        x_pos_err = @(q) forward_kinematics(q, param_robot) - xe0;
        fun = @(q) 1/2*dot(x_pos_err(q), Q1*x_pos_err(q)) ... 
                 + 1/2*Q2*(1/calc_manipulability(q, param_robot))^2 ...
                 + 1/2*dot(q-q_d, Q3*(q-q_d)) ...
                 + 1/2*dot(nl_spring(q), Q4*nl_spring(q));
    
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

                xe0_act = forward_kinematics(best_q, param_robot);
                if(norm(xe0_act - xe0) < tolerance) % Check if the position error between the calculated end-effector position and the desired end-effector position is within a tolerance.
                    fprintf(">>Solution found!<<\n"); % Print a message indicating that the desired end-effector position is found.
                    break_flag = 1;
                    break; % Exit the loop.
                end
            end
        end
    
        % Print the results.
        current_err = norm(xe0_act - xe0);
        if(break_flag == 0)
            fprintf(2, "\n! Warning ! Tolerance too small, no solution found!\n\n");
            fprintf(2, 'Pose Error:\n\t(current err) %d > %d (tolerance)\n', current_err, tolerance);
        else
            fprintf('Pose Error:\n\t(current err) %d < %d (tolerance)\n', current_err, tolerance);
        end
        
    
        manipulability = calc_manipulability(best_q, param_robot); % Calculate and print the manipulability for the joint angles corresponding to the maximum manipulability.
        decimalPlaces = 4;
        fprintf('Manipulability for the optimal joint angles:\n\t%.*f\n', decimalPlaces, manipulability);
    
        xe0_opt = forward_kinematics(best_q, param_robot)';
        decimalPlaces = 3;
        fprintf("Joint angles corresponding to the maximum manipulability:\n\t[%.*f, %.*f, %.*f, %.*f, %.*f, %.*f, %.*f]'\n", decimalPlaces, best_q(1), decimalPlaces, best_q(2), decimalPlaces, best_q(3), decimalPlaces, best_q(4), decimalPlaces, best_q(5), decimalPlaces, best_q(6), decimalPlaces, best_q(7));
        decimalPlaces = 3;
        fprintf('Optimal end-effector position:\n\t%.*f, %.*f, %.*f, %.*f, %.*f, %.*f\n', decimalPlaces, xe0_opt(1), decimalPlaces, xe0_opt(2), decimalPlaces, xe0_opt(3), decimalPlaces, xe0_opt(4), decimalPlaces, xe0_opt(5), decimalPlaces, xe0_opt(6));
        decimalPlaces = 3;
        fprintf('Desired end-effector position:\n\t%.*f, %.*f, %.*f, %.*f, %.*f, %.*f\n', decimalPlaces, xe0(1), decimalPlaces, xe0(2), decimalPlaces, xe0(3), decimalPlaces, xe0(4), decimalPlaces, xe0(5), decimalPlaces, xe0(6));
    end
    
    function y_W_E = forward_kinematics(q, param)
        H = hom_transform_endeffector(q, param);
        R_W_E = H(1:3,1:3);
        y_t_W_E = H(1:3,4);
        y_W_E = [y_t_W_E; rotm2eul(R_W_E, 'XYZ')'];
    end
    
    function val_out = norm_pi(val_in)
        val_in(val_in > pi) = val_in(val_in > pi) - 2*pi;
        val_out = val_in;
    end
    
    function [w] = calc_manipulability(q, param)
        J = geo_jacobian_endeffector(q, param);
        w = abs(sqrt(det(J*J')));
    end