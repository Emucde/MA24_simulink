function JJ_red_colin = calc_collinearity_reduced(q_reduced, param_robot)
    % This function calculates the collinearity matrix for a reduced set of
    % joint angles q_reduced, given the robot parameters in param_robot.
    q = param_robot.q_0_ref;
    n_indices = param_robot.n_indices;
    q(n_indices) = q_reduced;

    J = geo_jacobian_endeffector_py(q);
    J_red = J(:, n_indices);

    J_red_tilde = J_red./vecnorm(J_red);
    JJ_red_colin = J_red_tilde'*J_red_tilde;
end