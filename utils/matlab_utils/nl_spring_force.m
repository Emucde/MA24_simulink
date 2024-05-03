function [f_kn] = nl_spring_force(q, ct_ctrl_param, param_robot)
    % Calculates the nonlinear spring force based on the joint position and
    % the robots limitations
    
    % percentage of joint range when the NL spring should start...
    nl_spring_threshold = ct_ctrl_param.nl_spring_threshold;
    k_n_nl = ct_ctrl_param.k_n_nl;

    q_min = param_robot.q_limit_lower;
    q_max = param_robot.q_limit_upper;
    
    delta_q = (q_max - q_min) .* nl_spring_threshold;
    
    f_low = k_n_nl * (1 ./ (q-q_min).^2 - 1./(delta_q).^2);
    f_up = k_n_nl * (1 ./ (delta_q).^2 - 1./(q_max-q).^2);
    
    f_kn =  ((q > q_min) & (q < (q_min + delta_q))) .* f_low + ...
            ((q > (q_max - delta_q)) & (q < q_max)) .* f_up;
    f_kn(q < q_min) = NaN;
    f_kn(q > q_max) = NaN;
end

