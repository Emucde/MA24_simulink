function [q_dot, Q] = quat_deriv(q, omega)
    % Calculate the derivative of a quaternion given its angular velocity
    %   q: Quaternion (1x4)
    %   omega: Angular velocity (3x1)
    %   q_dot: Derivative of the quaternion (1x4)

    % Calculate the quaternion's components
    q1 = q(1); % = eta
    q2 = q(2); % )
    q3 = q(3); % } = epsilon
    q4 = q(4); % )

    Q =  0.5 * [ -q2 -q3 -q4; ...
                  q1  q4 -q3; ...
                 -q4  q1  q2; ...
                  q3 -q2  q1];
    % q_eta^T = -0.5 * [q2; q3; q4]^T = -0.5 * epsilon^T
    % Q_epsilon = 0.5 * ( q1*eye(3) - skew([q2; q3; q4]) ) = 0.5 * ( eta*eye(3) - skew(epsilon) )
    % Q = [q_eta^T; Q_epsilon]

    % Calculate the derivative of the quaternion
    q_dot = Q * omega;
end