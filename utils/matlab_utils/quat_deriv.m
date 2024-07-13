function [q_dot, q_ddot, Q] = quat_deriv(q, omega, omega_p)
    % Calculate the first and second derivative of a quaternion given its angular velocity
    %   q: Quaternion (1x4)
    %   omega: Angular velocity (3x1)
    %   q_dot: Derivative of the quaternion (1x4)

    % Calculate the quaternion's components

    Q = @(q) 0.5 * [ -q(2) -q(3) -q(4); ...
                      q(1)  q(4) -q(3); ...
                     -q(4)  q(1)  q(2); ...
                      q(3) -q(2)  q(1)];
    % q_eta^T = -0.5 * [q2; q3; q4]^T = -0.5 * epsilon^T
    % Q_epsilon = 0.5 * ( q1*eye(3) - skew([q2; q3; q4]) ) = 0.5 * ( eta*eye(3) - skew(epsilon) )
    % Q = [q_eta^T; Q_epsilon]

    % Calculate the derivative of the quaternion
    q_dot = Q(q) * omega;
    q_ddot = Q(q_dot) * omega + Q(q) * omega_p;
end