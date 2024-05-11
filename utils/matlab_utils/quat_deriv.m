function [q_dot, Q] = quat_deriv(q, omega)
    % Calculate the derivative of a quaternion given its angular velocity
    %   q: Quaternion (1x4)
    %   omega: Angular velocity (3x1)
    %   q_dot: Derivative of the quaternion (1x4)

    % Calculate the quaternion's components
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);

    Q = [ -q2 -q3 -q4; ...
           q1  q4 -q3; ...
          -q4  q1  q2; ...
           q3 -q2  q1];

    % Calculate the derivative of the quaternion
    q_dot = 0.5 * Q * omega;
end