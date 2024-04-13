function tau = compute_tau(q, dq, ddq, param)
  % This function computes the joint torques (tau) for a robot manipulator.

  % Inputs:
  %   q (n x 1): The robot joint positions (in radians or meters, depending on the robot).
  %   dq (n x 1): The robot joint velocities (in radians/s or meters/s).
  %   ddq (n x 1): The robot joint accelerations (in radians/s^2 or meters/s^2).
  %   param: A structure containing robot parameters specific to the manipulator.
  %         The structure fields and their content will depend on the robot.
  %
  % Outputs:
  %   tau (n x 1): The joint torques required to achieve the desired accelerations (ddq)
  %                given the current positions (q) and velocities (dq).

  % Calculate individual components of the robot dynamics equation
  M = inertia_matrix(q, param);  % Inertia matrix
  C = coriolis_matrix(q, dq, param);  % Coriolis and centrifugal terms
  g = gravitational_forces(q, param);  % Gravitational forces

  % Robot dynamics equation: tau = M*ddq + C*dq + g
  tau = M * ddq + C * dq + g;
end