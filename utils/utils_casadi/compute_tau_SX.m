function tau = compute_tau_SX(q, dq, ddq, param, output_dir)
  arguments
    q (:,1) casadi.SX
    dq (:,1) casadi.SX
    ddq (:,1) casadi.SX
    param struct
    output_dir char = './s_functions/s_functions_7dof/'
  end

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
  if ~exist([output_dir, 'M_fun', '.casadi'], 'file') || ...
     ~exist([output_dir, 'C_fun', '.casadi'], 'file') || ...
     ~exist([output_dir, 'g_fun', '.casadi'], 'file')
      M_SX = inertia_matrix_casadi_SX(q, param);  % Inertia matrix
      C_SX = coriolis_matrix_casadi_SX(q, dq, param);  % Coriolis and centrifugal terms
      g_SX = gravitational_forces_casadi_SX(q, param);  % Gravitational forces
      
      M = casadi.Function('M_fun', {q}, {M_SX});
      M.save([output_dir, 'M_fun', '.casadi']);
      C = casadi.Function('C_fun', {q, dq}, {C_SX});
      C.save([output_dir, 'C_fun', '.casadi']);
      g = casadi.Function('g_fun', {q}, {g_SX});
      g.save([output_dir, 'g_fun', '.casadi']);
  else
      M =  casadi.Function.load([output_dir, 'M_fun', '.casadi']);
      C =  casadi.Function.load([output_dir, 'C_fun', '.casadi']);
      g =  casadi.Function.load([output_dir, 'g_fun', '.casadi']);
  end

    % Robot dynamics equation: tau = M*ddq + C*dq + g
    tau = M(q) * ddq + C(q, dq) * dq + g(q);
  end