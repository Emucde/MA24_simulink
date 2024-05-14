function [dx] = sys_fun_SX(x, u, param, output_dir)
  arguments
    x (:,1) casadi.SX
    u (:,1) casadi.SX
    param struct
    output_dir char = './s_functions/s_functions_7dof/'
  end
    %               [x1]   [x2                  ]
    % d/dt x = d/dt [x2] = [M(x1)^(-1)*(u-C(x)-g(x1)]

    import casadi.*
    coder.allowpcode('plain'); % allows compilation of p-code file

    n = param.n_DOF;

    x1 = x(1:n);     % = q
    x2 = x(n+1:2*n); % = d/dt q

    dx1 = x2;

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

    % solve Ax = b
    % with
    % A = M(x1)
    % b = u-C(x)*x2-g(x1)
    dx2 = solve( M(x1), u - C(x1,x2)*x2 - g(x1) ); % = d^2/dt^2 q

    %M_inv = inertia_matrix_inv_casadi_SX(x1, param);
    %dx2 = M_inv*(u-C*x2-g);

    dx = [dx1;dx2];
end