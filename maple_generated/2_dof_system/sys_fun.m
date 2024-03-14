function [dx] = sys_fun(x, u, param)
    %               [x1]   [x2                  ]
    % d/dt x = d/dt [x2] = [M(x1)^(-1)*(u-C(x)-g(x1)]

    import casadi.*
    coder.allowpcode('plain'); % allows compilation of p-code file

    n = param.n_DOF;

    x1 = x(1:n);     % = q
    x2 = x(n+1:2*n); % = d/dt q

    dx1 = x2;

    M = inertia_matrix_casadi(x1, param);
    C = coriolis_matrix_casadi(x1, x2, param);
    g = gravitational_forces_casadi(x1, param);

    % solve Ax = b
    % with
    % A = M(x1)
    % b = u-C(x)*x2-g(x1)
    %dx2 = solve(M, u-C*x2-g); % = d^2/dt^2 q

    M_inv = inertia_matrix_inv_casadi(x1, param);
    dx2 = M_inv*(u-C*x2-g);

    dx = [dx1;dx2];
end