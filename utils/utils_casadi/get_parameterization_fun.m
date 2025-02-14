import casadi.*;

if(parametric_type == parametric_mode.polynomial)
    t0 = SX.sym('t0');
    x0 = SX.sym('x0', 2*n_red, 1);
    q0 = x0(1:n_red);
    q1 = x0(n_red+1:end);

    % theta0 = SX.sym('theta', n_red, 3);
    % q = Function('q_scalar', {x0, t0, theta0}, {q0 + q1*t0 + 1/2*theta0(:, 1)*t0^2 + 1/6*theta0(:, 2)*t0^3 + 1/12*theta0(:, 3)*t0^4});
    % q_p = Function('q_scalar_p', {x0, t0, theta0}, {q1 + theta0(:, 1)*t0 + 1/2*theta0(:, 2)*t0^2 + 1/3*theta0(:, 3)*t0^3});
    % q_pp = Function('q_scalar_pp', {t0, theta0}, {theta0(:, 1) + theta0(:, 2)*t0 + theta0(:, 3)*t0^2});
    
    theta0 = SX.sym('theta_0', n_red, parametric_order+1);
    qq_pp = SX(0);
    qq_p = q1;
    qq = q0 + q1*t0;
    for i=0:parametric_order
        qq_pp = qq_pp + theta0(:, i+1)*t0^i;
        qq_p = qq_p + theta0(:, i+1)*t0^(i+1)/(i+1);
        qq = qq + theta0(:, i+1)*t0^(i+2)/((i+1)*(i+2));
    end
    qq_pp = cse(qq_pp);
    qq_p = cse(qq_p);
    qq = cse(qq);
    
    q = Function('q', {x0, t0, theta0}, {qq});
    q_p = Function('q_p', {x0, t0, theta0}, {qq_p});
    q_pp = Function('q_pp', {t0, theta0}, {qq_pp});
elseif(parametric_type == parametric_mode.chebyshev)
    t0 = SX.sym('t0');
    x0 = SX.sym('x0', 2*n_red, 1);
    q0 = x0(1:n_red);
    q1 = x0(n_red+1:end);

    T_max = (N_MPC-1)*param_global.Ta;
    t_hat = (2*t0 - T_max)/T_max;

    % first kind
    % T0 = SX(1);
    % T0_I = t_hat;
    % TO_II = 1/2*t_hat^2;

    % T1 = t_hat;
    % T1_I = 1/2*t_hat^2;
    % T1_II = 1/6 * t_hat^3;

    % T2 = 2*t_hat^2 - 1;
    % T2_I = 2/3 * t_hat^3 - t_hat;
    % T2_II = 1/6 * t_hat^4 - 1/2 * t_hat^2;

    % T3 = 4*t_hat^3 - 3*t_hat;
    % T3_I = t_hat^4 - 3/2*t_hat^2;
    % T3_II = 1/5*t_hat^5 - 1/2*t_hat^3;

    % second kind
    T0 = SX(1);
    T0_I = t_hat;
    TO_II = 1/2*t_hat^2;

    T1 = 2*t_hat;
    T1_I = t_hat^2;
    T1_II = 1/3 * t_hat^3;

    T2 = 4*t_hat^2 - 1;
    T2_I = 4/3 * t_hat^3 - t_hat;
    T2_II = 1/3 * t_hat^4 - 1/2 * t_hat^2;

    T3 = 8*t_hat^3 - 4*t_hat;
    T3_I = 2*t_hat^4 - 2*t_hat^2;
    T3_II = 2/5*t_hat^5 - 2/3*t_hat^3;


    T_arr = {T0, T1, T2, T3};
    TI_arr = {T0_I, T1_I, T2_I, T3_I};
    TII_arr = {TO_II, T1_II, T2_II, T3_II};

    theta0 = SX.sym('theta_0', n_red, parametric_order+1);
    qq_pp = SX(0);
    qq_p = SX(0);
    qq = SX(0);
    for i=0:parametric_order
        qq_pp = qq_pp + theta0(:, 1 + (i)) * T_arr{1 + (i)};
        qq_p = qq_p + theta0(:, 1 + (i)) * TI_arr{1 + (i)};
        qq = qq + theta0(:, 1 + (i)) * TII_arr{1 + (i)};
    end
    qq_pp = cse(qq_pp);
    qq_p = cse(qq_p);
    qq = cse(qq);

    qq_pp_fun = Function('qq_pp_fun', {t0, theta0}, {qq_pp});
    qq_p_fun = Function('qq_p_fun', {t0, theta0}, {qq_p});
    qq_fun = Function('qq_fun', {t0, theta0}, {qq});

    % add the initial conditions
    c1 = cse( q1 - qq_p_fun(0, theta0) );
    c0 = cse( q0 - c1*t0 - qq_fun(0, theta0) );
    c0_plus_c1t = cse( q0 - qq_fun(0, theta0) );

    q = Function('q', {x0, t0, theta0}, {simplify( c0_plus_c1t + qq )});
    q_p = Function('q_p', {x0, t0, theta0}, {simplify( c1 + qq_p )});
    q_pp = Function('q_pp', {t0, theta0}, {simplify( qq_pp )});

    % Probe:
    % simplify(q_p(x0, 0, theta0));
    % simplify(q(x0, 0, theta0));
    % BUT WHY DOES IT NOT WORK?!
    warning('The chebyshev method does not work!');
else
    error('parametric_type not supported');
end