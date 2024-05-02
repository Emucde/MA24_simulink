function [x_d] = create_sinus_traj(x_target, x0_target, t, param_traj_sin_poly)

    T = param_traj_sin_poly.T;
    omega = param_traj_sin_poly.omega;
    phi = param_traj_sin_poly.phi;
    if(t > T)
        s    = 1;
        s_p  = 0;
        s_pp = 0;
    else
        [s, s_p, s_pp] = trajectory_poly(t, 0, 1, T);
    end
    xT = x_target(1);
    x0 = x0_target(1);

    a = (xT - x0)/2; % width of sinus movement

    sin_t = a*sin(omega*t - phi);
    cos_t = a*cos(omega*t - phi);

    x_ref = x0 + (a + sin_t) * s;
    x_ref_p =    cos_t * omega * s   + (a + sin_t) * s_p;
    x_ref_pp =  -sin_t * omega^2 * s + cos_t * s_p * omega * 2 + (a + sin_t) * s_pp;

    x_d.p_d = [x_ref; x_target(2:3)];
    x_d.p_d_p = [x_ref_p; 0; 0];
    x_d.p_d_pp = [x_ref_pp; 0; 0];
    
    x_d.q_d = zeros(4,1);
    x_d.omega_d   = zeros(3,1);
    x_d.omega_d_p = zeros(3,1);
end