function [x_d] = create_equilibrium_traj(x_target, R_init, Phi_target)

    R_act = quat2rotm_v2(x_target(4:7));

    x_d.p_d    = x_target(1:3);
    x_d.p_d_p  = zeros(3,1);
    x_d.p_d_pp = zeros(3,1);

    omega_d   = zeros(3,1);
    omega_d_p = zeros(3,1);
    q_d   = rotation2quaternion(R_act);
    %q_d   = rotm2quat_v3(R_act);
    q_d_p = zeros(4,1);
    q_d_pp = zeros(4,1);


    [rot_ax_d, alpha_d] = find_rotation_axis(R_init, R_act);

    x_d.p_d       = x_target(1:3);
    x_d.p_d_p     = zeros(3,1);
    x_d.p_d_pp    = zeros(3,1);
    x_d.Phi_d     = Phi_target;
    x_d.Phi_d_p   = zeros(3,1);
    x_d.Phi_d_pp  = zeros(3,1);
    x_d.R_d       = R_act;
    x_d.q_d       = q_d;
    x_d.q_d_p     = q_d_p;
    x_d.q_d_pp    = q_d_pp;
    x_d.omega_d   = omega_d;
    x_d.omega_d_p = omega_d_p;
    x_d.alpha_d   = alpha_d;
    x_d.alpha_d_p = 0;
    x_d.alpha_d_pp = 0;
    x_d.rot_ax_d = rot_ax_d;
end