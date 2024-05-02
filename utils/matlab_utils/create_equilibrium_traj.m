function [x_d] = create_equilibrium_traj(x_target)

    x_d.p_d = x_target(1:3);
    x_d.p_d_p =  zeros(3,1);
    x_d.p_d_pp = zeros(3,1);
    x_d.q_d = rotation2quaternion(eul2rotm(x_target(4:6)', "ZYZ"));
    x_d.omega_d   = zeros(3,1);
    x_d.omega_d_p = zeros(3,1);

end