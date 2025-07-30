function [x_d] = create_spline_traj(traj_select, t, param_traj, init_bus_param)
% create_spline_traj - Create a spline trajectory for the given trajectory selection and time.
% Syntax:
%   [x_d] = create_spline_traj(traj_select, t, param_traj, init_bus_param)
%% Inputs:
%   traj_select - Index of the trajectory to select from param_traj.
%   t           - Current time at which the trajectory is evaluated.
%   param_traj  - Structure containing trajectory parameters including:
%                 - pose: 3D positions and orientations.
%                 - alpha: orientation angles.
%                 - rotation: rotation matrices.
%                 - rot_ax: rotation axes.
%                 - start_index: starting indices for each trajectory segment.
%                 - stop_index: stopping indices for each trajectory segment.
%                 - bspline: B-spline parameters for the trajectory.
%   init_bus_param - Initial bus parameters containing initial state information.
%% Outputs:
%   x_d         - Structure containing the desired trajectory   
    start_index = param_traj.start_index(traj_select);
    stop_index  = param_traj.stop_index(traj_select);
    t_val       = param_traj.time(start_index:stop_index);

    bspline = param_traj.bspline{traj_select};

    T = t_val(end);


    alpha0 = param_traj.alpha(start_index);

    R_init = param_traj.rotation(:, :, start_index);
    rot_ax = param_traj.rot_ax(:, start_index+1);

    if(t > T)
        alpha    = 1;
        alpha_p  = 0;
        alpha_pp = 0;
        data = C_fun(1, 2, bspline);
    else
        [alpha, alpha_p, alpha_pp] = trajectory_poly(t, 0, 1, T);
        data = C_fun(t/T, 2, bspline);
    end


    x_ref = data(:,1);
    x_ref_p = data(:,2);
    x_ref_pp = data(:,3);

    skew_ew  = skew(rot_ax);

    RR = (eye(3) + sin(alpha-alpha0)*skew_ew + (1-cos(alpha-alpha0))*skew_ew^2);
    
    % R_act    = RR*R_init; % Vormultiplikation (in find_rotation_axis wird Nachmultiplikation für RR verwendet!!)
    R_act    = R_init*RR; % Nachmultiplikation (in find_rotation_axis wird Nachmultiplikation für RR verwendet!!)

    omega_d   = alpha_p*rot_ax;
    omega_d_p = alpha_pp*rot_ax;

    q_d   = quat_R_endeffector_py(R_act);
    [q_d_p, q_d_pp] = quat_deriv(q_d, omega_d, omega_d_p);


    % w = T Phi_p -> Phi_p = T^(-1)w = T\w
    % w_p = Tp Phi_p + T Phi_pp -> Phi_pp = T^-1(w_p - Tp Phi_p) = T\(w_p - Tp Phi_p)
    
    Phi_act = rotm2rpy(R_act);
    Phi_act_p = T_rpy(Phi_act)\omega_d;
    Phi_act_pp = T_rpy(Phi_act)\(omega_d_p - T_rpy_p(Phi_act, Phi_act_p)*Phi_act_p);

    % Phi_act = rotm2eul_v2(R_act);
    % Phi_act_p = T_eul(Phi_act)\omega_d;
    % Phi_act_pp = T_eul(Phi_act)\(omega_d_p - T_eul_p(Phi_act, Phi_act_p)*Phi_act_p);

    alpha_d = alpha;
    alpha_d_p = alpha_p;
    alpha_d_pp = alpha_pp;
    rot_ax_d = rot_ax;


    x_d = init_bus_param.x_d;
    x_d.p_d       = x_ref;
    x_d.p_d_p     = x_ref_p;
    x_d.p_d_pp    = x_ref_pp;
    x_d.Phi_d     = Phi_act;
    x_d.Phi_d_p   = Phi_act_p;
    x_d.Phi_d_pp  = Phi_act_pp;
    x_d.R_d       = R_act;
    x_d.q_d       = q_d;
    x_d.q_d_p     = q_d_p;
    x_d.q_d_pp    = q_d_pp;
    x_d.omega_d   = omega_d;
    x_d.omega_d_p = omega_d_p;
    x_d.alpha_d   = alpha_d;
    x_d.alpha_d_p = alpha_d_p;
    x_d.alpha_d_pp = alpha_d_pp;
    x_d.rot_ax_d = rot_ax_d;
    x_d.alpha_d_offset = alpha0;
    x_d.q_d_rel = quat_R_endeffector_py(R_init);
end

function [CC] = spline_vec_fun(k, bspline)
    Cx = @(uu) arrayfun(@(u) C_tj_k_fun(u, 1, k, bspline), uu);
    Cy = @(uu) arrayfun(@(u) C_tj_k_fun(u, 2, k, bspline), uu);
    Cz = @(uu) arrayfun(@(u) C_tj_k_fun(u, 3, k, bspline), uu);

    CC = @(uu) [Cx(uu); Cy(uu); Cz(uu)];
end

function [CC] = C_fun(theta, k, bspline)
    i = bspline_findspan(theta, bspline);
    p = bspline.degree;
    control_points = bspline.control_points;
    CC = control_points(1+(i-p):1+(i),:)' * bspline_basisfunction(theta,i,k,bspline)';
end

function [Crow] = C_tj_k_fun(theta, row, k, bspline)
    CC = C_fun(theta, 1, bspline);
    Crow = CC(row, k+1);
end