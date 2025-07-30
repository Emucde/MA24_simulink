% This file creates the parameters for the visualization of the 2DOF manipulator
% in Simulink. It is used in the Simulink model and the Simscape model.
% The parameters are used to visualize the robot and its components.

param_vis.T = T_sim;

Rx = @(al) [1 0 0; 0 cos(al) -sin(al); 0 sin(al) cos(al)];
Ry = @(al) [cos(al) 0 sin(al); 0 1 0; -sin(al) 0 cos(al)];
Rz = @(al) [cos(al) -sin(al) 0;sin(al) cos(al) 0; 0 0 1];

%% Trace of end-effector
param_vis.vis_point.pix = 6;                          % pixel size
param_vis.vis_point.col = [1 1 0.06667]; %[0.4, 1, 0.2];              % colour
param_vis.vis_point.opa = 1;                          % transparency
param_vis.vis_point.delta = round(param_vis.T/100/param_global.Ta);  % distance of time between points

%% Cosmetic settings

% path coords len
coord_xaxis_len = 5; %in cm
coord_yaxis_len = 5; %in cm
coord_zaxis_len = 5; %in cm

% path coords style
coord_xaxis_color = [1 0.0 0.0]; % red
coord_xaxis_opacity = 1;
coord_yaxis_color = [0 1 0.0]; % green
coord_yaxis_opacity = 1;
coord_zaxis_color = [0 0.0 1]; % blue
coord_zaxis_opacity = 1;

% Transparenzfaktor für achsen und stl files (damit man KOS sieht)
transparency_base = 1/2;

%% Robot gravity
param_robot.g_vis=[0;-param_robot.g;0];

%% Simscape Settings
param_robot.R_0 = eye(3);
param_robot.p_0 = [0 0 0];

param_robot.holder.R_0 = eye(3);
param_robot.holder.d_0 = [0 param_robot.l__I 0];
param_robot.holder.R__joint = param_robot.holder.R_0;
param_robot.holder.d__joint = param_robot.holder.d_0;
param_robot.holder.R__inertiaOrigin = eye(3);
param_robot.holder.d__inertiaOrigin = [0 0 0];
param_robot.holder.d__inertiaMatrixDiag = param_robot.inertia_matrix_sI;
param_robot.holder.m = param_robot.mI;

param_robot.link1_offset = [param_robot.l__s1 0 0];

param_robot.link1.R__joint = eye(3);
param_robot.link1.d__joint = [param_robot.l__1-param_robot.l__s1 0 0];
param_robot.link1.R__inertiaOrigin = eye(3);
param_robot.link1.d__inertiaOrigin = [0 0 0];
param_robot.link1.d__inertiaMatrixDiag = param_robot.inertia_matrix_s1;
param_robot.link1.m = param_robot.m1;

param_robot.link2_offset = [param_robot.l__s2 0 0];

param_robot.link2.R__joint = eye(3);
param_robot.link2.d__joint = [param_robot.l__2 - param_robot.l__s2 0 0];
param_robot.link2.R__inertiaOrigin = eye(3);
param_robot.link2.d__inertiaOrigin = [0 0 0];
param_robot.link2.d__inertiaMatrixDiag = param_robot.inertia_matrix_s2;
param_robot.link2.m = param_robot.m2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FR3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param_robot_fr3 = struct;
white = [1 1 1];
off_white = [0.901961 0.921569 0.929412];
black = [0.25 0.25 0.25];
green = [0 1 0];
light_blue = [0.039216 0.541176 0.780392];

fr3_base_opacity = 1;
rpy2rotm = @(rpy) eul2rotm(rpy, "XYZ");

%%%% Param Robot (FR3) %%%% (TODO)
%param_robot.l__I = 0.0583789; %m
%param_robot.mI = rho*249312.22389582483*1e-9; %kg
%param_robot.inertia_matrix_sI = rho*[2.60656e+08 4.65146e+08 4.47456e+08]*1e-15; %kgm^2
%param_robot.II = param_robot.inertia_matrix_sI(3); %kgm^2