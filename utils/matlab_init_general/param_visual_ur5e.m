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

cap_color = [141, 181, 213]/255;
rod_color = [185, 185, 185]/255;
con_color = [4, 4, 4]/255;
joi_color = [104, 104, 104]/255;

ur5e_base_opacity = 1;
rpy2rotm_xyz = @(rpy) eul2rotm(rpy, "XYZ"); % defined in matlab function

mesh_path = './stl_files/Meshes_ur5e/';

%% UR5e Robot
% src: https://github.com/mcboyd/ur5e_with_robotiq_hande/blob/ros1-melodic/urdf/ur5e_with_robotiq_hande.urdf
ur5e.robot = struct;

ur5e.robot.p_0 = [0; 0; 0]; % m
ur5e.robot.R_0 = eye(3); % m

ur5e.robot.R__world_basejoint = rpy2rotm_xyz([0 0 0]);
ur5e.robot.d__world_basejoint = [0; 0; 0];

% Settings of base body
ur5e.robot.R__base_link0 = rpy2rotm_xyz([0 0 0]);
ur5e.robot.d__base_link0 = [0; 0; 0];
R2_corr = Rx(-pi/2);

% Settings of link0 (Base) body
ur5e.robot.link0.rod.stl_path     = [mesh_path, 'ur5e-link0_rod.stl'];
ur5e.robot.link0.rod.color        = rod_color;
ur5e.robot.link0.con.stl_path     = [mesh_path, 'ur5e-link0_con.stl'];
ur5e.robot.link0.con.color        = con_color;
ur5e.robot.link0.opacity          = ur5e_base_opacity;

% Homogeneous Transformation from link0 to joint1: Rotation axis: z
ur5e.robot.R__link0_joint1 = rpy2rotm_xyz([0 0 0]);
ur5e.robot.d__link0_joint1 = [0; 0; 0.1625];

% Settings of link1 body
ur5e.robot.link1.joi.stl_path     = [mesh_path, 'ur5e-link1_joi.stl'];
ur5e.robot.link1.joi.color        = joi_color;
ur5e.robot.link1.con.stl_path     = [mesh_path, 'ur5e-link1_con.stl'];
ur5e.robot.link1.con.color        = con_color;
urte.robot.link1.cap.stl_path     = [mesh_path, 'ur5e-link1_cap.stl'];
ur5e.robot.link1.cap.color        = cap_color;
ur5e.robot.link1.opacity          = ur5e_base_opacity;

% Homogeneous Transformation from link1 to joint2: Rotation axis: y
ur5e.robot.R__link1_joint2 = rpy2rotm_xyz([0 pi/2 0]);
ur5e.robot.d__link1_joint2 = [0; 0.138; 0];

% Settings of link2 body
ur5e.robot.link2.rod.stl_path     = [mesh_path, 'ur5e-link2_rod.stl'];
ur5e.robot.link2.rod.color        = rod_color;
ur5e.robot.link2.joi1.stl_path     = [mesh_path, 'ur5e-link2_joi1.stl'];
ur5e.robot.link2.joi1.color        = joi_color;
ur5e.robot.link2.joi2.stl_path     = [mesh_path, 'ur5e-link2_joi2.stl'];
ur5e.robot.link2.joi2.color        = joi_color;
ur5e.robot.link2.con1.stl_path     = [mesh_path, 'ur5e-link2_con1.stl'];
ur5e.robot.link2.con1.color        = con_color;
ur5e.robot.link2.con2.stl_path     = [mesh_path, 'ur5e-link2_con2.stl'];
ur5e.robot.link2.con2.color        = con_color;
ur5e.robot.link3.con3.stl_path     = [mesh_path, 'ur5e-link2_con3.stl'];
ur5e.robot.link2.con3.color        = con_color;
ur5e.robot.link2.cap1.stl_path     = [mesh_path, 'ur5e-link2_cap1.stl'];
ur5e.robot.link2.cap1.color        = cap_color;
ur5e.robot.link2.cap2.stl_path     = [mesh_path, 'ur5e-link2_cap2.stl'];
ur5e.robot.link2.cap2.color        = cap_color;
ur5e.robot.link2.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from link2 to joint3: Rotation axis: y
ur5e.robot.R__link2_joint3 = rpy2rotm_xyz([0 0 0]);
ur5e.robot.d__link2_joint3 = [0; -0.131; 0.425];

% Settings of link3 body
ur5e.robot.link3.rod.stl_path     = [mesh_path, 'ur5e-link3_rod.stl'];
ur5e.robot.link3.rod.color        = rod_color;
ur5e.robot.link3.joi1.stl_path     = [mesh_path, 'ur5e-link3_joi1.stl'];
ur5e.robot.link3.joi1.color        = joi_color;
ur5e.robot.link3.joi2.stl_path     = [mesh_path, 'ur5e-link3_joi2.stl'];
ur5e.robot.link3.joi2.color        = joi_color;
ur5e.robot.link3.con1.stl_path     = [mesh_path, 'ur5e-link3_con1.stl'];
ur5e.robot.link3.con1.color        = con_color;
ur5e.robot.link3.con2.stl_path     = [mesh_path, 'ur5e-link3_con2.stl'];
ur5e.robot.link3.con2.color        = con_color;
ur5e.robot.link3.con3.stl_path     = [mesh_path, 'ur5e-link3_con3.stl'];
ur5e.robot.link3.con3.color        = con_color;
ur5e.robot.link3.cap.stl_path     = [mesh_path, 'ur5e-link3_cap.stl'];
ur5e.robot.link3.cap.color        = cap_color;
ur5e.robot.link3.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from link3 to joint4: Rotation axis: y
ur5e.robot.R__link3_joint4 = rpy2rotm_xyz([0, pi/2, 0]);
ur5e.robot.d__link3_joint4 = [0; 0; 0.3922];

% Settings of link4 body
ur5e.robot.link4.joi.stl_path     = [mesh_path, 'ur5e-link4_joi.stl'];
ur5e.robot.link4.joi.color        = joi_color;
ur5e.robot.link4.con.stl_path     = [mesh_path, 'ur5e-link4_con.stl'];
ur5e.robot.link4.con.color        = con_color;
ur5e.robot.link4.cap.stl_path     = [mesh_path, 'ur5e-link4_cap.stl'];
ur5e.robot.link4.cap.color        = cap_color;
ur5e.robot.link4.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from link4 to joint5: Rotation axis: z
ur5e.robot.R__link4_joint5 = rpy2rotm_xyz([0 0 0]);
ur5e.robot.d__link4_joint5 = [0.0; 0.127; 0];

% Settings of link5 body
ur5e.robot.link5.joi.stl_path     = [mesh_path, 'ur5e-link5_joi.stl'];
ur5e.robot.link5.joi.color        = joi_color;
ur5e.robot.link5.con.stl_path     = [mesh_path, 'ur5e-link5_con.stl'];
ur5e.robot.link5.con.color        = con_color;
ur5e.robot.link5.cap.stl_path     = [mesh_path, 'ur5e-link5_cap.stl'];
ur5e.robot.link5.cap.color        = cap_color;
ur5e.robot.link5.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from link5 to joint6: Rotation axis: y
ur5e.robot.R__link5_joint6 = rpy2rotm_xyz([0 0 0]);
ur5e.robot.d__link5_joint6 = [0.0; 0.0; 0.0997];

% Settings of link6 body
ur5e.robot.link6.rod.stl_path     = [mesh_path, 'ur5e-link6_rod.stl'];
ur5e.robot.link6.rod.color        = rod_color;
ur5e.robot.link6.opacity            = ur5e_base_opacity;

% Settings of TCP
ur5e.robot.R__hand_fixedTCPjoint = rpy2rotm_xyz([0 0 0]);
ur5e.robot.d__hand_fixedTCPjoint = [0; 0.0996; 0];


% MAXIMUM RATINGS
% https://www.universal-robots.com/media/1827367/05_2023_collective_data-sheet.pdf

% Gelenkpositionsgrenzen in Radiant
q_max = [2*pi; 2*pi; 2*pi; 2*pi; 2*pi; 2*pi];
q_min = -q_max;

% Maximale Gelenkgeschwindigkeit in Radiant pro Sekunde
q_dot_max = [pi; pi; pi; pi; pi; pi];
q_dot_min = -q_dot_max;

% Maximales Drehmoment in Newtonmeter
% https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques-cb3-and-e-series/
tau_max = [150; 150; 150; 28; 28; 28];
tau_min = -tau_max;