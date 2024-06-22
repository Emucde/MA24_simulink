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

cup_color = [141, 181, 213]/255;
rod_color = [185, 185, 185]/255;
con_color = [4, 4, 4]/255;
joint_color = [104, 104, 104]/255;

ur5e_base_opacity = 1;
rpy2rotm_xyz = @(rpy) eul2rotm(rpy, "XYZ"); % defined in matlab function

mesh_path = './stl_files/Meshes_FR3/visual/';

%% FR Robot
ur5e.robot = struct;

% Settings of link0 (Base) body
ur5e.robot.link0.rod.stl_path     = [mesh_path, 'ur5e-base1.stl'];
ur5e.robot.link0.rod.color        = rod_color;
ur5e.robot.link0.con.stl_path     = [mesh_path, 'ur5e-base2.stl'];
ur5e.robot.link0.con.color        = con_color;
ur5e.robot.link0.opacity          = ur5e_base_opacity;

% Homogeneous Transformation from link0 to joint1
ur5e.robot.R__link0_joint1 = rpy2rotm_xyz([0 0 0]);
ur5e.robot.d__link0_joint1 = [0; 0; 0.333];

% Joint 1 limits
ur5e.robot.joint1.lb = -2.3093;
ur5e.robot.joint1.ub = 2.3093;

% Settings of link1 body
ur5e.robot.link1.white.stl_path     = [mesh_path, 'link1_white.stl'];
ur5e.robot.link1.white.color        = white;
ur5e.robot.link1.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from link1 to joint2
ur5e.robot.R__link1_joint2 = rpy2rotm_xyz([-1.5707963267948966 0 0]);
ur5e.robot.d__link1_joint2 = [0; 0; 0];

% Joint 2 limits
ur5e.robot.joint2.lb = -1.5133;
ur5e.robot.joint2.ub = 1.5133;

% Settings of link2 body
ur5e.robot.link2.white.stl_path     = [mesh_path, 'link2_white.stl'];
ur5e.robot.link2.white.color        = white;
ur5e.robot.link2.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from link2 to joint3
ur5e.robot.R__link2_joint3 = rpy2rotm_xyz([1.5707963267948966 0 0]);
ur5e.robot.d__link2_joint3 = [0; -0.316; 0];

% Joint 3 limits
ur5e.robot.joint3.lb = -2.4937;
ur5e.robot.joint3.ub = 2.4937;

% Settings of link3 body
ur5e.robot.link3.white.stl_path     = [mesh_path, 'link3_white.stl'];
ur5e.robot.link3.white.color        = white;
ur5e.robot.link3.opacity            = ur5e_base_opacity;
ur5e.robot.link3.black.stl_path     = [mesh_path, 'link3_black.stl'];
ur5e.robot.link3.black.color        = black;
ur5e.robot.link3.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from link3 to joint4
ur5e.robot.R__link3_joint4 = rpy2rotm_xyz([1.5707963267948966 0 0]);
ur5e.robot.d__link3_joint4 = [0.0825; 0; 0];

% Joint 4 limits
ur5e.robot.joint4.lb = -2.7478;
ur5e.robot.joint4.ub = -0.4461;

% Settings of link4 body
ur5e.robot.link4.white.stl_path     = [mesh_path, 'link4_white.stl'];
ur5e.robot.link4.white.color        = white;
ur5e.robot.link4.black.stl_path     = [mesh_path, 'link4_black.stl'];
ur5e.robot.link4.black.color        = black;
ur5e.robot.link4.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from link4 to joint5
ur5e.robot.R__link4_joint5 = rpy2rotm_xyz([-1.5707963267948966 0 0]);
ur5e.robot.d__link4_joint5 = [-0.0825; 0.384; 0];

% Joint 5 limits
ur5e.robot.joint5.lb = -2.48;
ur5e.robot.joint5.ub = 2.48;

% Settings of link5 body
ur5e.robot.link5.white.stl_path     = [mesh_path, 'link5_white.stl'];
ur5e.robot.link5.white.color        = white;
ur5e.robot.link5.black.stl_path     = [mesh_path, 'link5_black.stl'];
ur5e.robot.link5.black.color        = black;
ur5e.robot.link5.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from link5 to joint6
ur5e.robot.R__link5_joint6 = rpy2rotm_xyz([1.5707963267948966 0 0]);
ur5e.robot.d__link5_joint6 = [0; 0; 0];

% Joint 6 limits
ur5e.robot.joint6.lb = 0.8521;
ur5e.robot.joint6.ub = 4.2094;

% Settings of link6 body
ur5e.robot.link6.black.stl_path     = [mesh_path, 'link6_black.stl'];
ur5e.robot.link6.black.color        = black;
ur5e.robot.link6.green.stl_path     = [mesh_path, 'link6_green.stl'];
ur5e.robot.link6.green.color        = green;
ur5e.robot.link6.light_blue.stl_path     = [mesh_path, 'link6_light_blue.stl'];
ur5e.robot.link6.light_blue.color        = light_blue;
ur5e.robot.link6.off_white.stl_path     = [mesh_path, 'link6_white.stl'];
ur5e.robot.link6.off_white.color        = off_white;
ur5e.robot.link6.white.stl_path     = [mesh_path, 'link6_white.stl'];
ur5e.robot.link6.white.color        = white;
ur5e.robot.link6.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from link6 to joint7
ur5e.robot.R__link6_joint7 = rpy2rotm_xyz([1.5707963267948966 0 0]);
ur5e.robot.d__link6_joint7 = [0.088; 0; 0];

% Joint 7 limits
ur5e.robot.joint7.lb = -2.6895;
ur5e.robot.joint7.ub = 2.6895;

% Settings of link7 body
ur5e.robot.link7.white.stl_path     = [mesh_path, 'link7_white.stl'];
ur5e.robot.link7.white.color        = white;
ur5e.robot.link7.black.stl_path     = [mesh_path, 'link7_black.stl'];
ur5e.robot.link7.black.color        = black;
ur5e.robot.link7.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from link7 to fixed joint8
ur5e.robot.R__link7_fixedjoint8 = rpy2rotm_xyz([0 0 0]);
ur5e.robot.d__link7_fixedjoint8 = [0; 0; 0.107];

% Homogeneous Transformation from link8 to fixed hand joint
ur5e.robot.R__link8_fixedhandjoint = rpy2rotm_xyz([0 0 -0.7853981633974483]);
ur5e.robot.d__link8_fixedhandjoint = [0; 0; 0];

% Settings of hand body
ur5e.robot.hand.white.stl_path     = [mesh_path, 'hand_white.stl'];
ur5e.robot.hand.white.color        = white;
ur5e.robot.hand.off_white.stl_path = [mesh_path, 'hand_off_white.stl'];
ur5e.robot.hand.off_white.color    = off_white;
ur5e.robot.hand.black.stl_path     = [mesh_path, 'hand_black.stl'];
ur5e.robot.hand.black.color        = black;
ur5e.robot.hand.opacity            = ur5e_base_opacity;

% Homogeneous Transformation from hand to fixed left finger joint
finger_openwidth = 0.01;
ur5e.robot.R__hand_fixedleftfingerjoint = rpy2rotm_xyz([0 0 0]);
ur5e.robot.d__hand_fixedleftfingerjoint = [0; finger_openwidth; 0.0584];

% Homogeneous Transformation from hand to fixed TCP joint
ur5e.robot.R__hand_fixedTCPjoint = rpy2rotm_xyz([0 0 0]);
ur5e.robot.d__hand_fixedTCPjoint = [0 0 0.1034];

% Homogeneous Transformation from hand to fixed right finger joint
ur5e.robot.R__hand_fixedrightfingerjoint = rpy2rotm_xyz([0 0 3.141592653589793]);
ur5e.robot.d__hand_fixedrightfingerjoint = [0; -finger_openwidth; 0.0584];

% Settings of left finger link
ur5e.robot.leftfinger.off_white.stl_path = [mesh_path, 'finger_0_off_white.stl'];
ur5e.robot.leftfinger.off_white.color    = off_white;
ur5e.robot.leftfinger.black.stl_path     = [mesh_path, 'finger_1_black.stl'];
ur5e.robot.leftfinger.black.color        = black;
ur5e.robot.leftfinger.opacity            = ur5e_base_opacity;

% Settings of right finger link
ur5e.robot.rightfinger = ur5e.robot.leftfinger;

% Limits of ur5e
% https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3

% Maximale und minimale Gelenkwinkel in Radiant
%q_max = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159];
%q_min = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159];

% Maximale Gelenkgeschwindigkeit in Radiant pro Sekunde
%q_dot_max = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26];
%q_dot_min = -q_dot_max;

% Maximale Gelenkbeschleunigung in Radiant pro Sekunde Quadrat
q_ddot_max = [10; 10; 10; 10; 10; 10; 10];
q_ddot_min = -q_ddot_max;

% Maximale Gelenkruck in Radiant pro Sekunde Kubik
q_dddot_max = [5000; 5000; 5000; 5000; 5000; 5000; 5000];
q_dddot_min = -q_dddot_max;

% Maximales Drehmoment in Newtonmeter
tau_max = [87; 87; 87; 87; 12; 12; 12];
tau_min = -tau_max;

% Maximale Drehmomentänderung in Newtonmeter pro Sekunde
tau_dot_max = [1000; 1000; 1000; 1000; 1000; 1000; 1000];
tau_dot_min = -tau_dot_max;

% Die exakten Werte für die Gelenkgeschwindigkeit könnte man über diese Formeln berechnen
% Allerdings habe ich keine Quelle für diese Formeln gefunden.
qi_dot_max_fun = @(q, i, a1, a2, a3) min(q_dot_max(i), max(0, a1 + sqrt(max(0, a2 * (a3 - q(i))))));
qi_dot_min_fun = @(q, i, a1, a2, a3) max(q_dot_min(i), min(0, a1 - sqrt(max(0, a2 * (a3 - q(i))))));

q_dot_max_fun = @(q) [qi_dot_max_fun(q, 1, -0.30, 12.0, 2.75010); ...
                      qi_dot_max_fun(q, 2, -0.20, 5.17, 1.79180); ...
                      qi_dot_max_fun(q, 3, -0.20, 7.00, 2.90650); ...
                      qi_dot_max_fun(q, 4, -0.30, 8.00, -0.1458); ...
                      qi_dot_max_fun(q, 5, -0.35, 34.0, 2.81010); ...
                      qi_dot_max_fun(q, 6, -0.35, 11.0, 4.52050); ...
                      qi_dot_max_fun(q, 7, -0.35, 34.0, 3.01960)];

q_dot_min_fun = @(q) [qi_dot_max_fun(q, 1, 0.30, 12.0, 2.750100); ...
                      qi_dot_max_fun(q, 2, 0.20, 5.17, 1.791800); ...
                      qi_dot_max_fun(q, 3, 0.20, 7.00, 2.906500); ...
                      qi_dot_max_fun(q, 4, 0.30, 8.00, 2.810100); ...
                      qi_dot_max_fun(q, 5, 0.35, 34.0, 2.810100); ...
                      qi_dot_max_fun(q, 6, 0.35, 11.0, -0.54092); ...
                      qi_dot_max_fun(q, 7, 0.35, 34.0, 3.01960)];

% Allerdings liefern die folgenden Formeln eine gute Näherung, die von Franka GMBH empfohlen werden:
% D. h. man plottet die obigen Funktionen q_p_i(q_i) und zeichnet ein Rechteck ein
% Damit verkleinert man zwar den erreichbaren Arbeitsraum, überschreitet aber sicher keine konfigurationsabhängigen
% Geschwindigkeitslimits. Mehr dazu in der Dokumentation von Franka GMBH im obigen Link.

% Gelenkpositionsgrenzen in Radiant
q_max = [2.3093; 1.5133; 2.4937; -0.4461; 2.4800; 4.2094; 2.6895];
q_min = [-2.3093; -1.5133; -2.4937; -2.7478; -2.4800; 0.8521; -2.6895];

% Maximale Gelenkgeschwindigkeit in Radiant pro Sekunde
q_dot_max = [2; 1; 1.5; 1.25; 3; 1.5; 3];
q_dot_min = -q_dot_max;