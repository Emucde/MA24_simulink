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

%% FR Robot
fr3.robot = struct;

% Settings of link0 (Base) body
fr3.robot.link0.white.stl_path     = './stl_files/Meshes_FR3/link0_white.stl';
fr3.robot.link0.white.color        = white;
fr3.robot.link0.off_white.stl_path = './stl_files/Meshes_FR3/link0_off_white.stl';
fr3.robot.link0.off_white.color    = off_white;
fr3.robot.link0.black.stl_path     = './stl_files/Meshes_FR3/link0_black.stl';
fr3.robot.link0.black.color        = black;
fr3.robot.link0.opacity            = fr3_base_opacity;
fr3.robot.link0.R__inertiaOrigin = rpy2rotm([0 0 0]);
fr3.robot.link0.d__inertiaOrigin = [-0.041018; -0.00014; 0.049974];
fr3.robot.link0.d__inertiaMatrix = [0.00315, 8.2904e-07, 0.00015;
                                   8.2904e-07, 0.00388, 8.2299e-06;
                                   0.00015, 8.2299e-06, 0.004285];
fr3.robot.link0.m = 0.629769;

% Homogeneous Transformation from link0 to joint1
fr3.robot.R__link0_joint1 = rpy2rotm([0 0 0]);
fr3.robot.d__link0_joint1 = [0; 0; 0.333];

% Joint 1 limits
fr3.robot.joint1.lb = -2.3093;
fr3.robot.joint1.ub = 2.3093;

% Settings of link1 body
fr3.robot.link1.white.stl_path     = './stl_files/Meshes_FR3/link1_white.stl';
fr3.robot.link1.white.color        = white;
fr3.robot.link1.opacity            = fr3_base_opacity;
fr3.robot.link1.R__inertiaOrigin = rpy2rotm([0 0 0]);
fr3.robot.link1.d__inertiaOrigin = [0.003875; 0.002081; -0.04762];
fr3.robot.link1.d__inertiaMatrix = [0.70337, -0.000139, 0.006772;
                                   -0.000139, 0.70661, 0.019169;
                                   0.006772, 0.019169, 0.009117];
fr3.robot.link1.m = 4.970684;

% Homogeneous Transformation from link1 to joint2
fr3.robot.R__link1_joint2 = rpy2rotm([-1.5707963267948966 0 0]);
fr3.robot.d__link1_joint2 = [0; 0; 0];

% Joint 2 limits
fr3.robot.joint2.lb = -1.5133;
fr3.robot.joint2.ub = 1.5133;

% Settings of link2 body
fr3.robot.link2.white.stl_path     = './stl_files/Meshes_FR3/link2_white.stl';
fr3.robot.link2.white.color        = white;
fr3.robot.link2.opacity            = fr3_base_opacity;
fr3.robot.link2.R__inertiaOrigin = rpy2rotm([0 0 0]);
fr3.robot.link2.d__inertiaOrigin = [-0.003141; -0.02872; 0.003495];
fr3.robot.link2.d__inertiaMatrix = [0.007962, -0.003925, 0.010254;
                                   -0.003925, 0.02811, 0.000704;
                                   0.010254, 0.000704, 0.025995];
fr3.robot.link2.m = 0.646926;

% Homogeneous Transformation from link2 to joint3
fr3.robot.R__link2_joint3 = rpy2rotm([1.5707963267948966 0 0]);
fr3.robot.d__link2_joint3 = [0; -0.316; 0];

% Joint 3 limits
fr3.robot.joint3.lb = -2.4937;
fr3.robot.joint3.ub = 2.4937;

% Settings of link3 body
fr3.robot.link3.white.stl_path     = './stl_files/Meshes_FR3/link3_white.stl';
fr3.robot.link3.white.color        = white;
fr3.robot.link3.opacity            = fr3_base_opacity;
fr3.robot.link3.black.stl_path     = './stl_files/Meshes_FR3/link3_black.stl';
fr3.robot.link3.black.color        = black;
fr3.robot.link3.opacity            = fr3_base_opacity;
fr3.robot.link3.R__inertiaOrigin = rpy2rotm([0 0 0]);
fr3.robot.link3.d__inertiaOrigin = [2.7518e-02; 3.9252e-02; -6.6502e-02];
fr3.robot.link3.d__inertiaMatrix = [0.037242, -0.004761, -0.011396;
                                   -0.004761, 0.036155, -0.012805;
                                   -0.011396, -0.012805, 0.01083];
fr3.robot.link3.m = 3.228604;

% Homogeneous Transformation from link3 to joint4
fr3.robot.R__link3_joint4 = rpy2rotm([1.5707963267948966 0 0]);
fr3.robot.d__link3_joint4 = [0.0825; 0; 0];

% Joint 4 limits
fr3.robot.joint4.lb = -2.7478;
fr3.robot.joint4.ub = -0.4461;

% Settings of link4 body
fr3.robot.link4.white.stl_path     = './stl_files/Meshes_FR3/link4_white.stl';
fr3.robot.link4.white.color        = white;
fr3.robot.link4.black.stl_path     = './stl_files/Meshes_FR3/link4_black.stl';
fr3.robot.link4.black.color        = black;
fr3.robot.link4.opacity            = fr3_base_opacity;
fr3.robot.link4.R__inertiaOrigin = rpy2rotm([0 0 0]);
fr3.robot.link4.d__inertiaOrigin = [-5.317e-02; 1.04419e-01; 2.7454e-02];
fr3.robot.link4.d__inertiaMatrix = [0.025853, 0.007796, -0.001332;
                                   0.007796, 0.019552, 0.008641;
                                   -0.001332, 0.008641, 0.028323];
fr3.robot.link4.m = 3.587895;

% Homogeneous Transformation from link4 to joint5
fr3.robot.R__link4_joint5 = rpy2rotm([-1.5707963267948966 0 0]);
fr3.robot.d__link4_joint5 = [-0.0825; 0.384; 0];

% Joint 5 limits
fr3.robot.joint5.lb = -2.48;
fr3.robot.joint5.ub = 2.48;

% Settings of link5 body
fr3.robot.link5.white.stl_path     = './stl_files/Meshes_FR3/link5_white.stl';
fr3.robot.link5.white.color        = white;
fr3.robot.link5.black.stl_path     = './stl_files/Meshes_FR3/link5_black.stl';
fr3.robot.link5.black.color        = black;
fr3.robot.link5.opacity            = fr3_base_opacity;
fr3.robot.link5.R__inertiaOrigin = rpy2rotm([0 0 0]);
fr3.robot.link5.d__inertiaOrigin = [-1.1953e-02; 4.1065e-02; -3.8437e-02];
fr3.robot.link5.d__inertiaMatrix = [0.035549, -0.002117, -0.004037;
                                   -0.002117, 0.029474, 0.000229;
                                   -0.004037, 0.000229, 0.008627];
fr3.robot.link5.m = 1.225946;

% Homogeneous Transformation from link5 to joint6
fr3.robot.R__link5_joint6 = rpy2rotm([1.5707963267948966 0 0]);
fr3.robot.d__link5_joint6 = [0; 0; 0];

% Joint 6 limits
fr3.robot.joint6.lb = 0.8521;
fr3.robot.joint6.ub = 4.2094;

% Settings of link6 body
fr3.robot.link6.black.stl_path     = './stl_files/Meshes_FR3/link6_black.stl';
fr3.robot.link6.black.color        = black;
fr3.robot.link6.green.stl_path     = './stl_files/Meshes_FR3/link6_green.stl';
fr3.robot.link6.green.color        = green;
fr3.robot.link6.light_blue.stl_path     = './stl_files/Meshes_FR3/link6_light_blue.stl';
fr3.robot.link6.light_blue.color        = light_blue;
fr3.robot.link6.off_white.stl_path     = './stl_files/Meshes_FR3/link6_white.stl';
fr3.robot.link6.off_white.color        = off_white;
fr3.robot.link6.white.stl_path     = './stl_files/Meshes_FR3/link6_white.stl';
fr3.robot.link6.white.color        = white;
fr3.robot.link6.opacity            = fr3_base_opacity;
fr3.robot.link6.R__inertiaOrigin = rpy2rotm([0 0 0]);
fr3.robot.link6.d__inertiaOrigin = [6.0149e-02; -1.4117e-02; -1.0517e-02];
fr3.robot.link6.d__inertiaMatrix = [0.001964, 0.000109, -0.001158;
                                   0.000109, 0.004354, 0.000341;
                                   -0.001158, 0.000341, 0.005433];
fr3.robot.link6.m = 1.666555;

% Homogeneous Transformation from link6 to joint7
fr3.robot.R__link6_joint7 = rpy2rotm([1.5707963267948966 0 0]);
fr3.robot.d__link6_joint7 = [0.088; 0; 0];

% Joint 7 limits
fr3.robot.joint7.lb = -2.6895;
fr3.robot.joint7.ub = 2.6895;

% Settings of link7 body
fr3.robot.link7.white.stl_path     = './stl_files/Meshes_FR3/link7_white.stl';
fr3.robot.link7.white.color        = white;
fr3.robot.link7.black.stl_path     = './stl_files/Meshes_FR3/link7_black.stl';
fr3.robot.link7.black.color        = black;
fr3.robot.link7.opacity            = fr3_base_opacity;

fr3.robot.link7.R__inertiaOrigin = rpy2rotm([0 0 0]);
fr3.robot.link7.d__inertiaOrigin = [1.0517e-02; -4.252e-03; 6.1597e-02];
fr3.robot.link7.d__inertiaMatrix = [0.012516, -0.000428, -0.001196;
                                   -0.000428, 0.010027, -0.000741;
                                   -0.001196, -0.000741, 0.004815];
fr3.robot.link7.m = 0.735522;

% Homogeneous Transformation from link7 to fixed joint8
fr3.robot.R__link7_fixedjoint8 = rpy2rotm([0 0 0]);
fr3.robot.d__link7_fixedjoint8 = [0; 0; 0.107];

% Settings of link8 body
fr3.robot.link8.R__inertiaOrigin = rpy2rotm([0 0 0]);
fr3.robot.link8.d__inertiaOrigin = [0; 0; 0];
fr3.robot.link8.d__inertiaMatrix = [1e-6, 0, 0;
                                   0, 1e-6, 0;
                                   0, 0, 1e-6];
fr3.robot.link8.m = 1e-5;

% Homogeneous Transformation from link8 to fixed hand joint
fr3.robot.R__link8_fixedhandjoint = rpy2rotm([0 0 -0.7853981633974483]);
fr3.robot.d__link8_fixedhandjoint = [0; 0; 0];

% Settings of hand body
fr3.robot.hand.white.stl_path     = './stl_files/Meshes_FR3/hand_white.stl';
fr3.robot.hand.white.color        = white;
fr3.robot.hand.off_white.stl_path = './stl_files/Meshes_FR3/hand_off_white.stl';
fr3.robot.hand.off_white.color    = off_white;
fr3.robot.hand.black.stl_path     = './stl_files/Meshes_FR3/hand_black.stl';
fr3.robot.hand.black.color        = black;
fr3.robot.hand.opacity            = fr3_base_opacity;

fr3.robot.hand.R__inertiaOrigin = rpy2rotm([0 0 0]);
fr3.robot.hand.d__inertiaOrigin = [-0.01; 0; 0.03];
fr3.robot.hand.d__inertiaMatrix = [0.001, 0, 0;
                                  0, 0.0025, 0;
                                  0, 0, 0.0017];
fr3.robot.hand.m = 0.73;

% Homogeneous Transformation from hand to fixed left finger joint
finger_openwidth = 0.01;
fr3.robot.R__hand_fixedleftfingerjoint = rpy2rotm([0 0 0]);
fr3.robot.d__hand_fixedleftfingerjoint = [0; finger_openwidth; 0.0584];

% Homogeneous Transformation from hand to fixed TCP joint
fr3.robot.R__hand_fixedTCPjoint = rpy2rotm([0 0 0]);
fr3.robot.d__hand_fixedTCPjoint = [0 0 0.1034];

% Homogeneous Transformation from hand to fixed right finger joint
fr3.robot.R__hand_fixedrightfingerjoint = rpy2rotm([0 0 3.141592653589793]);
fr3.robot.d__hand_fixedrightfingerjoint = [0; -finger_openwidth; 0.0584];

% Settings of left finger link
fr3.robot.leftfinger.off_white.stl_path = './stl_files/Meshes_FR3/finger_0_off_white.stl';
fr3.robot.leftfinger.off_white.color    = off_white;
fr3.robot.leftfinger.black.stl_path     = './stl_files/Meshes_FR3/finger_1_black.stl';
fr3.robot.leftfinger.black.color        = black;
fr3.robot.leftfinger.opacity            = fr3_base_opacity;
%fr3.robot.R__hand_fixedrightfingerjoint zeigt bereits in Schwerpunkt
fr3.robot.leftfinger.R__inertiaOrigin = rpy2rotm([0 0 0]);
fr3.robot.leftfinger.d__inertiaOrigin = [0; 0; 0];
fr3.robot.leftfinger.d__inertiaMatrix = [2.3749999999999997e-06, 0, 0;
                                        0, 2.3749999999999997e-06, 0;
                                        0, 0, 7.5e-07];
fr3.robot.leftfinger.m = 0.015;

% Settings of right finger link
fr3.robot.rightfinger = fr3.robot.leftfinger;

% Limits of FR3
% https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3

% Maximale und minimale Gelenkwinkel in Radiant
%q_max = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159];
%q_min = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159];

% Maximale Gelenkgeschwindigkeit in Radiant pro Sekunde
%q_dot_max = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26];
%q_dot_min = -q_dot_max;

% Maximale Gelenkbeschleunigung in Radiant pro Sekunde Quadrat
q_ddot_max = [10, 10, 10, 10, 10, 10, 10];
q_ddot_min = -q_ddot_max;

% Maximale Gelenkruck in Radiant pro Sekunde Kubik
q_dddot_max = [5000, 5000, 5000, 5000, 5000, 5000, 5000];
q_dddot_min = -q_dddot_max;

% Maximales Drehmoment in Newtonmeter
tau_max = [87, 87, 87, 87, 12, 12, 12];
tau_min = -tau_max;

% Maximale Drehmomentänderung in Newtonmeter pro Sekunde
tau_dot_max = [1000, 1000, 1000, 1000, 1000, 1000, 1000];
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
q_min = [-2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895];
q_max = [2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895];

% Maximale Gelenkgeschwindigkeit in Radiant pro Sekunde
q_dot_max = [2, 1, 1.5, 1.25, 3, 1.5, 3];
q_dot_min = -q_dot_max;