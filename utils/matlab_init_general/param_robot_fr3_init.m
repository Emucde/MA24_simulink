%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% FR3 Parameter for Simulink %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

white = [1 1 1];
off_white = [0.901961 0.921569 0.929412];
black = [0.25 0.25 0.25];
green = [0 1 0];
light_blue = [0.039216 0.541176 0.780392];

fr3_base_opacity = 1;
rpy2rotm_xyz = @(rpy) eul2rotm(rpy, "XYZ"); % defined in matlab function

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
fr3.robot.link0.R__inertiaOrigin = rpy2rotm_xyz([0 0 0]);
fr3.robot.link0.d__inertiaOrigin = [-0.041018; -0.00014; 0.049974];
fr3.robot.link0.d__inertiaMatrix = [0.00315, 8.2904e-07, 0.00015;
                                   8.2904e-07, 0.00388, 8.2299e-06;
                                   0.00015, 8.2299e-06, 0.004285];
fr3.robot.link0.m = 0.629769;

% Homogeneous Transformation from link0 to joint1
fr3.robot.R__link0_joint1 = rpy2rotm_xyz([0 0 0]);
fr3.robot.d__link0_joint1 = [0; 0; 0.333];

% Joint 1 limits
fr3.robot.joint1.lb = -2.3093;
fr3.robot.joint1.ub = 2.3093;

% Settings of link1 body
fr3.robot.link1.white.stl_path     = './stl_files/Meshes_FR3/link1_white.stl';
fr3.robot.link1.white.color        = white;
fr3.robot.link1.opacity            = fr3_base_opacity;
fr3.robot.link1.R__inertiaOrigin = rpy2rotm_xyz([0 0 0]);
fr3.robot.link1.d__inertiaOrigin = [0.003875; 0.002081; -0.04762];
fr3.robot.link1.d__inertiaMatrix = [0.70337, -0.000139, 0.006772;
                                   -0.000139, 0.70661, 0.019169;
                                   0.006772, 0.019169, 0.009117];
fr3.robot.link1.m = 4.970684;

% Homogeneous Transformation from link1 to joint2
fr3.robot.R__link1_joint2 = rpy2rotm_xyz([-1.5707963267948966 0 0]);
fr3.robot.d__link1_joint2 = [0; 0; 0];

% Joint 2 limits
fr3.robot.joint2.lb = -1.5133;
fr3.robot.joint2.ub = 1.5133;

% Settings of link2 body
fr3.robot.link2.white.stl_path     = './stl_files/Meshes_FR3/link2_white.stl';
fr3.robot.link2.white.color        = white;
fr3.robot.link2.opacity            = fr3_base_opacity;
fr3.robot.link2.R__inertiaOrigin = rpy2rotm_xyz([0 0 0]);
fr3.robot.link2.d__inertiaOrigin = [-0.003141; -0.02872; 0.003495];
fr3.robot.link2.d__inertiaMatrix = [0.007962, -0.003925, 0.010254;
                                   -0.003925, 0.02811, 0.000704;
                                   0.010254, 0.000704, 0.025995];
fr3.robot.link2.m = 0.646926;

% Homogeneous Transformation from link2 to joint3
fr3.robot.R__link2_joint3 = rpy2rotm_xyz([1.5707963267948966 0 0]);
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
fr3.robot.link3.R__inertiaOrigin = rpy2rotm_xyz([0 0 0]);
fr3.robot.link3.d__inertiaOrigin = [2.7518e-02; 3.9252e-02; -6.6502e-02];
fr3.robot.link3.d__inertiaMatrix = [0.037242, -0.004761, -0.011396;
                                   -0.004761, 0.036155, -0.012805;
                                   -0.011396, -0.012805, 0.01083];
fr3.robot.link3.m = 3.228604;

% Homogeneous Transformation from link3 to joint4
fr3.robot.R__link3_joint4 = rpy2rotm_xyz([1.5707963267948966 0 0]);
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
fr3.robot.link4.R__inertiaOrigin = rpy2rotm_xyz([0 0 0]);
fr3.robot.link4.d__inertiaOrigin = [-5.317e-02; 1.04419e-01; 2.7454e-02];
fr3.robot.link4.d__inertiaMatrix = [0.025853, 0.007796, -0.001332;
                                   0.007796, 0.019552, 0.008641;
                                   -0.001332, 0.008641, 0.028323];
fr3.robot.link4.m = 3.587895;

% Homogeneous Transformation from link4 to joint5
fr3.robot.R__link4_joint5 = rpy2rotm_xyz([-1.5707963267948966 0 0]);
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
fr3.robot.link5.R__inertiaOrigin = rpy2rotm_xyz([0 0 0]);
fr3.robot.link5.d__inertiaOrigin = [-1.1953e-02; 4.1065e-02; -3.8437e-02];
fr3.robot.link5.d__inertiaMatrix = [0.035549, -0.002117, -0.004037;
                                   -0.002117, 0.029474, 0.000229;
                                   -0.004037, 0.000229, 0.008627];
fr3.robot.link5.m = 1.225946;

% Homogeneous Transformation from link5 to joint6
fr3.robot.R__link5_joint6 = rpy2rotm_xyz([1.5707963267948966 0 0]);
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
fr3.robot.link6.R__inertiaOrigin = rpy2rotm_xyz([0 0 0]);
fr3.robot.link6.d__inertiaOrigin = [6.0149e-02; -1.4117e-02; -1.0517e-02];
fr3.robot.link6.d__inertiaMatrix = [0.001964, 0.000109, -0.001158;
                                   0.000109, 0.004354, 0.000341;
                                   -0.001158, 0.000341, 0.005433];
fr3.robot.link6.m = 1.666555;

% Homogeneous Transformation from link6 to joint7
fr3.robot.R__link6_joint7 = rpy2rotm_xyz([1.5707963267948966 0 0]);
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

fr3.robot.link7.R__inertiaOrigin = rpy2rotm_xyz([0 0 0]);
fr3.robot.link7.d__inertiaOrigin = [1.0517e-02; -4.252e-03; 6.1597e-02];
fr3.robot.link7.d__inertiaMatrix = [0.012516, -0.000428, -0.001196;
                                   -0.000428, 0.010027, -0.000741;
                                   -0.001196, -0.000741, 0.004815];
fr3.robot.link7.m = 0.735522;

% Homogeneous Transformation from link7 to fixed joint8
fr3.robot.R__link7_fixedjoint8 = rpy2rotm_xyz([0 0 0]);
fr3.robot.d__link7_fixedjoint8 = [0; 0; 0.107];

% Settings of link8 body
fr3.robot.link8.R__inertiaOrigin = rpy2rotm_xyz([0 0 0]);
fr3.robot.link8.d__inertiaOrigin = [0; 0; 0];
fr3.robot.link8.d__inertiaMatrix = [1e-6, 0, 0;
                                   0, 1e-6, 0;
                                   0, 0, 1e-6];
fr3.robot.link8.m = 1e-5;

% Homogeneous Transformation from link8 to fixed hand joint
fr3.robot.R__link8_fixedhandjoint = rpy2rotm_xyz([0 0 -0.7853981633974483]);
fr3.robot.d__link8_fixedhandjoint = [0; 0; 0];

% Settings of hand body
fr3.robot.hand.white.stl_path     = './stl_files/Meshes_FR3/hand_white.stl';
fr3.robot.hand.white.color        = white;
fr3.robot.hand.off_white.stl_path = './stl_files/Meshes_FR3/hand_off_white.stl';
fr3.robot.hand.off_white.color    = off_white;
fr3.robot.hand.black.stl_path     = './stl_files/Meshes_FR3/hand_black.stl';
fr3.robot.hand.black.color        = black;
fr3.robot.hand.opacity            = fr3_base_opacity;

fr3.robot.hand.R__inertiaOrigin = rpy2rotm_xyz([0 0 0]);
fr3.robot.hand.d__inertiaOrigin = [-0.01; 0; 0.03];
fr3.robot.hand.d__inertiaMatrix = [0.001, 0, 0;
                                  0, 0.0025, 0;
                                  0, 0, 0.0017];
fr3.robot.hand.m = 0.73;

% Homogeneous Transformation from hand to fixed left finger joint
finger_openwidth = 0.01;
fr3.robot.R__hand_fixedleftfingerjoint = rpy2rotm_xyz([0 0 0]);
fr3.robot.d__hand_fixedleftfingerjoint = [0; finger_openwidth; 0.0584];

% Homogeneous Transformation from hand to fixed TCP joint
fr3.robot.R__hand_fixedTCPjoint = rpy2rotm_xyz([0 0 0]);
fr3.robot.d__hand_fixedTCPjoint = [0 0 0.1034];

% Homogeneous Transformation from hand to fixed right finger joint
fr3.robot.R__hand_fixedrightfingerjoint = rpy2rotm_xyz([0 0 3.141592653589793]);
fr3.robot.d__hand_fixedrightfingerjoint = [0; -finger_openwidth; 0.0584];

% Settings of left finger link
fr3.robot.leftfinger.off_white.stl_path = './stl_files/Meshes_FR3/finger_0_off_white.stl';
fr3.robot.leftfinger.off_white.color    = off_white;
fr3.robot.leftfinger.black.stl_path     = './stl_files/Meshes_FR3/finger_1_black.stl';
fr3.robot.leftfinger.black.color        = black;
fr3.robot.leftfinger.opacity            = fr3_base_opacity;
%fr3.robot.R__hand_fixedrightfingerjoint zeigt bereits in Schwerpunkt
fr3.robot.leftfinger.R__inertiaOrigin = rpy2rotm_xyz([0 0 0]);
fr3.robot.leftfinger.d__inertiaOrigin = [0; 0; 0];
fr3.robot.leftfinger.d__inertiaMatrix = [2.3749999999999997e-06, 0, 0;
                                        0, 2.3749999999999997e-06, 0;
                                        0, 0, 7.5e-07];
fr3.robot.leftfinger.m = 0.015;

% Settings of right finger link
fr3.robot.rightfinger = fr3.robot.leftfinger;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Limits of FR3 robot %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% FR3 Parameter for Casadi and Maple Files %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fr3.param = struct;

fr3.param.n_DOF = n; % DOF of the robot

% fr3.param.q_0_ref = [0; 0; pi/4; -pi/2; 0; pi/2; 0]; % only q3=pi/4 is fixed
fr3.param.q_0_ref = [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]; % only q3=pi/4 is fixed
fr3.param.q_0_p_ref = zeros(7, 1);
fr3.param.q_0_pp_ref = zeros(7, 1);

fr3.param.m_t   = 3; % Translational task space
fr3.param.m_r   = 3; % Rotational task space
fr3.param.m     = 6;

fr3.param.g = [0;0;9.81]; %m/s
% Update: Vermutlich wegen der Kräpfte Konvention in Maple darf
% scheinbar nicht g = [0,0,-9.81] gewählt werden (warum auch immer). Es lässt sich schnell zeigen, da der Roboter bei
% tau=0 sonst nach oben gezogen wird, als würde er nach unten hängen.
fr3.param.g_x = fr3.param.g(1);
fr3.param.g_y = fr3.param.g(2);
fr3.param.g_z = fr3.param.g(3);
%% Robot gravity
fr3.param.g_vis=[0;0;-9.81]; %m/s^2

fr3.param.q_limit_upper = q_max;
fr3.param.q_limit_lower = q_min;

fr3.param.q_p_limit_upper = q_dot_max;
fr3.param.q_p_limit_lower = q_dot_min;

fr3.param.q_pp_limit_upper = q_ddot_max;
fr3.param.q_pp_limit_lower = q_ddot_min;

fr3.param.q_n = (q_max + q_min) / 2; % not the best reference pose for fr3

fr3.param.torque_limit_upper = tau_max;
fr3.param.torque_limit_lower = tau_min;

% Inertial System
fr3.param.p_0 = [0; 0; 0]; % m
fr3.param.R_0 = eye(3); % m

% LINK 0 (BASE)
fr3.param.sp0_x = -0.041018; % m
fr3.param.sp0_y = -0.00014; % m
fr3.param.sp0_z = 0.049974; % m
fr3.param.m0 = 0.629769; % kg
fr3.param.I0_xx = 0.00315; % kgm^2
fr3.param.I0_xy = 8.2904e-07; % kgm^2
fr3.param.I0_xz = 0.00015; % kgm^2
fr3.param.I0_yy = 0.00388; % kgm^2
fr3.param.I0_yz = 8.2299e-06; % kgm^2
fr3.param.I0_zz = 0.004285; % kgm^2

% LINK 1
fr3.param.l1 = 0.333; % m joint1: link0 to link1
fr3.param.sp1_x = 0.003875; % m
fr3.param.sp1_y = 0.002081; % m
fr3.param.sp1_z = -0.04762; % m
fr3.param.m1 = 4.970684; % kg
fr3.param.I1_xx = 0.70337; % kgm^2
fr3.param.I1_xy = -0.000139; % kgm^2
fr3.param.I1_xz = 0.006772; % kgm^2
fr3.param.I1_yy = 0.70661; % kgm^2
fr3.param.I1_yz = 0.019169; % kgm^2
fr3.param.I1_zz = 0.009117; % kgm^2

% LINK 2
fr3.param.l2 = 0.316; % m, joint 3: link2 to link3
fr3.param.sp2_x = -0.003141; % m
fr3.param.sp2_y = -0.02872; % m
fr3.param.sp2_z = 0.003495; % m
fr3.param.m2 = 0.646926; % kg
fr3.param.I2_xx = 0.007962; % kgm^2
fr3.param.I2_xy = -0.003925; % kgm^2
fr3.param.I2_xz = 0.010254; % kgm^2
fr3.param.I2_yy = 0.02811; % kgm^2
fr3.param.I2_yz = 0.000704; % kgm^2
fr3.param.I2_zz = 0.025995; % kgm^2

% LINK 3
fr3.param.l3 = 0.0825; % m, joint 4, link3 to link4
fr3.param.sp3_x = 0.027518; % m
fr3.param.sp3_y = 0.039252; % m
fr3.param.sp3_z = -0.066502; % m
fr3.param.m3 = 3.228604; % kg
fr3.param.I3_xx = 0.037242; % kgm^2
fr3.param.I3_xy = -0.004761; % kgm^2
fr3.param.I3_xz = -0.011396; % kgm^2
fr3.param.I3_yy = 0.036155; % kgm^2
fr3.param.I3_yz = -0.012805; % kgm^2
fr3.param.I3_zz = 0.01083; % kgm^2

% LINK 4
fr3.param.l4 = 0.384; % m, joint 5, link4 to link5
fr3.param.sp4_x = -0.05317; % m
fr3.param.sp4_y = 0.104419; % m
fr3.param.sp4_z = 0.027454; % m
fr3.param.m4 = 3.587895; % kg
fr3.param.I4_xx = 0.025853; % kgm^2
fr3.param.I4_xy = 0.007796; % kgm^2
fr3.param.I4_xz = -0.001332; % kgm^2
fr3.param.I4_yy = 0.019552; % kgm^2
fr3.param.I4_yz = 0.008641; % kgm^2
fr3.param.I4_zz = 0.028323; % kgm^2

% LINK 5
fr3.param.l5 = 0.088; % m, joint7, link6 to link7
fr3.param.sp5_x = -0.011953; % m
fr3.param.sp5_y = 0.041065; % m
fr3.param.sp5_z = -0.038437; % m
fr3.param.m5 = 1.225946; % kg
fr3.param.I5_xx = 0.035549; % kgm^2
fr3.param.I5_xy = -0.002117; % kgm^2
fr3.param.I5_xz = -0.004037; % kgm^2
fr3.param.I5_yy = 0.029474; % kgm^2
fr3.param.I5_yz = 0.000229; % kgm^2
fr3.param.I5_zz = 0.008627; % kgm^2

% LINK 6
fr3.param.l6 = 0.107; % m, fixed joint 8, link7 to link8
fr3.param.sp6_x = 0.060149; % m
fr3.param.sp6_y = -0.014117; % m
fr3.param.sp6_z = -0.010517; % m
fr3.param.m6 = 1.666555; % kg
fr3.param.I6_xx = 0.001964; % kgm^2
fr3.param.I6_xy = 0.000109; % kgm^2
fr3.param.I6_xz = -0.001158; % kgm^2
fr3.param.I6_yy = 0.004354; % kgm^2
fr3.param.I6_yz = 0.000341; % kgm^2
fr3.param.I6_zz = 0.005433; % kgm^2

% LINK 7
fr3.param.l7 = 0.1034; % m, fr3_hand_tcp_joint, fr3_hand to fr3_hand_tcp
fr3.param.sp7_x = 0.010517; % m
fr3.param.sp7_y = -0.004252; % m
fr3.param.sp7_z = 0.061597; % m
fr3.param.m7 = 0.735522; % kg
fr3.param.I7_xx = 0.012516; % kgm^2
fr3.param.I7_xy = -0.000428; % kgm^2
fr3.param.I7_xz = -0.001196; % kgm^2
fr3.param.I7_yy = 0.010027; % kgm^2
fr3.param.I7_yz = -0.000741; % kgm^2
fr3.param.I7_zz = 0.004815; % kgm^2

% LINK 8
fr3.param.sp8_x = 0; % m
fr3.param.sp8_y = 0; % m
fr3.param.sp8_z = 0; % m
fr3.param.m8 = 1e-5; % kg
fr3.param.I8_xx = 1e-6; % kgm^2
fr3.param.I8_xy = 0; % kgm^2
fr3.param.I8_xz = 0; % kgm^2
fr3.param.I8_yy = 1e-6; % kgm^2
fr3.param.I8_yz = 0; % kgm^2
fr3.param.I8_zz = 1e-6; % kgm^2
fr3.param.sp8_x = 0.0; % m
fr3.param.sp8_y = 0.0; % m
fr3.param.sp8_z = 0.0; % m

% HAND
fr3.param.sp9_x = -0.01; % m
fr3.param.sp9_y = 0.0; % m
fr3.param.sp9_z = 0.03; % m
fr3.param.m9 = 0.73; % kg
fr3.param.I9_xx = 0.001; % kgm^2
fr3.param.I9_xy = 0; % kgm^2
fr3.param.I9_xz = 0; % kgm^2
fr3.param.I9_yy = 0.0025; % kgm^2
fr3.param.I9_yz = 0; % kgm^2
fr3.param.I9_zz = 0.0017; % kgm^2

% FINGER
fr3.param.w_finger = 0.1*0; % m
fr3.param.l_finger = 0.0584; % m, fixed fr3_finger_joint1, fr3_hand_tcp to (fr3_leftfinger, fr3_rightfinger)
fr3.param.sp_finger_x = 0.0; % m
fr3.param.sp_finger_y = 0.0; % m
fr3.param.sp_finger_z = 0.0; % m
fr3.param.m_finger = 0.015; % kg
fr3.param.I_finger_xx = 2.375e-06; % kgm^2
fr3.param.I_finger_xy = 0; % kgm^2
fr3.param.I_finger_xz = 0; % kgm^2
fr3.param.I_finger_yy = 2.375e-06; % kgm^2
fr3.param.I_finger_yz = 0; % kgm^2
fr3.param.I_finger_zz = 7.5e-07; % kgm^2

% Define sugihara limb vector
sugihara_limb_vector = [fr3.param.l1^2; fr3.param.l2^2; fr3.param.l3^2; fr3.param.l4^2; fr3.param.l5^2; fr3.param.l6^2; fr3.param.l7^2];
fr3.param.sugihara_limb_vector = sugihara_limb_vector;

fr3.param.n_indices_fixed = [];
fr3.param.n_indices = 1:n;