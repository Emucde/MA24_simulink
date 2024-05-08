%% PARAM ROBOT FR3
param_robot = struct;

param_robot.n_DOF = 7; % DOF
param_robot.m_t   = 3; % Translational task space
param_robot.m_r   = 3; % Rotational task space
param_robot.m     = param_robot.m_t + param_robot.m_r; % Task space dimension

param_robot.g = 9.81; %m/s
%% Robot gravity
param_robot.g_vis=[0;0;-9.81];

param_robot.q_limit_lower = q_min;
param_robot.q_limit_upper = q_max;

param_robot.q_p_limit_lower = q_dot_max;
param_robot.q_p_limit_upper = q_dot_min;

param_robot.q_n = (q_max + q_min) / 2;

param_robot.torque_limit_lower = tau_min;
param_robot.torque_limit_upper = tau_max;

% Inertial System
param_robot.p_0 = [0; 0; 0]; % m
param_robot.R_0 = eye(3); % m

% LINK 0 (BASE)
param_robot.sp0_x = -0.041018; % m
param_robot.sp0_y = -0.00014; % m
param_robot.sp0_z = 0.049974; % m
param_robot.m0 = 0.629769; % kg
param_robot.I0_xx = 0.00315; % kgm^2
param_robot.I0_xy = 8.2904e-07; % kgm^2
param_robot.I0_xz = 0.00015; % kgm^2
param_robot.I0_yy = 0.00388; % kgm^2
param_robot.I0_yz = 8.2299e-06; % kgm^2
param_robot.I0_zz = 0.004285; % kgm^2

% LINK 1
param_robot.l1 = 0.333; % m
param_robot.sp1_x = 0.003875; % m
param_robot.sp1_y = 0.002081; % m
param_robot.sp1_z = -0.04762; % m
param_robot.m1 = 4.970684; % kg
param_robot.I1_xx = 0.70337; % kgm^2
param_robot.I1_xy = -0.000139; % kgm^2
param_robot.I1_xz = 0.006772; % kgm^2
param_robot.I1_yy = 0.70661; % kgm^2
param_robot.I1_yz = 0.019169; % kgm^2
param_robot.I1_zz = 0.009117; % kgm^2

% LINK 2
param_robot.l2 = 0.316; % m
param_robot.sp2_x = -0.003141; % m
param_robot.sp2_y = -0.02872; % m
param_robot.sp2_z = 0.003495; % m
param_robot.m2 = 0.646926; % kg
param_robot.I2_xx = 0.007962; % kgm^2
param_robot.I2_xy = -0.003925; % kgm^2
param_robot.I2_xz = 0.010254; % kgm^2
param_robot.I2_yy = 0.02811; % kgm^2
param_robot.I2_yz = 0.000704; % kgm^2
param_robot.I2_zz = 0.025995; % kgm^2

% LINK 3
param_robot.l3 = 0.0825; % m
param_robot.sp3_x = 0.027518; % m
param_robot.sp3_y = 0.039252; % m
param_robot.sp3_z = -0.066502; % m
param_robot.m3 = 3.228604; % kg
param_robot.I3_xx = 0.037242; % kgm^2
param_robot.I3_xy = -0.004761; % kgm^2
param_robot.I3_xz = -0.011396; % kgm^2
param_robot.I3_yy = 0.036155; % kgm^2
param_robot.I3_yz = -0.012805; % kgm^2
param_robot.I3_zz = 0.01083; % kgm^2

% LINK 4
param_robot.l4 = 0.384; % m
param_robot.sp4_x = -0.05317; % m
param_robot.sp4_y = 0.104419; % m
param_robot.sp4_z = 0.027454; % m
param_robot.m4 = 3.587895; % kg
param_robot.I4_xx = 0.025853; % kgm^2
param_robot.I4_xy = 0.007796; % kgm^2
param_robot.I4_xz = -0.001332; % kgm^2
param_robot.I4_yy = 0.019552; % kgm^2
param_robot.I4_yz = 0.008641; % kgm^2
param_robot.I4_zz = 0.028323; % kgm^2

% LINK 5
param_robot.l5 = 0.088; % m
param_robot.sp5_x = -0.011953; % m
param_robot.sp5_y = 0.041065; % m
param_robot.sp5_z = -0.038437; % m
param_robot.m5 = 1.225946; % kg
param_robot.I5_xx = 0.035549; % kgm^2
param_robot.I5_xy = -0.002117; % kgm^2
param_robot.I5_xz = -0.004037; % kgm^2
param_robot.I5_yy = 0.029474; % kgm^2
param_robot.I5_yz = 0.000229; % kgm^2
param_robot.I5_zz = 0.008627; % kgm^2

% LINK 6
param_robot.l6 = 0.107; % m
param_robot.sp6_x = 0.060149; % m
param_robot.sp6_y = -0.014117; % m
param_robot.sp6_z = -0.010517; % m
param_robot.m6 = 1.666555; % kg
param_robot.I6_xx = 0.001964; % kgm^2
param_robot.I6_xy = 0.000109; % kgm^2
param_robot.I6_xz = -0.001158; % kgm^2
param_robot.I6_yy = 0.004354; % kgm^2
param_robot.I6_yz = 0.000341; % kgm^2
param_robot.I6_zz = 0.005433; % kgm^2

% LINK 7
param_robot.l7 = 0.1034; % m
param_robot.sp7_x = 0.010517; % m
param_robot.sp7_y = -0.004252; % m
param_robot.sp7_z = 0.061597; % m
param_robot.m7 = 0.735522; % kg
param_robot.I7_xx = 0.012516; % kgm^2
param_robot.I7_xy = -0.000428; % kgm^2
param_robot.I7_xz = -0.001196; % kgm^2
param_robot.I7_yy = 0.010027; % kgm^2
param_robot.I7_yz = -0.000741; % kgm^2
param_robot.I7_zz = 0.004815; % kgm^2

% LINK 8 (HAND)
param_robot.m8 = 1e-5; % kg
param_robot.I8_xx = 1e-6; % kgm^2
param_robot.I8_xy = 0; % kgm^2
param_robot.I8_xz = 0; % kgm^2
param_robot.I8_yy = 1e-6; % kgm^2
param_robot.I8_yz = 0; % kgm^2
param_robot.I8_zz = 1e-6; % kgm^2
param_robot.sp8_x = 0.0; % m
param_robot.sp8_y = 0.0; % m
param_robot.sp8_z = 0.0; % m

% HAND
param_robot.m9 = 0.73; % kg
param_robot.I9_xx = 0.001; % kgm^2
param_robot.I9_xy = 0; % kgm^2
param_robot.I9_xz = 0; % kgm^2
param_robot.I9_yy = 0.0025; % kgm^2
param_robot.I9_yz = 0; % kgm^2
param_robot.I9_zz = 0.0017; % kgm^2
param_robot.sp9_x = -0.01; % m
param_robot.sp9_y = 0.0; % m
param_robot.sp9_z = 0.03; % m

% FINGER
param_robot.w_finger = 0.1; % m
param_robot.l_finger = 0.0584; % m
param_robot.m_finger = 0.015; % kg
param_robot.I_finger_xx = 2.375e-06; % kgm^2
param_robot.I_finger_xy = 0; % kgm^2
param_robot.I_finger_xz = 0; % kgm^2
param_robot.I_finger_yy = 2.375e-06; % kgm^2
param_robot.I_finger_yz = 0; % kgm^2
param_robot.I_finger_zz = 7.5e-07; % kgm^2
param_robot.sp_finger_x = 0.0; % m
param_robot.sp_finger_y = 0.0; % m
param_robot.sp_finger_z = 0.0; % m