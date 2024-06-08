% In diesem File werden Busse definiert
% https://de.mathworks.com/matlabcentral/answers/29638-rename-bus

% to check try 'typeeditor'

%% debug data bus
bus_temp_struct = struct;
bus_temp_struct.p           = [0; 0; 0];
bus_temp_struct.p_p         = [0; 0; 0];
bus_temp_struct.p_pp        = [0; 0; 0];
bus_temp_struct.Phi         = [0; 0; 0];
bus_temp_struct.Phi_p       = [0; 0; 0];
bus_temp_struct.Phi_pp      = [0; 0; 0];
bus_temp_struct.q           = [0; 0; 0; 0];
bus_temp_struct.omega       = [0; 0; 0];
bus_temp_struct.omega_p     = [0; 0; 0];
bus_temp_struct.p_err       = [0; 0; 0];
bus_temp_struct.p_p_err     = [0; 0; 0];
bus_temp_struct.p_pp_err    = [0; 0; 0];
bus_temp_struct.q_err       = [0; 0; 0; 0];
bus_temp_struct.omega_err   = [0; 0; 0];
bus_temp_struct.omega_p_err = [0; 0; 0];

init_debug_data_bus = bus_temp_struct;
debug_data_bus = eval(Simulink.Bus.createObject(bus_temp_struct).busName);
clear -regexp slBus; clear bus_temp_struct;

%% trajectory bus init (already defined in bus.mat)
bus_temp_struct            = struct;
bus_temp_struct.p_d        = [0; 0; 0];
bus_temp_struct.p_d_p      = [0; 0; 0];
bus_temp_struct.p_d_pp     = [0; 0; 0];
bus_temp_struct.Phi_d      = [0; 0; 0];
bus_temp_struct.Phi_d_p    = [0; 0; 0];
bus_temp_struct.Phi_d_pp   = [0; 0; 0];
bus_temp_struct.R_d        = eye(3);
bus_temp_struct.q_d        = [0; 0; 0; 0]; % quaternion
bus_temp_struct.q_d_p      = [0; 0; 0; 0];
bus_temp_struct.q_d_pp     = [0; 0; 0; 0];
bus_temp_struct.omega_d    = [0; 0; 0];
bus_temp_struct.omega_d_p  = [0; 0; 0];
bus_temp_struct.alpha_d    = 0;
bus_temp_struct.alpha_d_p  = 0;
bus_temp_struct.alpha_d_pp = 0;
bus_temp_struct.rot_ax_d   = [0; 0; 0];

init_bus_param.init_x_d_bus = bus_temp_struct;
x_d = eval(Simulink.Bus.createObject(bus_temp_struct).busName);
clear -regexp slBus; clear bus_temp_struct;

%% debug position and orientation bus for ct control
clear bus_temp_struct;

bus_temp_struct.x = 0;
bus_temp_struct.x_d = 0;
bus_temp_struct.y =  0;
bus_temp_struct.y_d = 0;
bus_temp_struct.z =  0;
bus_temp_struct.z_d = 0;
bus_temp_struct.phi = 0;
bus_temp_struct.phi_d = 0;
bus_temp_struct.theta = 0;
bus_temp_struct.theta_d = 0;
bus_temp_struct.psi = 0;
bus_temp_struct.psi_d = 0;
bus_temp_struct.rot_x_err = 0;
bus_temp_struct.rot_y_err = 0;
bus_temp_struct.rot_z_err = 0;
init_bus_param.init_yy_debug_bus = bus_temp_struct;
yy_debug_bus = eval(Simulink.Bus.createObject(bus_temp_struct).busName);
clear -regexp slBus; clear bus_temp_struct;

%% robot model bus init

bus_temp_struct = struct;
bus_temp_struct.q = zeros(n,1);
bus_temp_struct.q_p = zeros(n,1);
bus_temp_struct.q_pp = zeros(n,1);
bus_temp_struct.H = zeros(4);
bus_temp_struct.J = zeros(6, n);
bus_temp_struct.J_p = zeros(6, n);
bus_temp_struct.M = zeros(n);
%bus_temp_struct.C = zeros(n);
bus_temp_struct.C_rnea = zeros(n, 1); % = C_rnea(q, q_p) = C(q, q_p)q_p + g(q) = n(q, q_p)
bus_temp_struct.g = zeros(n,1);
init_bus_param.init_robot_model = bus_temp_struct;
robot_model = eval(Simulink.Bus.createObject(bus_temp_struct).busName);
clear -regexp slBus; clear bus_temp_struct;

%% Collinearity bus init

bus_temp_struct = struct;
if(n == 7)
    bus_temp_struct.JJ_Y12_B13_R14 = zeros(3,1);
    bus_temp_struct.JJ_Y15_B16_R17 = zeros(3,1);
    bus_temp_struct.JJ_Y23_B24_R25 = zeros(3,1);
    bus_temp_struct.JJ_Y26_B27_R34 = zeros(3,1);
    bus_temp_struct.JJ_Y35_B36_R37 = zeros(3,1);
    bus_temp_struct.JJ_Y45_B46_R47 = zeros(3,1);
    bus_temp_struct.JJ_Y56_B57_R67 = zeros(3,1);
elseif(n == 6)
    bus_temp_struct.JJ_Y12_B13_R14 = zeros(3,1);
    bus_temp_struct.JJ_Y15_B16_R23 = zeros(3,1);
    bus_temp_struct.JJ_Y24_B25_R26 = zeros(3,1);
    bus_temp_struct.JJ_Y34_B35_R36 = zeros(3,1);
    bus_temp_struct.JJ_Y45_B46_R56 = zeros(3,1);
else
    error('n not correct defined');
end
init_bus_param.init_collinearity_bus = bus_temp_struct;
collinearity_bus = eval(Simulink.Bus.createObject(bus_temp_struct).busName);
clear -regexp slBus; clear bus_temp_struct;

%% testing bus

bus_temp_struct = struct;
bus_temp_struct.mat_fun = 0;
bus_temp_struct.pyt_fun = 0;
init_bus_param.init_test_bus = bus_temp_struct;
test_bus = eval(Simulink.Bus.createObject(bus_temp_struct).busName);
clear -regexp slBus; clear bus_temp_struct;

%% Bus init definitions
% Diese Structs braucht man in den Matlab Funktionen da die Busse
% initialisiert werden müssen.
init_bus_param.init_debug_data_bus = init_debug_data_bus; %Simulink.Bus.createMATLABStruct('debug_snap_data_bus');