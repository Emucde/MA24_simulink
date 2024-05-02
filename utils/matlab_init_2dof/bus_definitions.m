% In diesem File werden Busse definiert
% https://de.mathworks.com/matlabcentral/answers/29638-rename-bus

% to check try 'typeeditor'
n = param_robot.n_DOF;

%% debug data bus
bus_temp_struct.p = zeros(n,1);
bus_temp_struct.p_p = zeros(n,1);
bus_temp_struct.p_pp = zeros(n,1);
bus_temp_struct.rot_x_err = 0;
bus_temp_struct.rot_y_err = 0;
bus_temp_struct.rot_z_err = 0;
init_debug_data_bus = bus_temp_struct;
debug_data_bus = eval(Simulink.Bus.createObject(bus_temp_struct).busName);
clear -regexp slBus; clear bus_temp_struct;

%% trajectory bus init (already defined in bus.mat)
bus_temp_struct = struct;
bus_temp_struct.p_d = [0; 0; 0];
bus_temp_struct.p_d_p = [0; 0; 0];
bus_temp_struct.p_d_pp = [0; 0; 0];
bus_temp_struct.q_d = [0; 0; 0; 0];
bus_temp_struct.omega_d = [0; 0; 0];
bus_temp_struct.omega_d_p = [0; 0; 0];
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
bus_temp_struct.C = zeros(n);
bus_temp_struct.g = zeros(n,1);
init_bus_param.init_robot_model = bus_temp_struct;
robot_model = eval(Simulink.Bus.createObject(bus_temp_struct).busName);
clear -regexp slBus; clear bus_temp_struct;

%% Bus init definitions
% Diese Structs braucht man in den Matlab Funktionen da die Busse
% initialisiert werden müssen.
init_bus_param.init_debug_data_bus = init_debug_data_bus; %Simulink.Bus.createMATLABStruct('debug_snap_data_bus');