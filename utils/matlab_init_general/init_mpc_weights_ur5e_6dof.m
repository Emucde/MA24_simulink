x_min = [param_robot.q_limit_lower; param_robot.q_p_limit_lower];
x_max = [param_robot.q_limit_upper; param_robot.q_p_limit_upper];

u_min = [param_robot.torque_limit_lower];
u_max = [param_robot.torque_limit_upper];

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 1) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC01';
param_weight.(MPC).Q_y      = 1e3*diag([1*ones(3,1); 1*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN     = 1e5*diag([1*ones(3,1); 1*ones(3,1)]);  % D_N
param_weight.(MPC).R_q_pp   = 1e-10*diag(ones(n,1));  % c_kpn

param_weight.(MPC).x_min    = x_min;
param_weight.(MPC).x_max    = x_max;
param_weight.(MPC).u_min    = u_min;
param_weight.(MPC).u_max    = u_max;
% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 6) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC6';
param_weight.(MPC).Q_y        = diag([1e5*ones(3,1); 1e3*ones(3,1)]);  % d_kpn

param_weight.(MPC).R_q_pp     = 1e-3*diag(ones(n,1));  % d_kpn

param_weight.(MPC).Q_y_p_ref  = diag([50*ones(3,1); 50*ones(3,1)]);
param_weight.(MPC).Q_y_ref    = diag([50*ones(3,1); 50*ones(3,1)]);

param_weight.(MPC).epsilon_t = 1e-5;%1e-5;
param_weight.(MPC).epsilon_r = 1e-5;%inf;

param_weight.(MPC).x_min    = x_min; 
param_weight.(MPC).x_max    = x_max; 
param_weight.(MPC).u_min    = u_min; 
param_weight.(MPC).u_max    = u_max; 
% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 7) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC7';
param_weight.(MPC).Q_y        = diag([1e3*ones(3,1); 1e3*ones(3,1)]);  % d_kpn

param_weight.(MPC).R_q_pp     = 1e-10*diag(ones(n,1));  % d_kpn

param_weight.(MPC).Q_y_p_ref  = diag([100*ones(3,1); 100*ones(3,1)]);
param_weight.(MPC).Q_y_ref    = diag([100*ones(3,1); 100*ones(3,1)]);

param_weight.(MPC).epsilon_t = 1e-5;%1e-5;
param_weight.(MPC).epsilon_r = 1e-5;%inf;

param_weight.(MPC).x_min    = x_min; 
param_weight.(MPC).x_max    = x_max; 
param_weight.(MPC).u_min    = u_min; 
param_weight.(MPC).u_max    = u_max; 
% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 8) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC8';
param_weight.(MPC).Q_y        = diag([1e3*ones(3,1); 1e3*ones(3,1)]);  % d_kpn

param_weight.(MPC).R_q_pp     = 1e-2*diag(ones(n,1));  % d_kpn

param_weight.(MPC).Q_y_p_ref  = diag([100*ones(3,1); 100*ones(3,1)]);
param_weight.(MPC).Q_y_ref    = diag([100*ones(3,1); 100*ones(3,1)]);

K_D_q = 100*eye(n);     param_jointspace_ct.(MPC).K_D_q  = K_D_q;
K_P_q = K_D_q^2/4;      param_jointspace_ct.(MPC).K_P_q  = K_P_q;

param_weight.(MPC).epsilon_t = 1e-5;%1e-5;
param_weight.(MPC).epsilon_r = 1e-5;%inf;

% param_weight.(MPC).x_min    = x_min; 
% param_weight.(MPC).x_max    = x_max; 
% param_weight.(MPC).u_min    = u_min; 
% param_weight.(MPC).u_max    = u_max; 
param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

%%%%%%%%%%%%%%%%%%% generate param MPC weights struct %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
names = fieldnames(param_weight)';
param_weight_init = struct;
for name=names
    mpc_name = name{1};
    temp = merge_cell_arrays(struct2cell(param_weight.(mpc_name))');
    param_weight_init.(mpc_name) = temp{1};
end