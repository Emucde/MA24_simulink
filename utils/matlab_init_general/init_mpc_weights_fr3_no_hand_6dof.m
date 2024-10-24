x_min = [param_robot.q_limit_lower; param_robot.q_p_limit_lower];
x_max = [param_robot.q_limit_upper; param_robot.q_p_limit_upper];

u_min = [param_robot.torque_limit_lower];
u_max = [param_robot.torque_limit_upper];

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 1) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC01';
param_weight.(MPC).Q_y   = 1e2*diag([1*ones(3,1); 1*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN  = 1e5*diag([1*ones(3,1); 1*ones(3,1)]);  % D_N
param_weight.(MPC).R_u   = 1e-10*diag(ones(n,1));  % c_kpn
param_weight.(MPC).R_x   = 1e-10*diag(ones(2*n,1));  % c_kpn

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
param_weight.(MPC).Q_y        = diag([1e1*ones(3,1); 1e1*ones(3,1)]);  % d_kpn

param_weight.(MPC).R_q_pp     = 1e-10*diag(ones(n,1));  % d_kpn

param_weight.(MPC).Q_y_p_ref  = diag([10*ones(3,1); 10*ones(3,1)]);
param_weight.(MPC).Q_y_ref    = diag([10*ones(3,1); 10*ones(3,1)]);

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
param_weight.(MPC).Q_y        = diag([1000*ones(3,1); 100*ones(3,1)]);  % d_kpn

param_weight.(MPC).R_q_pp     = 1e-10*diag(ones(n,1));  % d_kpn

param_weight.(MPC).Q_y_p_ref  = diag([100*ones(3,1); 10*ones(3,1)]);
param_weight.(MPC).Q_y_ref    = diag([100*ones(3,1); 10*ones(3,1)]);

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
% kinematic mpc with integration without refsys after thelenberg
MPC='MPC8';
param_weight.(MPC).Q_y      = 1e2*diag([1*ones(3,1); 1*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_ykp1   = 1e2*diag([1*ones(3,1); 1*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN     = 1e5*diag([1*ones(3,1); 1*ones(3,1)]);  % D_N
param_weight.(MPC).R_q_pp   = 1e-2*diag(ones(n,1));
param_weight.(MPC).R_x      = 1e-2*diag(ones(2*n,1));

param_weight.(MPC).x_min    = x_min; 
param_weight.(MPC).x_max    = x_max; 
param_weight.(MPC).u_min    = param_robot.q_pp_limit_lower; 
param_weight.(MPC).u_max    = param_robot.q_pp_limit_upper; 
% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 9) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kinematic mpc with integration and refsys for u
MPC='MPC9';
param_weight.(MPC).Q_y     = diag([1e2*ones(3,1); 1e2*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN    = diag([1e5*ones(3,1); 1e5*ones(3,1)]);  % D_N
param_weight.(MPC).R_q_pp  = 1e-10*diag(ones(n,1));  % d_kpn
param_weight.(MPC).R_v     = 1e-10*diag(ones(n,1));  % d_kpn

param_weight.(MPC).lambda_u  = 10*ones(n,1);

% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max
% param_weight.(MPC).v_min    = -inf(size(param_robot.q_pp_limit_lower)); %x_min 
% param_weight.(MPC).v_max    = +inf(size(param_robot.q_pp_limit_upper)); %x_min 


param_weight.(MPC).x_min    = x_min;
param_weight.(MPC).x_max    = x_max;
param_weight.(MPC).u_min    = param_robot.q_pp_limit_lower;
param_weight.(MPC).u_max    = param_robot.q_pp_limit_upper;
param_weight.(MPC).v_min    = param_robot.q_pp_limit_lower;
param_weight.(MPC).v_max    = param_robot.q_pp_limit_upper;

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 10) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kinematic mpc with numerical deviation without reference system
MPC='MPC10';

param_weight.(MPC).Q_y    = diag([1e0*ones(3,1); 1e0*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN   = diag([1e2*ones(3,1); 1e2*ones(3,1)]);  % D_N
param_weight.(MPC).R_q_pp = 1e-10*diag(ones(n,1));  % d_kpn

% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

param_weight.(MPC).x_min    = x_min;
param_weight.(MPC).x_max    = x_max;
param_weight.(MPC).u_min    = param_robot.q_pp_limit_lower*1;
param_weight.(MPC).u_max    = param_robot.q_pp_limit_upper*1;

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 11) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kinematic integrator path following mpc v6
MPC='MPC11';
param_weight.(MPC).Q_y    = 1e0*diag([1*ones(3,1); 1*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN   = 1e3*diag([1*ones(3,1); 1*ones(3,1)]);  % D_N
param_weight.(MPC).Q_theta = 1e2;
param_weight.(MPC).Q_thetaN = 1e5;
param_weight.(MPC).lambda_theta = 1;

param_weight.(MPC).R_q_pp = 1e-10*diag(ones(n,1));

% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

param_weight.(MPC).x_min    = x_min;
param_weight.(MPC).x_max    = x_max;
param_weight.(MPC).u_min    = param_robot.q_pp_limit_lower;
param_weight.(MPC).u_max    = param_robot.q_pp_limit_upper;

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 12) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kinematic integrator planner mpc v7
MPC='MPC12';
param_weight.(MPC).Q_y    = 1e2*diag([1*ones(3,1); 1*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN   = 1e5*diag([1*ones(3,1); 1*ones(3,1)]);  % D_N

param_weight.(MPC).R_q_pp = 1e-10*diag(ones(n,1));

param_jointspace_ct.(MPC).K_D_q = 64*eye(n);
param_jointspace_ct.(MPC).K_P_q = param_jointspace_ct.(MPC).K_D_q^2/4;

% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

param_weight.(MPC).x_min    = x_min;
param_weight.(MPC).x_max    = x_max;
param_weight.(MPC).u_min    = param_robot.q_pp_limit_lower;
param_weight.(MPC).u_max    = param_robot.q_pp_limit_upper;

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 10) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kinematic deviation planner mpc v8
MPC='MPC13';

param_weight.(MPC).Q_y    = diag([1e2*ones(3,1); 1e2*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN   = diag([1e5*ones(3,1); 1e5*ones(3,1)]);  % D_N
param_weight.(MPC).R_q_pp = 1e-10*diag(ones(n,1));  % d_kpn

param_jointspace_ct.(MPC).K_D_q = 64*eye(n);
param_jointspace_ct.(MPC).K_P_q = param_jointspace_ct.(MPC).K_D_q^2/4;

% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

param_weight.(MPC).x_min    = x_min;
param_weight.(MPC).x_max    = x_max;
param_weight.(MPC).u_min    = param_robot.q_pp_limit_lower;
param_weight.(MPC).u_max    = param_robot.q_pp_limit_upper;

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 14) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kinematic mpc with integration without refsys after thelenberg with only 2 dof (xz plane)
MPC='MPC14';
param_weight.(MPC).Q_y      = 1e2*diag([1*ones(3,1); 1*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_ykp1   = 1e2*diag([1*ones(3,1); 1*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN     = 1e5*diag([1*ones(3,1); 1*ones(3,1)]);  % D_N
param_weight.(MPC).R_q_pp   = 1e-10*diag(ones(n,1));

% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

param_weight.(MPC).x_min    = x_min;
param_weight.(MPC).x_max    = x_max;
param_weight.(MPC).u_min    = param_robot.q_pp_limit_lower;
param_weight.(MPC).u_max    = param_robot.q_pp_limit_upper;

%%%%%%%%%%%%%%%%%%% generate param MPC weights struct %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
names = fieldnames(param_weight)';
param_weight_init = struct;
for name=names
    mpc_name = name{1};
    temp = merge_cell_arrays(struct2cell(param_weight.(mpc_name))');
    param_weight_init.(mpc_name) = temp{1};
end