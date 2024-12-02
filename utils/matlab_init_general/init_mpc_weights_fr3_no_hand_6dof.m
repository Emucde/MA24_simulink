x_min = [param_robot.q_limit_lower; param_robot.q_p_limit_lower];
x_max = [param_robot.q_limit_upper; param_robot.q_p_limit_upper];

u_min = [param_robot.torque_limit_lower];
u_max = [param_robot.torque_limit_upper];

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 1) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC01';
param_weight.(MPC).Q_y   = 1e2*diag([1*ones(3,1); ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN  = 1e5*diag([1*ones(3,1); ones(3,1)]);  % D_N

% param_weight.(MPC).R_u0  = 1e-6*diag(ones(n,1));  % c_kpn
% param_weight.(MPC).R_u   = 1e-7*diag(ones(n,1));  % c_kpn
% param_weight.(MPC).R_x0  = 1e-4*diag(ones(2*n,1));  % c_kpn
% param_weight.(MPC).R_x   = 1e-4*diag(ones(2*n,1));  % c_kpn
param_weight.(MPC).R_u0    = 0*diag(ones(n,1));  % c_kpn
param_weight.(MPC).R_u     = 0*diag(ones(n,1));  % c_kpn
param_weight.(MPC).R_x0    = 1e-10*diag([1*ones(n,1); 1e5*ones(n,1)]);
param_weight.(MPC).R_x     = 1e-10*diag([1*ones(n,1); 1e5*ones(n,1)]);

param_weight.(MPC).x_min    = x_min;
param_weight.(MPC).x_max    = x_max;
param_weight.(MPC).u_min    = u_min;
param_weight.(MPC).u_max    = u_max;
param_weight.(MPC).q_pp_min    = param_robot.q_pp_limit_lower; 
param_weight.(MPC).q_pp_max    = param_robot.q_pp_limit_upper; 
% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 
% param_weight.(MPC).q_pp_min = -inf(size(u_max)); %u_max 
% param_weight.(MPC).q_pp_max = +inf(size(u_max)); %u_max 

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 6) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC6';
param_weight.(MPC).Q_y        = diag([1e2*ones(3,1); 1e2*ones(3,1)]);  % d_kpn

param_weight.(MPC).Q_y_p_ref  = diag([100*ones(3,1); 100*ones(3,1)]);
param_weight.(MPC).Q_y_ref    = param_weight.(MPC).Q_y_p_ref^2/4;

param_weight.(MPC).epsilon_t = 1e-5;%1e-5;
param_weight.(MPC).epsilon_r = 1e-5;%inf;

param_weight.(MPC).R_u0  = 1e-10*diag(ones(n,1));  % c_kpn
param_weight.(MPC).R_u   = 1e-10*diag(ones(n,1));  % c_kpn
param_weight.(MPC).R_x0  = 1e-10*diag([1*ones(n,1); 1*ones(n,1)]);
param_weight.(MPC).R_x   = 1e-10*diag([1*ones(n,1); 1*ones(n,1)]);
param_weight.(MPC).R_z     = 1e-10*diag(ones(13,1)); % [zt, zr, d/dt zt, d/dt zr] = [3+4+3+3] = 13 (zr as quaternion)
param_weight.(MPC).R_alpha = 1e-10*diag(ones(6,1));

param_weight.(MPC).x_min    = x_min; 
param_weight.(MPC).x_max    = x_max; 
param_weight.(MPC).u_min    = u_min; 
param_weight.(MPC).u_max    = u_max;
param_weight.(MPC).q_pp_min    = param_robot.q_pp_limit_lower; 
param_weight.(MPC).q_pp_max    = param_robot.q_pp_limit_upper; 
% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max
% param_weight.(MPC).q_pp_min = -inf(size(u_max)); %u_max 
% param_weight.(MPC).q_pp_max = +inf(size(u_max)); %u_max 

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 7) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC7';
param_weight.(MPC).Q_y        = diag([1000*ones(3,1); 100*ones(3,1)]);  % d_kpn

param_weight.(MPC).Q_y_p_ref  = diag([100*ones(3,1); 10*ones(3,1)]);
param_weight.(MPC).Q_y_ref    = diag([100*ones(3,1); 10*ones(3,1)]);

param_weight.(MPC).epsilon_t = 1e-5;%1e-5;
param_weight.(MPC).epsilon_r = 1e-5;%inf;

param_weight.(MPC).R_u0  = 1e-10*diag(ones(n,1));  % c_kpn
param_weight.(MPC).R_u   = 1e-10*diag(ones(n,1));  % c_kpn
param_weight.(MPC).R_x0  = 1e-10*diag([1*ones(n,1); 1e5*ones(n,1)]);
param_weight.(MPC).R_x   = 1e-10*diag([1*ones(n,1); 1e5*ones(n,1)]);
param_weight.(MPC).R_z     = 1e-10*diag(ones(13,1)); % [zt, zr, d/dt zt, d/dt zr] = [3+4+3+3] = 13 (zr as quaternion)
param_weight.(MPC).R_alpha = 1e-10*diag(ones(6,1));

param_weight.(MPC).x_min    = x_min; 
param_weight.(MPC).x_max    = x_max; 
param_weight.(MPC).u_min    = u_min; 
param_weight.(MPC).u_max    = u_max; 
param_weight.(MPC).q_pp_min    = param_robot.q_pp_limit_lower; 
param_weight.(MPC).q_pp_max    = param_robot.q_pp_limit_upper; 
% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 
% param_weight.(MPC).q_pp_min = -inf(size(u_max)); %u_max 
% param_weight.(MPC).q_pp_max = +inf(size(u_max)); %u_max 

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 8) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kinematic mpc with integration without refsys after thelenberg
MPC='MPC8';
param_weight.(MPC).Q_y      = 1e2*diag([1*ones(3,1); 1e-2*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_ykp1   = 1e2*diag([1*ones(3,1); 1e-2*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN     = 1e5*diag([1*ones(3,1); 1e-2*ones(3,1)]);  % D_N

% stretch arm other dir, Tsmpc=200ms pred
param_weight.(MPC).R_q_p    = 1e-5*diag(ones(n, 1));
param_weight.(MPC).R_q_pp   = 1e-10*diag(ones(n, 1));
param_weight.(MPC).R_delta_x0 = 1e-5*diag([1*ones(n,1); 0*ones(n,1)]);
param_weight.(MPC).R_delta_x  = param_weight.(MPC).R_delta_x0;
param_weight.(MPC).R_delta_u  = 0*diag(ones(n, 1));

param_jointspace_ct.(MPC).K_P_q = 5000*eye(n);
param_jointspace_ct.(MPC).K_D_q = sqrt(2*param_jointspace_ct.(MPC).K_P_q);

% stretch arm noise Tsmpc=5ms pred
% param_weight.(MPC).R_q_p    = 1e-2*diag(ones(n, 1));
% param_weight.(MPC).R_q_pp   = 1e-5*diag(ones(n, 1));
% param_weight.(MPC).R_delta_x0 = 1e-2*diag([1*ones(n,1); 0*ones(n,1)]);
% param_weight.(MPC).R_delta_x  = param_weight.(MPC).R_delta_x0;
% param_weight.(MPC).R_delta_u  = 0*diag(ones(n, 1));

% param_weight.(MPC).Q_yt_tang = 1e2*diag([1*ones(3,1)]);
% param_weight.(MPC).Q_yt_N_tang = 1e5*diag([1*ones(3,1)]);

% param_weight.(MPC).Q_yt_perp = 1e2*diag([1*ones(3,1)]);
% param_weight.(MPC).Q_yt_N_perp = 1e5*diag([1*ones(3,1)]);

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

param_jointspace_ct.(MPC).K_P_q = 5000*eye(n);
param_jointspace_ct.(MPC).K_D_q = sqrt(2*param_jointspace_ct.(MPC).K_P_q);

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
% kinematic mpc with numerical derivative without reference system
MPC='MPC10';

param_weight.(MPC).Q_y    = 1e2*diag([1*ones(3,1); 1e-2*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN   = 1e5*diag([1*ones(3,1); 1e-2*ones(3,1)]);  % D_N
param_weight.(MPC).R_q_p   = 1e-2*diag(ones(n,1));  % d_kpn
param_weight.(MPC).R_q_pp   = 1e-5*diag(ones(n,1));  % d_kpn
param_weight.(MPC).R_q_prev = 1e-10*diag(ones(n,1));

param_jointspace_ct.(MPC).K_P_q = 1000*eye(n);
param_jointspace_ct.(MPC).K_D_q = 2*sqrt(param_jointspace_ct.(MPC).K_P_q);

% traj strecht arm out Ts5ms
% param_weight.(MPC).Q_y    = 1e2*diag([1*ones(3,1); 1*ones(3,1)]);  % d_kpn
% param_weight.(MPC).Q_yN   = 1e5*diag([1*ones(3,1); 1*ones(3,1)]);  % D_N
% param_weight.(MPC).R_q_pp = 1e-4*diag(ones(n,1));  % d_kpn
% param_weight.(MPC).R_x0     = 1*diag([1*ones(n,1); 1*ones(n,1)]);
% param_weight.(MPC).R_x      = 1*diag([1*ones(n,1); 1*ones(n,1)]);

%param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
%param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
%param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
%param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

param_weight.(MPC).x_min    = x_min;
param_weight.(MPC).x_max    = x_max;
param_weight.(MPC).u_min    = param_robot.q_pp_limit_lower;
param_weight.(MPC).u_max    = param_robot.q_pp_limit_upper;

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 11) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kinematic integrator path following mpc v6
MPC='MPC11';
param_weight.(MPC).Q_y    = 1e2*diag([1*ones(3,1); 1e-2*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN   = 1e5*diag([1*ones(3,1); 1e-2*ones(3,1)]);  % D_N
param_weight.(MPC).Q_theta = 1e5;
param_weight.(MPC).Q_thetaN = 1e8;
param_weight.(MPC).lambda_theta = 1;

param_weight.(MPC).R_q_pp = 1e-5*diag(ones(n,1));
param_weight.(MPC).R_x    = diag([0*ones(n,1); 1e-1*ones(n,1)]);
param_weight.(MPC).R_x0   = param_weight.(MPC).R_x;
param_weight.(MPC).R_theta_prev =0;

param_jointspace_ct.(MPC).K_P_q = 5000*eye(n);
param_jointspace_ct.(MPC).K_D_q = sqrt(2*param_jointspace_ct.(MPC).K_P_q);

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
param_weight.(MPC).Q_y    = 1e2*diag([1*ones(3,1); 1e-2*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN   = 1e5*diag([1*ones(3,1); 1e-2*ones(3,1)]);  % D_N

% stretch sing
% param_weight.(MPC).R_q_pp = 1e-5*diag(ones(n,1));
% param_weight.(MPC).R_x0   = 1e-2*diag([1*ones(n,1); 1*ones(n,1)]);
% param_weight.(MPC).R_x    = 1e-2*diag([1*ones(n,1); 1*ones(n,1)]);

param_weight.(MPC).R_q_pp = 1e-5*diag(ones(n,1));
param_weight.(MPC).R_x0   = 1e-2*diag([1*ones(n,1); 1*ones(n,1)]);
param_weight.(MPC).R_x    = 1e-2*diag([1*ones(n,1); 1*ones(n,1)]);

param_jointspace_ct.(MPC).K_P_q = 500*eye(n);
param_jointspace_ct.(MPC).K_D_q = sqrt(2*param_jointspace_ct.(MPC).K_P_q);

% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

param_weight.(MPC).x_min    = x_min;
param_weight.(MPC).x_max    = x_max;
param_weight.(MPC).u_min    = param_robot.q_pp_limit_lower;
param_weight.(MPC).u_max    = param_robot.q_pp_limit_upper;

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 10) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kinematic derivative planner mpc v8

MPC='MPC13';

param_weight.(MPC).Q_y    = 1e2*diag([ones(3,1); 1e-2*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN   = 1e5*diag([ones(3,1); 1e-2*ones(3,1)]);  % D_N
param_weight.(MPC).R_q_pp   = 1e-5*diag(ones(n,1));  % d_kpn
param_weight.(MPC).R_x0     = diag([0*ones(n,1); 1e-2*ones(n,1)]);
param_weight.(MPC).R_x      = param_weight.(MPC).R_x0;

param_jointspace_ct.(MPC).K_P_q = 500*eye(n);
param_jointspace_ct.(MPC).K_D_q = sqrt(2*param_jointspace_ct.(MPC).K_P_q);

% param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
% param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
% param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
% param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

param_weight.(MPC).x_min    = x_min;
param_weight.(MPC).x_max    = x_max;
param_weight.(MPC).u_min    = param_robot.q_pp_limit_lower;
param_weight.(MPC).u_max    = param_robot.q_pp_limit_upper;

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 14) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kinematic derivative path following mpc v6
MPC='MPC14';
param_weight.(MPC).Q_y    = 1e2*diag([1*ones(3,1); 1e-2*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN   = 1e5*diag([1*ones(3,1); 1e-2*ones(3,1)]);  % D_N
param_weight.(MPC).Q_theta = 1e5;
param_weight.(MPC).Q_thetaN = 1e8;
param_weight.(MPC).lambda_theta = 1;

param_weight.(MPC).R_q_pp = 1e-5*diag(ones(n,1));
param_weight.(MPC).R_x    = diag([0*ones(n,1); 1e-1*ones(n,1)]);
param_weight.(MPC).R_x0   = param_weight.(MPC).R_x;
param_weight.(MPC).R_theta_prev =0;

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