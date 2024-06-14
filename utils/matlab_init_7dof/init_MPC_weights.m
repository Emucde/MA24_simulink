x_min = [param_robot.q_limit_lower; param_robot.q_p_limit_lower];
x_max = [param_robot.q_limit_upper; param_robot.q_p_limit_upper];

u_min = [param_robot.torque_limit_lower];
u_max = [param_robot.torque_limit_upper];

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 1) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC1';
param_weight.(MPC).Q_y      = 1e3*diag([0*ones(3,1); 1e2*ones(3,1)]);  % d_kpn
param_weight.(MPC).Q_yN     = 1e5*diag([0*ones(3,1); 1e2*ones(3,1)]);  % D_N
param_weight.(MPC).R_q_pp   = 1e-2*diag(ones(n,1));  % c_kpn

%param_weight.(MPC).x_min    = x_min; 
%param_weight.(MPC).x_max    = x_max; 
%param_weight.(MPC).u_min    = u_min; 
%param_weight.(MPC).u_max    = u_max; 
param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 6) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC6';
%m = 6;
%param_weight.(MPC).Q_y        = 1e5*diag([1 1 1]);  % d_kpn
param_weight.(MPC).Q_y        = diag([1e3*ones(3,1); 1e3*ones(3,1)]);  % d_kpn

param_weight.(MPC).R_q_pp     = 1e-10*diag(ones(n,1));  % d_kpn

alpha_var = 4;
if(alpha_var == 1)
    param_weight.(MPC).Q_y_p_ref  = diag([100*ones(3,1); 100]);
    param_weight.(MPC).Q_y_ref    = param_weight.(MPC).Q_y_p_ref^2/4;
elseif(alpha_var == 2)
    param_weight.(MPC).Q_y_p_ref  = diag([100*ones(3,1); 100*ones(4,1)]);
    param_weight.(MPC).Q_y_ref    = param_weight.(MPC).Q_y_p_ref^2/4;
elseif(alpha_var == 3 || alpha_var == 4)
    param_weight.(MPC).Q_y_p_ref  = diag([100*ones(3,1); 100*ones(3,1)]);
    param_weight.(MPC).Q_y_ref    = diag([100*ones(3,1); 100*ones(3,1)]);
else
    error(['alpha_var not correct defined: ', num2str(alpha_var), ' (1=alpha, 2=quaternion, 3 or 4=euler angles or quat omega)']);
end

param_weight.(MPC).epsilon_t = 1e-5;
param_weight.(MPC).epsilon_r = inf;

%param_weight.(MPC).x_min    = x_min; 
%param_weight.(MPC).x_max    = x_max; 
%param_weight.(MPC).u_min    = u_min; 
%param_weight.(MPC).u_max    = u_max; 
param_weight.(MPC).x_min    = -inf(size(x_min)); %x_min 
param_weight.(MPC).x_max    = +inf(size(x_max)); %x_max 
param_weight.(MPC).u_min    = -inf(size(u_min)); %u_min 
param_weight.(MPC).u_max    = +inf(size(u_max)); %u_max 

%%%%%%%%%%%%%%%%%%%%% FEASIBLE BLOCK %%%%%%%%%%%%%%%%%%%%%
MPC='FeasibleBlock';
param_weight.(MPC).q_min    = param_robot.q_limit_lower;
param_weight.(MPC).q_max    = param_robot.q_limit_upper;

names = fieldnames(param_weight)';
param_weight_init = struct;
for name=names
    mpc_name = name{1};
    temp = merge_cell_arrays(struct2cell(param_weight.(mpc_name))');
    param_weight_init.(mpc_name) = temp{1};
end

%{
param_weight.(MPC).Q_y      = 1e5*diag(ones(6,1));  % d_kpn
param_weight.(MPC).Q_y_p    = 1e-5*diag(ones(6,1)); % d_kpn
param_weight.(MPC).Q_y_pp   = 1e-5*diag(ones(6,1)); % d_kpn
param_weight.(MPC).Q_y0_pp  = 1e-5*diag(ones(6,1)); % D_0
param_weight.(MPC).Q_y1     = 1e5*diag(ones(6,1));  % D_1
param_weight.(MPC).Q_y1_p   = 1e-5*diag(ones(6,1)); % D_1
param_weight.(MPC).Q_y1_pp  = 1e-5*diag(ones(6,1)); % D_1
param_weight.(MPC).Q_yN     = 1e1*diag(ones(6,1));  % D_N
param_weight.(MPC).Q_yN_p   = 1e-5*diag(ones(6,1)); % D_N

param_weight.(MPC).Q_q_p    = 0*diag(ones(6,1));  % c_kpn
param_weight.(MPC).Q_q_pp   = 0*diag(ones(6,1));  % c_kpn
param_weight.(MPC).Q_q0_pp  = 0*diag(ones(6,1));  % C_0
param_weight.(MPC).Q_qN_p   = 0*diag(ones(6,1));  % C_N

param_weight.(MPC).x_min    = inf*[ -pi; -pi; -20; -20 ];
param_weight.(MPC).x_max    = inf*[ pi; pi; 20; 20];
param_weight.(MPC).u_min    = inf*[ -10; -10 ];
param_weight.(MPC).u_max    = inf*[ 10; 10 ];
%}

%{
% TODO: FOR WITH ALL STRUCTS
% Fehlerprüfung
% Prüfe, ob alle Parameter gesetzt sind
if ~exist('QQ', 'var') || ~exist('RR_u', 'var') || ...
       ~exist('RR_du', 'var') || ~exist('RR_dx', 'var') || ...
       ~exist('RR_dx_km1', 'var') || ~exist('xx_min', 'var') || ...
       ~exist('xx_max', 'var') || ~exist('uu_min', 'var') || ...
       ~exist('uu_max', 'var')
    error('Es ist ein Fehler aufgetreten: Eine Variable ist nicht definiert.');
end

% Prüfe, ob alle Parameter gültig sind
if any(isnan(QQ(:))) || any(isnan(RR_u(:))) || any(isnan(RR_du(:))) || ...
       any(isnan(RR_dx(:))) || any(isnan(RR_dx_km1(:))) || ...
       any(isnan(xx_min(:))) || any(isnan(xx_max(:))) || ...
       any(isnan(uu_min(:))) || any(isnan(uu_max(:)))
    error('Es ist ein Fehler aufgetreten: Ein Parameter ist ungültig.');
end

% Prüfe, ob die Grenzen für x und u gültig sind
if any(xx_min >= xx_max) || any(uu_min >= uu_max)
    error('Es ist ein Fehler aufgetreten: Die Grenzen für x oder u sind ungültig.');
end

%}