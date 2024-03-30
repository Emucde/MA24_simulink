%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 1) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC1';
param_weight.(MPC).Q_y      = 1e5*diag([1 1]);  % d_kpn
param_weight.(MPC).Q_y_p    = 1e-5*diag([1 1]); % d_kpn
param_weight.(MPC).Q_y_pp   = 1e-3*diag([1 1]); % d_kpn
param_weight.(MPC).Q_y0_pp  = 0*1e-3*diag([1 1]);  % D_0
param_weight.(MPC).Q_y1     = 1e3*diag([1 1]);  % D_1
param_weight.(MPC).Q_y1_p   = 1e-3*diag([1 1]);  % D_1
param_weight.(MPC).Q_y1_pp  = 1e-2*diag([1 1]);  % D_1
param_weight.(MPC).Q_yN     = 1e5*diag([1 1]);  % D_N
param_weight.(MPC).Q_yN_p   = 1e-3*diag([1 1]);  % D_N

param_weight.(MPC).Q_q_p    = 1* 1e-5*diag([1 1]);  % c_kpn
param_weight.(MPC).Q_q_pp   = 1* 1e-5*diag([1 1]);  % c_kpn
param_weight.(MPC).Q_q0_pp  = 0*   diag([1 1]);  % C_0
param_weight.(MPC).Q_qN_p   = 1* 1e-5*diag([1 1]);  % C_N

param_weight.(MPC).x_min    = 1*[ -pi; -pi; -20; -20 ];
param_weight.(MPC).x_max    = 1*[ pi; pi; 20; 20];
param_weight.(MPC).u_min    = 1*[ -10; -10 ];
param_weight.(MPC).u_max    = 1*[ 10; 10 ];

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 2) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC2';
param_weight.(MPC).Q_y      = 1e5*diag([1 1]);  % d_kpn
param_weight.(MPC).Q_y_p    = 1e-5*diag([1 1]); % d_kpn
param_weight.(MPC).Q_y_pp   = 1e-3*diag([1 1]); % d_kpn
param_weight.(MPC).Q_y0_pp  = 0*1e-3*diag([1 1]);  % D_0
param_weight.(MPC).Q_y1     = 1e3*diag([1 1]);  % D_1
param_weight.(MPC).Q_y1_p   = 1e-3*diag([1 1]);  % D_1
param_weight.(MPC).Q_y1_pp  = 1e-2*diag([1 1]);  % D_1
param_weight.(MPC).Q_yN     = 1e5*diag([1 1]);  % D_N
param_weight.(MPC).Q_yN_p   = 1e-3*diag([1 1]);  % D_N

param_weight.(MPC).Q_q_p    = 1* 1e-5*diag([1 1]);  % c_kpn
param_weight.(MPC).Q_q_pp   = 1* 1e-5*diag([1 1]);  % c_kpn
param_weight.(MPC).Q_q0_pp  = 0*   diag([1 1]);  % C_0
param_weight.(MPC).Q_qN_p   = 1* 1e-5*diag([1 1]);  % C_N

param_weight.(MPC).x_min    = 1*[ -pi; -pi; -20; -20 ];
param_weight.(MPC).x_max    = 1*[ pi; pi; 20; 20];
param_weight.(MPC).u_min    = 1*[ -10; -10 ];
param_weight.(MPC).u_max    = 1*[ 10; 10 ];

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 4) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC3';
param_weight.(MPC).Q_y        = 1*diag([1 1]);  % d_kpn
param_weight.(MPC).Q_y_p_ref  = 1e3*diag([1 1]);
param_weight.(MPC).Q_y_ref    = 1e5*diag([1 1]);

param_weight.(MPC).epsilon    = 0.1;

param_weight.(MPC).x_min    = inf*[ -pi; -pi; -20; -20 ];
param_weight.(MPC).x_max    = inf*[ pi; pi; 20; 20];
param_weight.(MPC).u_min    = inf*[ -10; -10 ];
param_weight.(MPC).u_max    = inf*[ 10; 10 ];

%%%%%%%%%%%%%%%%%%%%%%%%%% (MPC 4) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC4';
param_weight.(MPC).Q_y      = 1e5*diag([1 1]);  % d_kpn
param_weight.(MPC).Q_yN     = 1e10*diag([1 1]);  % D_N

param_weight.(MPC).R_u      = 1e-3*diag([1 1]);  % d_kpn
param_weight.(MPC).R_du      = 1*1e2*diag([1 1]);  % d_kpn

param_weight.(MPC).Q_q_p    = 1* 1e-2*diag([1 1]);  % c_kpn
param_weight.(MPC).Q_q_pp   = 1* 1e2*diag([1 1]);  % c_kpn
param_weight.(MPC).Q_q0_pp  = 0*   diag([1 1]);  % C_0
param_weight.(MPC).Q_qN_p   = 0* 1e-5*diag([1 1]);  % C_N

param_weight.(MPC).x_min    = inf*[ -pi; -pi; -20; -20 ];
param_weight.(MPC).x_max    = inf*[ pi; pi; 20; 20];
param_weight.(MPC).u_min    = inf*[ -10; -10 ];
param_weight.(MPC).u_max    = inf*[ 10; 10 ];

names = fieldnames(param_weight)';
param_weight_init = struct;
for name=names
    mpc_name = name{1};
    temp = merge_cell_arrays(struct2cell(param_weight.(mpc_name))');
    param_weight_init.(mpc_name) = temp{1}'';
end

%{
param_weight.(MPC).Q_y      = 1e5*diag([1 1]);  % d_kpn
param_weight.(MPC).Q_y_p    = 1e-5*diag([1 1]); % d_kpn
param_weight.(MPC).Q_y_pp   = 1e-5*diag([1 1]); % d_kpn
param_weight.(MPC).Q_y0_pp  = 1e-5*diag([1 1]); % D_0
param_weight.(MPC).Q_y1     = 1e5*diag([1 1]);  % D_1
param_weight.(MPC).Q_y1_p   = 1e-5*diag([1 1]); % D_1
param_weight.(MPC).Q_y1_pp  = 1e-5*diag([1 1]); % D_1
param_weight.(MPC).Q_yN     = 1e1*diag([1 1]);  % D_N
param_weight.(MPC).Q_yN_p   = 1e-5*diag([1 1]); % D_N

param_weight.(MPC).Q_q_p    = 0*diag([1 1]);  % c_kpn
param_weight.(MPC).Q_q_pp   = 0*diag([1 1]);  % c_kpn
param_weight.(MPC).Q_q0_pp  = 0*diag([1 1]);  % C_0
param_weight.(MPC).Q_qN_p   = 0*diag([1 1]);  % C_N

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