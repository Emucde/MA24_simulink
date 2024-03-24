%%%%%%%%%%%%%%%%%%%%%%%%%% MPC1 & MPC2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param_weight.MPC1.Q_y      = 1e5*diag([1 1]);  % d_kpn
param_weight.MPC1.Q_y_p    = 1e-5*diag([1 1]); % d_kpn
param_weight.MPC1.Q_y_pp   = 1e-5*diag([1 1]); % d_kpn
param_weight.MPC1.Q_y0_pp  = 1e-5*diag([1 1]);  % D_0
param_weight.MPC1.Q_y1     = 1e5*diag([1 1]);  % D_1
param_weight.MPC1.Q_y1_p   = 1e-5*diag([1 1]);  % D_1
param_weight.MPC1.Q_y1_pp  = 1e-5*diag([1 1]);  % D_1
param_weight.MPC1.Q_yN     = 1e1*diag([1 1]);  % D_N
param_weight.MPC1.Q_yN_p   = 1e-5*diag([1 1]);  % D_N
param_weight.MPC1.x_min    = inf*[ -1; -1; -1; -1 ];
param_weight.MPC1.x_max    = inf*[ 1; 1; 1; 1];
param_weight.MPC1.u_min    = inf*[ -10; -10 ];
param_weight.MPC1.u_max    = inf*[ 10; 10 ];

param_weight.MPC2.Q_y     = param_weight.MPC1.Q_y;
param_weight.MPC2.Q_y_p   = param_weight.MPC1.Q_y_p;
param_weight.MPC2.Q_y_pp  = param_weight.MPC1.Q_y_pp;
param_weight.MPC2.Q_y0_pp = param_weight.MPC1.Q_y0_pp;
param_weight.MPC2.Q_y1    = param_weight.MPC1.Q_y1;
param_weight.MPC2.Q_y1_p  = param_weight.MPC1.Q_y1_p;
param_weight.MPC2.Q_y1_pp = param_weight.MPC1.Q_y1_pp;
param_weight.MPC2.Q_yN    = param_weight.MPC1.Q_yN;
param_weight.MPC2.Q_yN_p  = param_weight.MPC1.Q_yN_p;
param_weight.MPC2.x_min   = param_weight.MPC1.x_min;
param_weight.MPC2.x_max   = param_weight.MPC1.x_max;
param_weight.MPC2.u_min   = param_weight.MPC1.u_min;
param_weight.MPC2.u_max   = param_weight.MPC1.u_max;

%%%%%%%%%%%%%%%%%%%%%%%%%% MPC3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param_weight.MPC3.Q_y      = 1e5*diag([1 1]);  % d_kpn
param_weight.MPC3.Q_y_p    = 1e-3*diag([1 1]); % d_kpn
param_weight.MPC3.Q_y_pp   = 1e-3*diag([1 1]); % d_kpn
param_weight.MPC3.Q_y0_pp  = 1e-3*diag([1 1]);  % D_0
param_weight.MPC3.Q_y1     = 1e3*diag([1 1]);  % D_1
param_weight.MPC3.Q_y1_p   = 1e-3*diag([1 1]);  % D_1
param_weight.MPC3.Q_y1_pp  = 1e-3*diag([1 1]);  % D_1
param_weight.MPC3.Q_yN     = 1e5*diag([1 1]);  % D_N
param_weight.MPC3.Q_yN_p   = 1e-3*diag([1 1]);  % D_N
param_weight.MPC3.x_min    = [ -inf; -inf; -inf; -inf ];
param_weight.MPC3.x_max    = [ inf; inf; inf; inf];
param_weight.MPC3.u_min    = [ -inf; -inf ];
param_weight.MPC3.u_max    = [ inf; inf ];

%%%%%%%%%%%%%%%%%%%%%%%%%% MPC4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param_weight.MPC4.Q_y      = 1e5*diag([1 1]);  % d_kpn
param_weight.MPC4.Q_y_p    = 1e-5*diag([1 1]); % d_kpn
param_weight.MPC4.Q_y_pp   = 1e-5*diag([1 1]); % d_kpn
param_weight.MPC4.Q_y0_pp  = 1e-1*diag([1 1]);  % D_0
param_weight.MPC4.Q_y1     = 0*1e2*diag([1 1]);  % D_1
param_weight.MPC4.Q_y1_p   = 1e-5*diag([1 1]);  % D_1
param_weight.MPC4.Q_y1_pp  = 1e-5*diag([1 1]);  % D_1
param_weight.MPC4.Q_yN     = 1e5*diag([1 1]);  % D_N
param_weight.MPC4.Q_yN_p   = 1e-5*diag([1 1]);  % D_N
param_weight.MPC4.x_min    = 20*[ -pi; -pi; -1; -1 ];
param_weight.MPC4.x_max    = 20*[ pi; pi; 1; 1];
param_weight.MPC4.u_min    = 1*[ -10; -10 ];
param_weight.MPC4.u_max    = 1*[ 10; 10 ];


%{
% Old mpc with bug
param_weight_MPC1.QQ_y    = 1e4*diag([1 1]);
param_weight_MPC1.QQ_y_p  = 1e-2*diag([1 1]);
param_weight_MPC1.QQ_y_pp = 1e-3*diag([1 1]);
param_weight_MPC1.QQ_yN   = 1e5*diag([1 1]);
param_weight_MPC1.xx_min  = [ -inf; -inf; -inf; -inf ];
param_weight_MPC1.xx_max  = [ inf; inf; inf; inf];
param_weight_MPC1.uu_min  = [ -inf; -inf ];
param_weight_MPC1.uu_max  = [ inf; inf ];
%}

%param_weight_MPC1.QQ_y    = 1e4*diag([1 1e1]);
%param_weight_MPC1.QQ_y_p  = 1e-1*diag([1 1]);
%param_weight_MPC1.QQ_y_pp = 1e-4*diag([1 1]);
%param_weight_MPC1.QQ_yN   = 1e4*diag([1 1]);
%param_weight_MPC1.xx_min  = [ -inf; -inf; -inf; -inf ];
%param_weight_MPC1.xx_max  = [ inf; inf; inf; inf];
%param_weight_MPC1.uu_min  = [ -inf; -inf ];
%param_weight_MPC1.uu_max  = [ inf; inf ];

%{
% STart in sing
param_weight_MPC1.QQ_y    = 1e4*diag([1 1]);
param_weight_MPC1.QQ_y_p  = 1e-1*diag([1 1]);
param_weight_MPC1.QQ_y_pp = 1e-3*diag([1 1]);
param_weight_MPC1.QQ_yN   = 1e4*diag([1 1]);
param_weight_MPC1.xx_min  = [ -inf; -inf; -inf; -inf ];
param_weight_MPC1.xx_max  = [ inf; inf; inf; inf];
param_weight_MPC1.uu_min  = [ -5; -5 ];
param_weight_MPC1.uu_max  = [ 5; 5 ];
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