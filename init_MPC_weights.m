%{
%% old
QQ = (diag([1e3 1e3])); % x y weight
RR_u = (diag([1e-12 1e-12])); % u absolut
RR_du = (diag([1e-9 1e-9])); % un-un-1
RR_dx = (diag([1e-6 1e-6 1e-6 1e-6]));
RR_dx_km1 = (diag([0 0 0 0]));

xx_min = [ 0; -pi; -10; -10 ];
xx_max = [ pi; pi; 10; 10];
uu_min = 0.1*[ -10; -10 ];
uu_max = 0.1*[ 10; 10 ];
%}

%{
% default
QQ = (diag([1e5 1e5])); % x y weight
RR_u = (diag([1e-6 1e-6])); % u absolut
RR_du = (diag([0 0])); % un-un-1
RR_dx = (diag([0 0 0 0]));
RR_dx_km1 = (diag([0 0 0 0]));
%}

%{
% best for not start in sing
QQ = (diag([1e5 1e5])); % x y weight
RR_u = (diag([1e-6 1e-6])); % u absolut
RR_du = (diag([0 0])); % un-un-1
RR_dx = (diag([0 0 1e-6 1e-6]));
RR_dx_km1 = (diag([0 0 0 0]));
%}

%QQ = (diag([1e5 1e5])); % x y weight
%RR_u = (diag([1e-4 1e-4])); % u absolut
%RR_du = (diag([0 0])); % un-un-1
%RR_dx = (diag([1e-2 1e-2 1e0 1e0]));
%RR_dx_km1 = (diag([0 0 0 0]));
%%}
%
%xx_min = [ -inf; -inf; -inf; -inf ];
%xx_max = [ inf; inf; inf; inf];
%uu_min = [ -inf; -inf ];
%uu_max = [ inf; inf ];

%%%%%%%%%%%%%%%%%%%%%%%%%% MPC1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% works for start in singularity (...)
param_weight_MPC1_qrqp_opti.QQ        = diag([1e5 1e5]); % x y weight
param_weight_MPC1_qrqp_opti.RR_u      = diag([1e-3 1e-3]); % u abso
param_weight_MPC1_qrqp_opti.RR_du     = diag([0 0]); % un-un-1
param_weight_MPC1_qrqp_opti.RR_dx     = 0*diag([1e-6 1e-6 1e-5 1e-5]);
param_weight_MPC1_qrqp_opti.RR_dx_km1 = diag([0 0 0 0]);
param_weight_MPC1_qrqp_opti.xx_min    = [ -inf; -inf; -inf; -inf ];
param_weight_MPC1_qrqp_opti.xx_max    = [ inf; inf; inf; inf];
param_weight_MPC1_qrqp_opti.uu_min    = [ -inf; -inf ];
param_weight_MPC1_qrqp_opti.uu_max    = [ inf; inf ];

%%%%%%%%%%%%%%%%%%%%%%%%%% MPC2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param_weight_MPC2_qrqp_opti.QQ        = diag([1e5 1e5]); % x y weight
param_weight_MPC2_qrqp_opti.RR_u      = diag([1e-3 1e-3]); % u abso
param_weight_MPC2_qrqp_opti.RR_du     = diag([0 0]); % un-un-1
param_weight_MPC2_qrqp_opti.RR_dx     = diag([1e-2 1e-2 1e-3 1e-3]);
param_weight_MPC2_qrqp_opti.RR_dx_km1 = diag([0 0 0 0]);
param_weight_MPC2_qrqp_opti.xx_min    = [ -inf; -inf; -inf; -inf ];
param_weight_MPC2_qrqp_opti.xx_max    = [ inf; inf; inf; inf];
param_weight_MPC2_qrqp_opti.uu_min    = [ -inf; -inf ];
param_weight_MPC2_qrqp_opti.uu_max    = [ inf; inf ];

%%%%%%%%%%%%%%%%%%%%%%%%%% MPC3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param_weight_MPC3_ipopt_nlpsol.QQ        = diag([1e5 1e5]); % x y weight
param_weight_MPC3_ipopt_nlpsol.RR_u      = diag([1e-3 1e-3]); % u abso
param_weight_MPC3_ipopt_nlpsol.RR_du     = diag([0 0]); % un-un-1
param_weight_MPC3_ipopt_nlpsol.RR_dx     = 0*diag([1e-6 1e-6 1e-5 1e-5]);
param_weight_MPC3_ipopt_nlpsol.RR_dx_km1 = diag([0 0 0 0]);
param_weight_MPC3_ipopt_nlpsol.xx_min    = [ -inf; -inf; -inf; -inf ];
param_weight_MPC3_ipopt_nlpsol.xx_max    = [ inf; inf; inf; inf];
param_weight_MPC3_ipopt_nlpsol.uu_min    = [ -inf; -inf ];
param_weight_MPC3_ipopt_nlpsol.uu_max    = [ inf; inf ];

%%%%%%%%%%%%%%%%%%%%%%%%%% MPC4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param_weight_MPC4_ipopt_nlpsol.QQ        = diag([1e5 1e5]); % x y weight
param_weight_MPC4_ipopt_nlpsol.RR_u      = 0*diag([1e-5 1e-5]); % u abso
param_weight_MPC4_ipopt_nlpsol.RR_du     = diag([0 0]); % un-un-1
param_weight_MPC4_ipopt_nlpsol.RR_dx     = 0*diag([1e-2 1e-2 1e-3 1e-3]);
param_weight_MPC4_ipopt_nlpsol.RR_dx_km1 = diag([0 0 0 0]);
param_weight_MPC4_ipopt_nlpsol.xx_min    = [ -inf; -inf; -inf; -inf ];
param_weight_MPC4_ipopt_nlpsol.xx_max    = [ inf; inf; inf; inf];
param_weight_MPC4_ipopt_nlpsol.uu_min    = [ -inf; -inf ];
param_weight_MPC4_ipopt_nlpsol.uu_max    = [ inf; inf ];

%%%%%%%%%%%%%%%%%%%%%%%%%% MPC5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param_weight_MPC5_qrqp_nlpsol.QQ        = diag([1e5 1e5]); % x y weight
param_weight_MPC5_qrqp_nlpsol.RR_u      = diag([1e-3 1e-3]); % u abso
param_weight_MPC5_qrqp_nlpsol.RR_du     = diag([0 0]); % un-un-1
param_weight_MPC5_qrqp_nlpsol.RR_dx     = diag([1e-2 1e-2 1e-3 1e-3]);
param_weight_MPC5_qrqp_nlpsol.RR_dx_km1 = diag([0 0 0 0]);
param_weight_MPC5_qrqp_nlpsol.xx_min    = [ -inf; -inf; -inf; -inf ];
param_weight_MPC5_qrqp_nlpsol.xx_max    = [ inf; inf; inf; inf];
param_weight_MPC5_qrqp_nlpsol.uu_min    = [ -inf; -inf ];
param_weight_MPC5_qrqp_nlpsol.uu_max    = [ inf; inf ];

%%%%%%%%%%%%%%%%%%%%%%%%%% MPC6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param_weight_MPC6_qrqp_nlpsol.yN_ineq_eps = 1e-8;%0.15; % >= 0 (if 0 then hard constrained)
param_weight_MPC6_qrqp_nlpsol.QQ          = diag([1e5 1e5]); % x y weight
param_weight_MPC6_qrqp_nlpsol.RR_u        = diag([1e-5 1e-5]); % u abso
param_weight_MPC6_qrqp_nlpsol.RR_du       = diag([0 0]); % un-un-1
param_weight_MPC6_qrqp_nlpsol.RR_dx       = 0*diag([1e-2 1e-2 1e-3 1e-3]);
param_weight_MPC6_qrqp_nlpsol.RR_dx_km1   = diag([0 0 0 0]);
param_weight_MPC6_qrqp_nlpsol.xx_min      = [ -inf; -inf; -inf; -inf ];
param_weight_MPC6_qrqp_nlpsol.xx_max      = [ inf; inf; inf; inf];
param_weight_MPC6_qrqp_nlpsol.uu_min      = [ -inf; -inf ];
param_weight_MPC6_qrqp_nlpsol.uu_max      = [ inf; inf ];

%%%%%%%%%%%%%%%%%%%%%%%%%% MPC7 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param_weight_MPC7_qrqp_nlpsol.yN_ineq_eps = -1;%0.2e-4; % >= 0 (if 0 then hard constrained)
param_weight_MPC7_qrqp_nlpsol.QQ_yN_soft = diag([1e5 1e5]);
param_weight_MPC7_qrqp_nlpsol.QQ         = diag([1e3 1e3]); % x y weight
param_weight_MPC7_qrqp_nlpsol.RR_u       = diag([1e-3 1e-3]); % u abso
param_weight_MPC7_qrqp_nlpsol.RR_du      = diag([0 0]); % un-un-1
param_weight_MPC7_qrqp_nlpsol.RR_dx      = diag([1e-2 1e-2 1e-3 1e-3]);
param_weight_MPC7_qrqp_nlpsol.RR_dx_km1  = diag([0 0 0 0]);
param_weight_MPC7_qrqp_nlpsol.xx_min     = [ -inf; -inf; -inf; -inf ];
param_weight_MPC7_qrqp_nlpsol.xx_max     = [ inf; inf; inf; inf];
param_weight_MPC7_qrqp_nlpsol.uu_min     = [ -inf; -inf ];
param_weight_MPC7_qrqp_nlpsol.uu_max     = [ inf; inf ];


%{
% singularity robust
diag([1e5 1e5]); % x y weight
diag([1e-4 1e-4]); % u abso
diag([0 0]); % un-un-1
diag([1e-2 1e-2 1e0 1e0]);
diag([0 0 0 0]);
%}

%{
QQ = DM(diag([1e6 1e6])); % x y weight
RR_u = DM(diag([1e-6 1e-6])); % u absolut
RR_du = DM(diag([0 0])); % un-un-1
RR_dx = DM(diag([0 0 0 0]));
RR_dx_km1 = DM(diag([0 0 0 0]));
%}

%{
% good for N=1
QQ = DM(diag([1e3 1e3])); % x y weight
RR_u = DM(diag([1e-12 1e-12])); % u absolut
RR_du = DM(diag([1e-9 1e-9])); % un-un-1
RR_dx = DM(diag([1e-6 1e-6 1e-5 0.5e-3]));
RR_dx_km1 = DM(diag([0 0 0 0]));
%}

%{
QQ = DM(diag([1e6 1e6])); % x y weight
RR_u = DM(diag([1e-12 1e-12])); % u absolut
RR_du = DM(diag([1e-9 1e-9])); % un-un-1
RR_dx = DM(diag([1e-6 1e-6 1e-3 1e-1]));
RR_dx_km1 = DM(diag([0 0 0 0]));
%}

%{
    QQ = (diag([1e5 1e5])); % x y weight
    RR_u = (diag([1e-4 1e-4])); % u absolut
    RR_du = (diag([0 0])); % un-un-1
    RR_dx = (diag([1e-2 1e-2 1e0 1e0]));
    RR_dx_km1 = (diag([0 0 0 0]));
%}

%{
% BEST for y as last v2, N = 5
QQ = DM(diag([1e2 1e1])); % x y weight
RR_u = DM(diag([1e-12 1e-12])); % u absolut
RR_du = DM(diag([0 0])); % un-un-1
RR_dx = DM(diag([1e-4 1e-4 1e-4 1e-4]));
RR_dx_km1 = DM(diag([0 0 0 0]));
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