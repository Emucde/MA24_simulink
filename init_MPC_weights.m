%%%%%%%%%%%%%%%%%%%%%%%%%% MPC1 & MPC2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param_weight_MPC1.QQ_y    = 1e5*diag([1 1]);
param_weight_MPC1.QQ_y_p  = 1e-3*diag([1 1]);
param_weight_MPC1.QQ_y_pp = 1e-3*diag([1 1]);
param_weight_MPC1.QQ_yN   = 1e5*diag([1 1]);
param_weight_MPC1.xx_min  = inf*[ -1; -1; -1; -1 ];
param_weight_MPC1.xx_max  = inf*[ 1; 1; 1; 1];
param_weight_MPC1.uu_min  = inf*[ -10; -10 ];
param_weight_MPC1.uu_max  = inf*[ 10; 10 ];

param_weight_MPC2.QQ_y    = param_weight_MPC1.QQ_y   ;
param_weight_MPC2.QQ_y_p  = param_weight_MPC1.QQ_y_p ;
param_weight_MPC2.QQ_y_pp = param_weight_MPC1.QQ_y_pp;
param_weight_MPC2.QQ_yN   = param_weight_MPC1.QQ_yN  ;
param_weight_MPC2.xx_min  = param_weight_MPC1.xx_min ;
param_weight_MPC2.xx_max  = param_weight_MPC1.xx_max ;
param_weight_MPC2.uu_min  = param_weight_MPC1.uu_min ;
param_weight_MPC2.uu_max  = param_weight_MPC1.uu_max ;

%%%%%%%%%%%%%%%%%%%%%%%%%% MPC3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
param_weight_MPC3.QQ_y      = 1e5*diag([1 1]);
param_weight_MPC3.QQ_y_p    = 1e-3*diag([1 1]);
param_weight_MPC3.QQ_y_pp   = 1e-3*diag([1 1]);
param_weight_MPC3.QQ_yN     = 1e5*diag([1 1]);
param_weight_MPC3.xx_min    = [ -inf; -inf; -inf; -inf ];
param_weight_MPC3.xx_max    = [ inf; inf; inf; inf];
param_weight_MPC3.uu_min    = [ -inf; -inf ];
param_weight_MPC3.uu_max    = [ inf; inf ];

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