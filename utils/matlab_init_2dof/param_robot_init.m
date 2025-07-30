% This file creates the parameters for the 2 DOF robot manipulator.
% It initializes the robot's degrees of freedom (DOF), task space dimensions,
% and physical properties such as mass, inertia, and dimensions of the robot's links.
% The parameters are used in simulations and control algorithms for the robot.

%% PARAM ROBOT

param_robot.n_DOF = 2; % DOF
param_robot.m     = 2; % Dimension of the task space
param_robot.m_t   = 2; % Translational task space
param_robot.m_r   = 0; % Rotational task space

rho = 2698.9; % Dichte Aluminium kg/m^3
param_robot.g = -9.81; %m/s

% 0.5m Alu Profil 2 DOF
% BASE
param_robot.l__I = 0.0583789; %m
param_robot.mI = rho*249312.22389582483*1e-9; %kg
param_robot.inertia_matrix_sI = rho*[2.60656e+08 4.65146e+08 4.47456e+08]*1e-15; %kgm^2
param_robot.II = param_robot.inertia_matrix_sI(3); %kgm^2

% LINK 1
param_robot.l__1 = 0.230;  %m
param_robot.l__s1 = 0.115;  %m
param_robot.m1 = rho*48913.507621886434*1e-9; %kg
param_robot.inertia_matrix_s1 = rho*[3.9259e+06 2.7931e+08 2.79257e+08]*1e-15; %kgm^2
param_robot.I1 = param_robot.inertia_matrix_s1(3); %kgm^2

% LINK 2
param_robot.l__2 = 0.230+20e-3/2; %m
param_robot.l__s2 = 0.123039; %m
param_robot.m2 = rho*68850.57565993538*1e-9; %kg
param_robot.inertia_matrix_s2 = rho*[7.46749e+06 3.35834e+08 3.33465e+08]*1e-15; %kgm^2
param_robot.I2 = param_robot.inertia_matrix_s2(3); %kgm^2

%{
% 2M Full Alu 2 DOF
% BASE
param_robot.l__I = 0.0774409;
param_robot.mI = rho*1643963.4469629836*1e-9;
param_robot.inertia_matrix_sI = rho*[1.12621e+10 2.45257e+10 1.70054e+10]*1e-15;
param_robot.II = rho*1.70054e+10*1e-15;


% LINK 1
param_robot.l__1 = 0.925; %m
param_robot.l__s1 = 0.4625; %m
param_robot.m1 = rho*5534464.850183298*1e-9; %kg
param_robot.inertia_matrix_s1 = rho*[5.14973e+09 4.49458e+11 4.49419e+11]*1e-15;
param_robot.I1 = rho*4.49419e+11*1e-15; %kgm^2

% LINK 2
param_robot.l__2 = (0.9+0.1/2); %m
param_robot.l__s2 = 0.487611; %m
param_robot.m2 = rho*8564878.255227575*1e-9; %kg
param_robot.inertia_matrix_s2 = rho*[1.36306e+10 5.9489e+11 5.95224e+11]*1e-15;
param_robot.I2 = rho*5.95224e+11*1e-15; %kgm^2
%}