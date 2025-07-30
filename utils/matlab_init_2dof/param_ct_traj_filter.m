% This file creates the parameters for the trajectory filter
% for the 2DOF manipulator.
% It is used in the Simulink model to filter the desired trajectory
% inputs for the manipulator.

%% Filter für xyz:
lambda = -10;  %Eigenwert für Sollwertfilter, die Zeitkonstante ist dann
% ja tau = 1/(abs(lambda)) in Sekunden, d. h. bei schnelleren
% Trajektorien muss man die Zeitkonstante des Sollwertfilters anpassen,
% damit es den Signalen schnell genug folgen kann. Momentan auf 200 ms
% eingestellt (lambda = -5), d. h. das Delay beträgt ca. 5 tau = 1s, d.
% h. erst nach dieser Zeit kann man damit rechnen, dass der Ausgang des
% Sollwertfilters mit dessen Eingang übereinstimmt.

coeffs = poly([lambda lambda lambda lambda lambda lambda]);


a0_f = coeffs(7);
a1_f = coeffs(6);
a2_f = coeffs(5);
a3_f = coeffs(4);
a4_f = coeffs(3);
a5_f = coeffs(2);

A_f = [0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; -a0_f -a1_f -a2_f -a3_f -a4_f -a5_f];
b_f = [0; 0; 0; 0; 0; a0_f];
C_f = eye(6)*a0_f; % damit [yf ; d/dt yf; d^2/dt^2 yf] = a0^3 * xf gilt.
D_f = [0; 0; 0; 0; 0; 0];

SYS_f = ss(A_f,b_f,C_f,D_f); % kontinuierliches System für y
SYS_f_z = c2d(SYS_f, param_global.Ta); % diskretes System für y (macht eh euler
%vorwärts)

Phi_yt = (eye(6) + A_f*param_global.Ta); % euler vorwärts
Gamma_yt = b_f*param_global.Ta;  % euler vorwärts

lamda_xyz = lambda;
%% Filter für alpha
%lambda = -5;
coeffs = poly([lambda lambda lambda lambda lambda lambda]);


a0_f = coeffs(7);
a1_f = coeffs(6);
a2_f = coeffs(5);
a3_f = coeffs(4);
a4_f = coeffs(3);
a5_f = coeffs(2);

A_f = [0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; -a0_f -a1_f -a2_f -a3_f -a4_f -a5_f];
b_f = [0; 0; 0; 0; 0; a0_f];
C_f = eye(6)*a0_f; % damit [yf ; d/dt yf; d^2/dt^2 yf] = a0^3 * xf gilt.
D_f = [0; 0; 0; 0; 0; 0];

Phi_alpha = (eye(6) + A_f*param_global.Ta); % euler vorwärts
Gamma_alpha = b_f*param_global.Ta;  % euler vorwärts

param_traj_filter.A = blkdiag(A_f, A_f, A_f, A_f);
param_traj_filter.b = blkdiag(b_f, b_f, b_f, b_f);
param_traj_filter.Ta = param_global.Ta;
param_traj_filter.Phi = blkdiag(Phi_yt, Phi_yt, Phi_yt, Phi_alpha);
param_traj_filter.Gamma = blkdiag(Gamma_yt,Gamma_yt,Gamma_yt,Gamma_alpha);

% Brauche ich in Simulink:
selector_index1 = [1 7 13 19];
selector_index2 = selector_index1+1;
selector_index3 = selector_index1+2;

param_traj_filter.p_d_index    = selector_index1;
param_traj_filter.p_d_p_index  = selector_index2;
param_traj_filter.p_d_pp_index = selector_index3;

lamda_alpha = lambda;
%{
%% Filter für xyz:
lambda = -5;  %Eigenwert für Sollwertfilter, die Zeitkonstante ist dann
% ja tau = 1/(abs(lambda)) in Sekunden, d. h. bei schnelleren
% Trajektorien muss man die Zeitkonstante des Sollwertfilters anpassen,
% damit es den Signalen schnell genug folgen kann. Momentan auf 200 ms
% eingestellt (lambda = -5), d. h. das Delay beträgt ca. 5 tau = 1s, d.
% h. erst nach dieser Zeit kann man damit rechnen, dass der Ausgang des
% Sollwertfilters mit dessen Eingang übereinstimmt.

coeffs = poly([lambda lambda lambda]);

a0_f = coeffs(4);
a1_f = coeffs(3);
a2_f = coeffs(2);

A_f = [0 1 0; 0 0 1; -a0_f -a1_f -a2_f];
b_f = [0; 0; a0_f];
C_f = eye(3)*a0_f; % damit [yf ; d/dt yf; d^2/dt^2 yf] = a0^3 * xf gilt.
D_f = [0; 0; 0];

SYS_f = ss(A_f,b_f,C_f,D_f); % kontinuierliches System für y
SYS_f_z = c2d(SYS_f, param_global.Ta); % diskretes System für y (macht eh euler
%vorwärts)

Phi_yt = (eye(6) + A_f*param_global.Ta); % euler vorwärts
Gamma_yt = b_f*param_global.Ta;  % euler vorwärts

%% Filter für alpha
%lambda = -5;
coeffs = poly([lambda lambda lambda]);

a0_f = coeffs(4);
a1_f = coeffs(3);
a2_f = coeffs(2);

A_f = [0 1 0; 0 0 1; -a0_f -a1_f -a2_f];
b_f = [0; 0; a0_f];
C_f = eye(3)*a0_f; % damit [yf ; d/dt yf; d^2/dt^2 yf] = a0^3 * xf gilt.
D_f = [0; 0; 0];

Phi_alpha = (eye(3) + A_f*param_global.Ta); % euler vorwärts
Gamma_alpha = b_f*param_global.Ta;  % euler vorwärts

param_traj_filter.A = blkdiag(A_f, A_f, A_f, A_f);
param_traj_filter.b = blkdiag(b_f, b_f, b_f, b_f);
param_traj_filter.Ta = param_global.Ta;
param_traj_filter.Phi = blkdiag(Phi_yt, Phi_yt, Phi_yt, Phi_alpha);
param_traj_filter.Gamma = blkdiag(Gamma_yt,Gamma_yt,Gamma_yt,Gamma_alpha);

% Brauche ich in Simulink:
selector_index1 = [1 4 7 10];
selector_index2 = selector_index1+1;
selector_index3 = selector_index1+2;
%}