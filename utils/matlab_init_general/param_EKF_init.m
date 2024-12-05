% Parameter initialization for the Extended Kalman Filter (EKF) for the
% estimation of the robot state.

noise_amp = 0.1e-3;
param_EKF.Rk = diag([1/2*pi*ones(1,7)*noise_amp^2, 1/2*pi*ones(1,7)*noise_amp^2]); % cov measurement noise
param_EKF.Rk_FR3 = diag([ones(1,7)*5e-12, ones(1,7)*6.43e-7]); % cov measurement noise
param_EKF.Qk = diag([1e3*1/2*pi*ones(1,7)*noise_amp^2, 1e5*ones(1,7)]);
param_EKF.Qk_FR3 = diag([1e3*ones(1,7)*5e-12, 1e5*ones(1,7)]);
% param_EKF.Qk_FR3 = diag([1e5*ones(1,7), 1e5*ones(1,7)]);
param_EKF.P0 = diag([1e0*ones(1,7), 1e0*ones(1,7)]);

% q Rk: 4.85e-12 bei fr3

% Rk = cov(v) ... cov measurement noise
% Qk = cov(w) ... cov process disturbance noise
%
% System:
%
% d/dt [q; q_p] = [q_p + w1; M(q)^(-1) * (tau - C(q, q_p) - G(q)) + w2]
% y = [q; q_p] + v
%
% Rk is estimated via noise measurement with known robot state.
% Qk is set by trial and error. It is the process noise covariance matrix.