function [y_d, dy_d, ddy_d] = trajectory_poly(t, y0, yT, T)
% The function plans a trajectory from y0 in R3 to yT in R3 and returns the 
% desired position and velocity on this trajectory at time t. The
% trajectory starts at t=0 with y_d(0) = y0 and dy_d(0) = 0 and reaches
% y_d(T)=yT and dy_d(T) = 0 at t=T. The trajectory moves along the
% straight interconnection between y0 and yT in R3.

% INPUTs:
% t: current time
% T: final time
% y0: 3x1 vector of the initial position
% yT: 3x1 vector of the final position
% 
% RETURN:
% y_d: 3x1 vector of the desired position at time t
% dy_d: 3x1 vector of the desired velocity at time t

%Polynom 3. Ordnung:
%y_d = (yT - y0) * (-2 * t ^ 3 / T ^ 3 + 3 * t ^ 2 / T ^ 2) + y0;
%dy_d = (yT - y0) * (-6 * t ^ 2 / T ^ 3 + 6 * t / T ^ 2);

% Polynom 5. Ordnung:
y_d   = y0 + (6   / T ^ 5 * t ^ 5 - 15  / T ^ 4 * t ^ 4 + 10 * t ^ 3 / T ^ 3) * (yT - y0);
dy_d  =      (30  / T ^ 5 * t ^ 4 - 60  / T ^ 4 * t ^ 3 + 30 * t ^ 2 / T ^ 3) * (yT - y0);
ddy_d =      (120 / T ^ 5 * t ^ 3 - 180 / T ^ 4 * t ^ 2 + 60 * t     / T ^ 3) * (yT - y0);

end