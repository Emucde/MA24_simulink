%run('main_matlab/parameters_7dof.m');

single_sing_pose = readmatrix('./main_c/sorted_output_single.csv');
double_sing_pose = readmatrix('./main_c/sorted_output_doublesing_7.csv');

figure(3);
plot(single_sing_pose(:, 4));
figure(4);
plot(double_sing_pose);

% 110612, 129903
q1=double_sing_pose(110612, :);
q2=double_sing_pose(129903, :);

% 40514, 40515
q3=double_sing_pose(40514, :);
q4=double_sing_pose(40515, :);

double_sing_pose_bestof = readmatrix('./main_c/singularity_solutions_double_pose.csv');
q5 = double_sing_pose_bestof(1, 1:7);
q6 = double_sing_pose_bestof(1, 8:end);

N_row = length(double_sing_pose_bestof);
H_best = zeros(N_row, 3);
for i=1:length(double_sing_pose_bestof)
    H = hom_transform_endeffector_py(double_sing_pose_bestof(i, 1:7));
    H_best(i,:) = H(1:3, 4);
end

q7 = double_sing_pose_bestof(12, 1:7);
q8 = double_sing_pose_bestof(11, 8:end);

N_row = length(single_sing_pose);
H_best_single = zeros(N_row, 3);
for i=1:length(single_sing_pose)
    H = hom_transform_endeffector_py(single_sing_pose(i, 1:7));
    H_best_single(i,:) = H(1:3, 4);
end

norms_single_sing_pose = vecnorm(single_sing_pose-param_robot.q_n', 2, 2);

% TRAJ 6 super aber kurz
q9 = single_sing_pose(48208, 1:7);
q10 = single_sing_pose(68131, 1:7);

q11 = single_sing_pose(29277, 1:7);
q12 = single_sing_pose(68098, 1:7);

% write back to file but 7 dim:
%csvwrite('./main_c/double_pose_singularities_7dim.csv', qq_all);

% calculate pose distances
% sorted_pose = zeros(size(qq_all, 1), 7);
% pose_distances = zeros(size(qq_all, 1), 1);
% sorted_pose(1, :) = qq_all(1, :);
% for i=1:size(qq_all, 1)
%     pose_i = sorted_pose(i, :);
%     pose_distances_temp = zeros(size(qq_all, 1)-i, 1);
%     cnt = 1;
%     for j=1:size(qq_all, 1)
%         pose_j = qq_all(j, :);
%         pose_distances_temp(cnt) = norm(pose_i - pose_j, 2);
%         cnt = cnt+1;
%     end
%     [pose_distances(i+1), min_idx] = min(pose_distances_temp);;
%     % remove min_idx from qq_all
%     sorted_pose(i+1, :) = qq_all(min_idx, :);
%     qq_all = qq_all([1:min_idx-1, min_idx+1:end], :);

%     if mod(i, 100) == 0
%         disp(['Iteration: ', num2str(i)]);
%         fprintf('Pose %d, pose_distances: %g\n', i, pose_distances(i));
%     end
% end

y = [1/2, -1/4, 1/4, -1/2]; % = Q_0, Q_1, ..., Q_n
z = [0 0 0 0];
x = [0 1/3 2/3 1];
QQ = [x; y; z]'; % e.g. measured data

% Use custom tang vek
T0 = [1 -1 0];
Tk = [1 0 0; 1 1 0]; % = [T1; T2]
Tn = [0 1 1];

TT = [T0; Tk; Tn];

% T1 = [1 0 0] active at Q1 = QQ(2,:), T2 = [1 1 0] active at Q2 = QQ(3,:)
Ind_deriv = [1, 2]; % index 1: T1, index 2: T2

% T0 and Tn are ignored (alpha(1) = alpha(end) = 0), T1 and T2 are scaled
% with approximated chord length 1*d. This means for this example:
% alpha(1) is active on T0 = TT(1, :),
% alpha(2) is active on T1 = TT(1+Ind_deriv(1), :)
% alpha(3) is active on T2 = TT(1+Ind_deriv(2), :)
% alpha(end) is active on Tn = TT(end, :)
alpha = [0 1 1 0];

p = 3; % spline order

PDi = bsplineCurveFitting(QQ, TT, Ind_deriv, alpha, p);
figure(5);
CC = spline_fun(0, 1, PDi);
fun = @(t) cell2mat(arrayfun(@(i) spline_fun(i, 1, PDi), t, 'UniformOutput', false));