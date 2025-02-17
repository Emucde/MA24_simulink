single_sing_pose = readmatrix('./main_c/sorted_output_single.csv');
double_sing_pose = readmatrix('./main_c/sorted_output_doublesing_7.csv');

%plot(single_sing_pose);
%plot(double_sing_pose);

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