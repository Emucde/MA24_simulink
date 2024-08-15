%% Pfade erstellen translatorisch (Polynome)
% Wenn man sorted_point_arr ändert bitte overwrite_path_t_valuesauf true
% setzen und danach wieder auf false (save mat file)
overwrite_path_t_values = false;
plotpath = false; % Plots path with post/pre-path
plot_path_orientation = false; % plot path with path orientations
plot_tang_stiffness_value = false;

% Achtung, wegen nichtlinearer Feder darf man nicht in qmin oder qmax von
% param_kuka (ganz unten) starten, da man sonst unendlich hohe Kräfte
% erhält, vgl. (3.44)

% Wenn q_0 verändert wird, muss in muss theta_star und
% xi_1 zu beginn wieder richtig initialisiert werden (passiert automatisch)

% Initial conditions, nonsingular pose
if( path_param.pathset == 1 ) % PUNKTE A POLYNOM OHNE AUFSATZ
    q_0 = [1.785, 1.384, 1.102, 1.257, -1.102, -1.171, -1.781, 0.303, -0.267]';
elseif( path_param.pathset == 2 ) % PUNKTE A SPLINE OHNE AUFSATZ
    q_0 = [1.785, 1.384, 1.102, 1.257, -1.102, -1.171, -1.781, 0.303, -0.267]';
elseif( path_param.pathset == 3 ) %ACIN
    %q_0 = [0.835, 0.879, -0.741, -1.540, 1.055, 1.368, 0.050, -1.746, -2.076]'; 
    q_0 = [1.370, 0.952, 1.891, 1.552, 2.126, 1.116, -0.254, -1.628, 2.055]';
elseif( path_param.pathset == 4 ) % WELDER
    %%q_0 = [1.494, 1.038, 0.088, -1.608, 0.248, -1.167, 2.610, -0.504, 1.969]';
    q_0 = [1.502, 1.118, 2.244, 1.373, -1.705, 1.478, 1.785, -1.683, 1.018]';
end

q_0_p = zeros(9,1);
q_0_pp = zeros(9,1);
% reduziertes System Anfangswerte
q_0_red = q_0(3:end);
q_0_p_red = q_0_p(3:end);
q_0_pp_red = q_0_pp(3:end);

g=gravitational_forces(q_0,param_robot);
theta_0 = q_0_red+param_robot.K\g(3:9);
theta_0_p = zeros(7,1);

%startpose im arbeitsraum
H_WE_0 = hom_transform_endeffector(q_0, param_robot);
xe0 = [H_WE_0(1:3,4);0;0;0];

J=geo_jacobian_endeffector(q_0,param_robot);
rank(J'*J);
sqrt(det(J*J'));
% Vorwärtskinematik nutzen, um Pose zu bestimmen

p0 = xe0(1:3);
R0 = H_WE_0(1:3,1:3);
quat0 = rotm2quat(R0);

% Position und Orientierung am Anfang
param_global.p0 = p0;
param_global.quat0 = quat0';

%% POLYNOME PUNKTE
Op = 3; %NACHTEIL: ALLE PFADE MUESSEN GLEICHE ORDNUNG HABEN

path_param.Op = repmat(Op, 1, N_path);

path_param.a_coeff = zeros(Op+1,3,N_path);
path_param.da_coeff = zeros(Op,3,N_path);
if(Op-1 < 1)
    path_param.dda_coeff = zeros(1,3,N_path);
else
    path_param.dda_coeff = zeros(Op-1,3,N_path);
end

if(Op-2 < 1)
    path_param.ddda_coeff = zeros(1,3,N_path);
else
    path_param.ddda_coeff = zeros(Op-2,3,N_path);
end

% TODO NICHT GANZ KONSISTENT
Op_out = 1; %Polynomordnung der Outofpath Geraden
path_param.a_coeff_theta_lt_0 = zeros(1,3,N_path);
path_param.da_coeff_theta_lt_0 = zeros(1,3,N_path);
path_param.a_coeff_theta_gt_1 = zeros(1,3,N_path);
path_param.da_coeff_theta_gt_1 = zeros(1,3,N_path);

sorted_point_arr = [0.8 1 2.5; 1 0.3 1; 1 1.4 1; 0.8 1.5 2.5];
xe0 = [sorted_point_arr(1,:) 0 0 0]'; %unsauber, orient. ign.
xeT = [sorted_point_arr(end,:) 0 0 0]'; %unsauber, orient. ign.

path_points = 4;
path_point_arr = zeros(path_points, 3, N_path);

%TODO
path_point_arr(:,:,1) = sorted_point_arr;
path_point_arr(:,:,2) = sorted_point_arr-repmat([0.1,0,0],path_points,1);% nur um z versetzt.
path_point_arr(:,:,3) = sorted_point_arr-repmat([0.3,0,0],path_points,1);

%% ROTATIONSMATRIZEN
%achtung det(R) = 1, rechtswendiges KOS gefordert!
R1_W_E = [1 0 0; 0 1 0; 0 0 1];
R2_W_E = [-1 0 0; 0 0 1; 0 1 0];

%%%%%%%%%% Pfade rotatorisch erstellen %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(det(R1_W_E) == -1)
    error('No right-handed coordinate system: det(R1_W_E)=-1');
elseif(det(R2_W_E) == -1)
    error('No right-handed coordinate system: det(R2_W_E)=-1');
end

RR = R1_W_E'*R2_W_E;

quat_R2 = rotation2quaternion(RR);
qq1 = quat_R2(1);
qq1_alpha = 2*acos(qq1);
qq1_r = quat_R2(2:4)/sin(qq1_alpha/2);

path_param.R_init = R1_W_E;
path_param.skew_ew = skew(qq1_r);
path_param.skew_ew_square = skew(qq1_r)^2;
path_param.alpha = qq1_alpha;

%%%%%%%%%%%%%%%%%%%%%%% Für die Pfaderstellung sollte ich extrafunktionen
%%%%%%%%%%%%%%%%%%%%%%% erstellen, damit es übersichtlicher wird.
%% SPLINE

Rx = @(al) [1 0 0; 0 cos(al) -sin(al); 0 sin(al) cos(al)];
Ry = @(al) [cos(al) 0 sin(al); 0 1 0; -sin(al) 0 cos(al)];
Rz = @(al) [cos(al) -sin(al) 0;sin(al) cos(al) 0; 0 0 1];


if( path_param.pathset == 1 )
    R_ee = reshape([R1_W_E R2_W_E], [3 3 2]);
    rotation = spline_rotmat(R_ee, [0 1]);
    path_param.rotation = rotation;
elseif( path_param.pathset == 2 )
    Ind_deriv = [2,4]; % Lä?nge ist = l-2
    alpha = [1 0.3 0.5 1];
    
    % p ... spline Ordnung, n ... Anzahl der Punkte (ohne Q_0)
    p = 4;
    
    l = length(alpha)-1;
    TT = zeros(l+1, 3, N_path);
    pointset = {0,0,0};
    for i=1:N_path
        % Use custom tang vek
        QQ = [path_point_arr(1,:,i);...
          mean(path_point_arr(1:2,:,i));...
          path_point_arr(2,:,i);...
          mean(path_point_arr(2:3,:,i));...
          path_point_arr(3,:,i);...
          mean(path_point_arr(3:4,:,i));...
          path_point_arr(4,:,i)];
    
        pointset{i} = QQ;
    
        T0 = QQ(2,:)-QQ(1,:);
        Tk = [(Rx(-pi/2)*(mean(sorted_point_arr([2 4],:))-QQ(3,:))')';QQ(6,:)-QQ(4,:)];
        Tn = QQ(end,:)-QQ(end-1,:);
        TT(:,:,i) = [T0; Tk; Tn];
    end
    bspline1 = bsplineCurveFitting(pointset{1}, TT(:,:,1), Ind_deriv, alpha, p);
    bspline2 = bsplineCurveFitting(pointset{2}, TT(:,:,2), Ind_deriv, alpha, p);
    bspline3 = bsplineCurveFitting(pointset{3}, TT(:,:,3), Ind_deriv, alpha, p);
    
    bspline_arr = [bspline1, bspline2, bspline3];
    R_ee = reshape([R1_W_E R2_W_E], [3 3 2]);
    rotation = spline_rotmat(R_ee, [0 1]);
    path_param.rotation = rotation;
elseif( path_param.pathset == 3 )
    bspline_arr = acin_text_spline_test([0.3 0.7 0.5], 1/4, 1/2);
    R_ee = reshape([R1_W_E R2_W_E], [3 3 2]);
    rotation = spline_rotmat(R_ee, [0 1]);
    path_param.rotation = rotation;
elseif( path_param.pathset == 4 )
    bspline_arr = [];
    xx_vec = [1, 1.2, 1.4];
    for i=1:N_path
        xx = xx_vec(i);
        QQ = [  xx 0.5 0.5;...
                xx 0.75 0.5;...
                xx 1 0.5;...
                xx 1 0.75;...
                xx 1 1;...
                xx 0.75 1;...
                xx 0.5 1];
    
        QQ(:,2) = QQ(:,2) + 0.5;
        QQ(:,3) = QQ(:,3) + 0.5;
    
        Ind_deriv = [2, 3, 4]; % Lä?nge ist = l-2
        alpha = [1 0.2 1.5 0.2 1];
        
        % p ... spline Ordnung, n ... Anzahl der Punkte (ohne Q_0)
        p = 4;
        
        T0 = [0 1 0];
        Tk = [QQ(4,:)-QQ(2,:); QQ(5,:)-QQ(3,:); QQ(6,:)-QQ(4,:)];
        Tn = [0 -1 0];
        TT = [T0; Tk; Tn];
    
        bspline1 = bsplineCurveFitting(QQ, TT, Ind_deriv, alpha, p);
        bspline_arr = [bspline_arr, bspline1];
    end

    %bspline_arr = bspline1;

    N_points = size(QQ,1);
    R_ee = zeros(3,3,6);
    R_ee(:,:,1) = Rx(pi/2)*Rz(-pi/4)*Ry(pi/2)*eye(3);
    R_ee(:,:,2) = Rx(pi/2)*Rz(-pi/4)*Ry(pi/2)*eye(3);
    R_ee(:,:,3) = Rx(pi)*Rz(-pi/4)*Ry(pi/2)*eye(3);
    R_ee(:,:,4) = Rx(pi)*Rz(-pi/4)*Ry(pi/2)*eye(3);
    R_ee(:,:,5) = Rx(3*pi/2)*Rz(-pi/4)*Ry(pi/2)*eye(3);
    R_ee(:,:,6) = Rx(3*pi/2)*Rz(-pi/4)*Ry(pi/2)*eye(3);
    
    %rotation = spline_rotmat(R_ee(:,:,1:3), bspline1.parameters([1 2 4]));
    theta_vals = bspline1.parameters(1:2:end);
    pdist = 0.05;
    theta_vals = [theta_vals(1) theta_vals(2)-pdist theta_vals(2)+pdist theta_vals(3)-pdist theta_vals(3)+pdist theta_vals(4)];
    rotation = spline_rotmat(R_ee, theta_vals);
    path_param.rotation = rotation;

    %plot_spline(bspline1, QQ, TT, alpha, Ind_deriv, 0:0.01:1, 0.1);
end

if(path_param.pathset > 1)
    % array von structs müssen identisch lang sein. Daher merge:
    path_param.bspline = combine_bsplines(bspline_arr);
else
    load('q0_prev.mat');
    path_param.bspline = bspline_old;
end


%%
load('q0_prev.mat'); % load q_0_old and pathset_old
if(any(q_0_old ~= q_0) || pathset_old ~= path_param.pathset || ...
        param_global.N_path ~= N_path_old || ...
        size(path_param.bspline.control_points,1) ~= size(bspline_old.control_points,1) || ...
        any(path_param.bspline.control_points ~= bspline_old.control_points, 'all'))
    q0_changed = true;
    fprintf("q_0 or pathset has changed!\n")
    print_q_joint_angle('q_0_old:', q_0_old, 3);
    print_q_joint_angle('q_0_new:', q_0, 3);
    fprintf("pathset_old = %d, pathset_new = %d\n", pathset_old, path_param.pathset);
    fprintf('\nUpdate path_param ...\n')
    q_0_old = q_0;
    pathset_old = path_param.pathset;
    bspline_old = path_param.bspline;
    N_path_old = N_path;
    save('q0_prev.mat', 'q_0_old', 'pathset_old', 'bspline_old', 'N_path_old')
else
    q0_changed = false;
end

%%%%%%%%%% POLYNOME:
%%%%%%%%%% Pfade translatorisch erstellen %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
filename = 'path_values.mat';
if(~exist(filename, 'file') || overwrite_path_t_values || q0_changed)
    for i=1:1:N_path
        [a_coeff, da_coeff, dda_coeff, ddda_coeff] = fit_polynomial_get_coeff(path_point_arr(:,:,i), Op);
    
        % set polycoeffs (necessary for functions ... todo)
        path_param.a_coeff(:,:,i)    = a_coeff;
        path_param.da_coeff(:,:,i)   = da_coeff;
        path_param.dda_coeff(:,:,i)  = dda_coeff;
        path_param.ddda_coeff(:,:,i) = ddda_coeff;
    
        % Check path feasibility:
        % TODO: mu_t_i check: doe path overlap, sum(mu_i) = 1??
        N_points = 50;
        theta0 = 0;
        theta1 = 1;
        theta_val = linspace(theta0, theta1, N_points);
    
        q_0_init = q_0;
        %q_0_init = (ub + lb)/2;
    
        [sum_norm_err, q_solution, x_pos_solution, pos_err_list] = check_path_feasibility(2, theta_val, q_0_init, i, param_robot, path_param);
        fprintf('Path %d: pos_err_sum = %.3g\n', i, sum_norm_err);

        % check regularity
        theta_arr = 0:0.01:1;
        if(path_param.pathset ~= 1)
            [~, dsigma_fun] = eval_path_t_vecfun(path_param, i);
            [val, ind] = min(vecnorm(dsigma_fun(theta_arr)',2));
            fprintf('Path %d: Path regularity check: min(||dC_fun(theta)||) = %.3g at theta = %.2f\n', i, val, theta_arr(ind));
        end
    end

    % Calulate length of paths
    N_samp = 5000;
    xi_1 = zeros(N_path, N_samp);
    xi_1_min = zeros(1, N_path);
    xi_1_max = zeros(1, N_path);
    

    %tic
    for i = 1:1:N_path
        xi_1(i, :) = compute_xi_1_lookup_table(i, N_samp, 0, 1, path_param);
        xi_1_min(i) = xi_1(i, 1);
        xi_1_max(i) = xi_1(i, end);
    end
    %toc

    % calculate prepath and postpath polynoms
    % p(theta)          = a0 + theta*a1 + theta^2*a2 + ... + theta^Op * a_{Op}
    % d/dtheta p(theta) =            a1 + 2*theta*a2 + ... + Op*theta^(Op-1) * a_{Op}
    % p(0) = a0 = a_coeff(1,:)
    % p(1) = a0 + a1 + 2*a2 + ... + Op*a_{Op} = sum(a_coeff)
    % d/dtheta p(0) = a1 = da_coeff(1,:)
    % d/dtheta p(1) = a1 + 2*a2 + ... + Op*a_{Op} = sum(da_coeff)
    %alte Version
    %a_coeff_theta_lt_0(:,:,1) = a_coeff(1,:);
    %da_coeff_theta_lt_0(:,:,1) = xi_1_max(1) * da_coeff(1,:)/norm(da_coeff(1,:));
    %a_coeff_theta_gt_1(:,:,1) = sum(a_coeff);
    %da_coeff_theta_gt_1(:,:,1) = xi_1_max(1) * sum(da_coeff,1)/norm(sum(da_coeff,1));

    a_coeff_theta_lt_0  = path_param.a_coeff_theta_lt_0; %=zeros(1,3,N_path)
    da_coeff_theta_lt_0 = path_param.da_coeff_theta_lt_0; %=zeros(1,3,N_path)
    a_coeff_theta_gt_1  = path_param.a_coeff_theta_gt_1; %=zeros(1,3,N_path)
    da_coeff_theta_gt_1 = path_param.da_coeff_theta_gt_1; %=zeros(1,3,N_path)

    for i = 1:1:N_path
        [sigma_t_val_0, dsigma_t_val_0, ~, ~] = eval_path_t(0, i, path_param);
        [sigma_t_val_1, dsigma_t_val_1, ~, ~] = eval_path_t(1, i, path_param);
    
        % Diese sinnlosen Variablen brauche ich nur fürs speichern
        a_coeff_theta_lt_0(:,:,i) = sigma_t_val_0;
        da_coeff_theta_lt_0(:,:,i) = dsigma_t_val_0;
        a_coeff_theta_gt_1(:,:,i) = sigma_t_val_1;
        da_coeff_theta_gt_1(:,:,i) = dsigma_t_val_1;
    
        path_param.a_coeff_theta_lt_0(:,:,i)  = sigma_t_val_0;
        path_param.da_coeff_theta_lt_0(:,:,i) = dsigma_t_val_0;
        path_param.a_coeff_theta_gt_1(:,:,i)  = sigma_t_val_1;
        path_param.da_coeff_theta_gt_1(:,:,i) = dsigma_t_val_1;
    end

    % Berechnen der xi_1 bei von startposition
    yt = H_WE_0(1:3,4); % current endeffector position
    epsilon = path_param.epsilon;
    max_steps = 80e4;

    % TODO: Optional: Besser mit Grid ausführen und lösen
    theta_star_down = zeros(1,N_path);
    theta_star_up = zeros(1,N_path);
    theta_star = zeros(1,N_path);
    for i = 1:N_path
        % as long as derivative is not small enough no minimum is found
        theta_star_init = 0;
        fprintf("Path %d: (1x Warning is ok)\n", i);
        theta_star_down(i) = find_theta_star(yt, theta_star_init, i, path_param, epsilon, max_steps);
        theta_star_init = 1;
        theta_star_up(i) = find_theta_star(yt, theta_star_init, i, path_param, epsilon, max_steps);
        if( norm(eval_path_t(theta_star_down(i), i, path_param) - yt) < norm(eval_path_t(theta_star_up(i), i, path_param) - yt))
            theta_star(i) = theta_star_down(i);
        else
            theta_star(i) = theta_star_up(i); 
        end
    end

    % Berechnen der xi_1(theta_star_0) Werte in erster Iteration
    xi_1_start_vals = zeros(1, N_path);
    %tic
    for i = 1:N_path
        if(theta_star(i) < 0)
            xi_1_vals = -compute_xi_1_lookup_table(i, N_samp, theta_star(i), 0, path_param);
        else
            xi_1_vals = compute_xi_1_lookup_table(i, N_samp, 0, theta_star(i), path_param);
        end
        xi_1_start_vals(i) = xi_1_vals(end);
    end
    %toc

    %TODO: bissl inkonsistent, damit a_coeff alle N_path coeff beinhaelt:
    a_coeff    = path_param.a_coeff;
    da_coeff   = path_param.da_coeff;
    dda_coeff  = path_param.dda_coeff;
    ddda_coeff = path_param.ddda_coeff;
    save(filename, 'xi_1', 'xi_1_min', 'xi_1_max', 'a_coeff', 'da_coeff', 'dda_coeff', 'ddda_coeff', 'a_coeff_theta_lt_0', 'da_coeff_theta_lt_0', 'a_coeff_theta_gt_1', 'da_coeff_theta_gt_1', 'theta_star', 'xi_1_start_vals');
else
    load(filename) % load xi_1
    % set polycoeffs
    path_param.a_coeff    = a_coeff;
    path_param.da_coeff   = da_coeff;
    path_param.dda_coeff  = dda_coeff;
    path_param.ddda_coeff = ddda_coeff;
    path_param.a_coeff_theta_lt_0  = a_coeff_theta_lt_0;
    path_param.da_coeff_theta_lt_0 = da_coeff_theta_lt_0;
    path_param.a_coeff_theta_gt_1  = a_coeff_theta_gt_1;
    path_param.da_coeff_theta_gt_1 = da_coeff_theta_gt_1;
    N_samp = length(xi_1);
end

% TODO: xi_sampled sollte nicht tunable sein! (in par oder extra parameter)
path_param.theta_star_start_vals = theta_star;
path_param.xi_1_start_vals = xi_1_start_vals;

path_param.xi_1_N_samp = N_samp;
path_param.xi_1_data = xi_1; % zugriff mit path_param.xi_1_data(i,1+round(theta_star*(N_samp-1)))

path_param.xi_1_min = xi_1_min;
path_param.xi_1_max = xi_1_max;

path_param.theta_min = zeros(1, N_path);
path_param.theta_max = ones(1, N_path);

if(plotpath)
    N_points = 100;
    postpath_delta = 0.1;
    theta_left = linspace(-postpath_delta,0,N_points);
    theta_mid = linspace(0,1,N_points);
    theta_right = linspace(1,1+postpath_delta,N_points);
    sigma_vals_left = zeros(N_points, 3);
    sigma_vals_right = zeros(N_points, 3);

    for i = 1 : N_path
        [sigma_fun, ~] = eval_path_t_vecfun(path_param, i);
        sigma_vals_mid = sigma_fun(theta_mid);
        sigma_vals_left = sigma_fun(theta_left);
        sigma_vals_right = sigma_fun(theta_right);
        %plot3([sorted_point_arr(:,1)], [sorted_point_arr(:,2)], [sorted_point_arr(:,3)], 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'blue')
        hold on;
        plot3(sigma_vals_left(:,1), sigma_vals_left(:,2), sigma_vals_left(:,3), 'LineWidth', 1, 'Color', 'blue');
        plot3(sigma_vals_mid(:,1), sigma_vals_mid(:,2), sigma_vals_mid(:,3), 'LineWidth', 1, 'Color', 'green');
        plot3(sigma_vals_right(:,1), sigma_vals_right(:,2), sigma_vals_right(:,3), 'LineWidth', 1, 'Color', 'red');
    end
end
%%

if(plot_path_orientation)
    R_fun = @(theta) eye(3) + sin(qq1_alpha*theta)*skew(qq1_r) + (1-cos(qq1_alpha*theta))*skew(qq1_r)^2;
    dR_fun = @(theta) skew(qq1_r)*R_fun(theta);
    dR_fun_2 = @(theta) cos(qq1_alpha*theta)*skew(qq1_r) + sin(qq1_alpha*theta)*skew(qq1_r)^2;
    % unskew(dR_fun(1/2)*R_fun(1/2)')

    figure(2)
    c_size=13;
    set(0,  'DefaultAxesFontWeight', 'normal', ...
            'DefaultAxesFontSize', c_size, ...
            'DefaultUicontrolFontName', 'Computer Modern', ...
            'DefaultUitableFontName', 'Computer Modern', ...
            'DefaultAxesFontName', 'Computer Modern', ...
            'DefaultTextFontName', 'Computer Modern', ...
            'DefaultUipanelFontName', 'Computer Modern', ...
            'DefaultTextInterpreter', 'latex', ...
            'DefaultAxesFontAngle', 'normal', ... % Not sure the difference here
            'DefaultAxesFontWeight', 'normal', ... % Not sure the difference here
            'DefaultAxesTitleFontWeight', 'normal', ...
            'DefaultAxesTitleFontSizeMultiplier', 1) ;
    set(groot,'defaultLegendInterpreter','latex');
    set(groot,'defaulttextinterpreter','latex');
    set(groot,'defaultLegendInterpreter','latex');
    set(gca,'TickLabelInterpreter','latex')

    title('path with orientations')
    %hold off
    N_points = 20;
    kos_ax_len = 0.15;
    theta_arr = linspace(0,1,N_points);
    for path_nr = 1:1:N_path
        for i = 1:N_points
            ori = eval_path_r(theta_arr(i), path_nr, path_param);
            e1 = kos_ax_len*ori(1:3,1);
            e2 = kos_ax_len*ori(1:3,2);
            e3 = kos_ax_len*ori(1:3,3);

            x = eval_path_t(theta_arr(i), path_nr, path_param);

            scale = 0.5;
            headsize = 2;
            quiver3(x(1), x(2), x(3), e1(1), e1(2), e1(3), 'r', 'LineWidth',1, 'AutoScale','on', 'AutoScaleFactor',scale, 'MaxHeadSize', headsize);
            hold on
            quiver3(x(1), x(2), x(3), e2(1), e2(2), e2(3), 'g', 'LineWidth',1, 'AutoScale','on', 'AutoScaleFactor',scale, 'MaxHeadSize', headsize);
            quiver3(x(1), x(2), x(3), e3(1), e3(2), e3(3), 'b', 'LineWidth',1, 'AutoScale','on', 'AutoScaleFactor',scale, 'MaxHeadSize', headsize);
        end
        

        [sigma_fun, ~] = eval_path_t_vecfun(path_param, path_nr);
        sigma_t_example = sigma_fun(theta_arr);

        plot3(sigma_t_example(:,1), sigma_t_example(:,2), sigma_t_example(:,3), 'k');
    end

    hold off
    grid on;
    grid minor;
    axis equal
    %xlim([0.4, 1.1]);
    %ylim([0,2]);
    %zlim([0.8,2.6]);
    %view(90.0713, -63.4722)
    xlabel('$x$ (m)');
    ylabel('$y$ (m)');
    zlabel('$z$ (m)');
    set(gca,'FontName','Computer Modern');
    set(gca,'TickLabelInterpreter','latex')
    title(['$\mbox{\boldmath $\sigma$}_{1}(0), \mbox{\boldmath $\sigma$}_{1}(1),' ...
        ' \mbox{\boldmath $\sigma$}_{2}(0), \mbox{\boldmath $\sigma$}_{2}(1),' ...
        ' \mbox{\boldmath $\sigma$}_{3}(0), \mbox{\boldmath $\sigma$}_{3}(1)$' ...
        '\raisebox{.6pt}{\textcircled{\raisebox{-.9pt} {1}}}' ...
        '\raisebox{.6pt}{\textcircled{\raisebox{-.9pt} {2}}}' ...
        '\raisebox{.6pt}{\textcircled{\raisebox{-.9pt} {3}}}' ...
        '\raisebox{.6pt}{\textcircled{\raisebox{-.9pt} {4}}}' ...
        '\raisebox{.6pt}{\textcircled{\raisebox{-.9pt} {5}}}' ...
        '\raisebox{.6pt}{\textcircled{\raisebox{-.9pt} {6}}}'], 'Interpreter','latex')
    % export svg from plot3d is buggy: Therefore:
    % copy in clipboard to paste for inkscape:
    % copygraphics(gcf, 'ContentType', 'vector');
    xlim([0, 1.5]);
    ylim([0,2]);
    zlim([0,2.5]);
end

if (plot_tang_stiffness_value)
    k_pp = @(theta, ka, k0, theta_a) ka * (theta < -theta_a) + ...
                                     (k0 - (ka - k0)/theta_a * theta) .* (-theta_a <= theta) .* (theta < 0) + ...
                                     k0 * (0 <= theta) .* (theta <= 1) + ...
                                     (k0 + (ka - k0)/theta_a * (theta-1)) .* (1<theta) .* (theta < 1+theta_a) + ...
                                     ka * (1+theta_a <= theta);
    
    plot(-1:0.01:2, k_pp(-1:0.01:2, 1, 0.1, 0.5))
end