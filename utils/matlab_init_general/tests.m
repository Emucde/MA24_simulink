% This file was only used for testing purposes.
% K_p_r = eye(3);
% z2 = [1.23;2.709;3.23;4.00000019]/norm([1.23;2.709;3.23;4.00000019]); %xe0(4:7);
% q_eps = z2(2:4);
% z4 = [0;0;0];
% m_t = 3;
% z4_p_v1 = -2 * ( z2(1)*eye(m_t) + skew(z2(2:4)) ) * K_p_r * q_eps - K_d_r*z4;
% [~, Q_q] = quat_deriv(z2, z4); % z2=q_ref, z4=omega_ref, Q_p is 4x3
% Q_eps = Q_q(2:4, :); % 3x3
% z4_p_v2 = -4 * Q_eps' * K_p_r * q_eps                             - K_d_r*z4; % without Q_eps calculatin 28% faster, with 2x slower
% z4_p_v1 - z4_p_v2

% test_val=1; % quaternionen tests
% test_val=2; % Speed test casadi python vs Maple23
% test_val=3; % Speed test casadi python vs Maple19
% test_val=4; % Speed test casadi python vs Maple23 casadi
% test_val=5; % Speed test casadi python vs Maple19 casadi
% test_val=6; % Compile casadi python & Maple23 casadi to matlab_sfun
% test_val=7; % Compile casadi python & Maple19 casadi to matlab_sfun
% test_val=8; % Speed test casadi python compiled vs Maple23 casadi compiled
% test_val=9; % Speed test casadi python compiled vs Maple19 casadi compiled

test_val=6;
compile = false;
if(test_val == 1)
    
    Q = @(q) [[-1/2*q(2), -1/2*q(3), -1/2*q(4)]; [1/2*q(1), 1/2*q(4), -1/2*q(3)]; [-1/2*q(4), 1/2*q(1), 1/2*q(2)]; [1/2*q(3), -1/2*q(2), 1/2*q(1)]];
    Q_buch = @(q) [[-1/2*q(2), -1/2*q(3), -1/2*q(4)]; [1/2*q(1), -1/2*q(4), 1/2*q(3)]; [1/2*q(4), 1/2*q(1), -1/2*q(2)]; [-1/2*q(3), 1/2*q(2), 1/2*q(1)]];
    
    omegaI = [1; 0; 0];
    RI = Rz(5*pi/180);
    qq1 = rotation2quaternion(RI);
    R1 = quaternion2rotation(qq1);
    
    % Erstellen des 3D-Plots
    figure;
    hold on;
    
    % Plot des Inertialsystems
    quiver3(0, 0, 0, 1, 0, 0, 'k', 'LineWidth', 1);
    quiver3(0, 0, 0, 0, 1, 0, 'k', 'LineWidth', 1);
    quiver3(0, 0, 0, 0, 0, 1, 'k', 'LineWidth', 1);
    
    % Plot des leicht um z verdrehten Koordinatensystems
    quiver3(0, 0, 0, R1(1,1), R1(2,1), R1(3,1), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
    quiver3(0, 0, 0, R1(1,2), R1(2,2), R1(3,2), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
    quiver3(0, 0, 0, R1(1,3), R1(2,3), R1(3,3), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
    
    % Rotation um omega = [0,0,1] liefert:
    qvel1 = Q(qq1)*omegaI; % thelenberg
    qvel2 = Q_buch(qq1)*omegaI; % buch
    
    for i = 1:5:100
        Ta = 0.1*i;
        %qq21 = (quat_mult(qq1, Ta*qvel1));
        %qq22 = (quat_mult(qq1, Ta*qvel2));
        qq21 = qq1 + Ta*qvel1;
        qq22 = qq1 + Ta*qvel2;
    
        qq21 = qq21/norm(qq21);
        qq22 = qq22/norm(qq22);
    
        R21 = quaternion2rotation(qq21);
        R22 = quaternion2rotation(qq22);
    
        vecR1 = [1; 0; 0];
        vecRI = R1*vecR1;
        vecI_1 = R21*vecR1;
        vecI_2 = R22*vecR1; % Demnach kommt es so vor, als würde die Formel aus den Buch in bezug auf das
        % Körperfeste Koordinatensystem definiert sein???
    
        disp([rotm2eul_v2(R21) rotm2eul_v2(R22) vecI_1 vecI_2]);
    
        % Plot der Vektoren
        quiver3(0, 0, 0, vecRI(1), vecRI(2), vecRI(3), 'b', 'LineWidth', 1);
        quiver3(0, 0, 0, vecI_1(1), vecI_1(2), vecI_1(3), 'r', 'LineWidth', 1);
        quiver3(0, 0, 0, vecI_2(1), vecI_2(2), vecI_2(3), 'g', 'LineWidth', 1);
    end
    
    legend('RI_x', 'RI_y', 'RI_z', 'R1_x', 'R1_y', 'R1_z', 'Vektor 1', 'Vektor 21', 'Vektor 22', 'Location', 'best');
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Plot der Vektoren und Koordinatensysteme');
    grid on;
    view(3);
    
    hold off;
    
    %%
    %{
    RR = zeros(3,3,8);
    RR_d = zeros(3,3,8);
    qq = zeros(4,8);
    qq_d = zeros(4,8);
    j=1;
    ppath = '../quaternion_flipping_problems/Problem2/';
    for i=78:85
        load([ppath, 'qdRd_t', num2str(i), '.mat']);
        RR(:,:,j) = R;
        qq(:,j) = q_e;
        RR_d(:,:,j) = R_d;
        qq_d(:,j) = q_d;
        j=j+1;
    end
    %}
    
    figure;
    hold on;
    
    % Plotte die erste Rotationsmatrix R_init
    quiver3(0, 0, 0, R_init(1,1), R_init(2,1), R_init(3,1), 'r', 'LineWidth', 2, 'DisplayName', 'R_{init}');
    quiver3(0, 0, 0, R_init(1,2), R_init(2,2), R_init(3,2), 'g', 'LineWidth', 2);
    quiver3(0, 0, 0, R_init(1,3), R_init(2,3), R_init(3,3), 'b', 'LineWidth', 2);
    
    % Plotte die zweite Rotationsmatrix R_target
    quiver3(0, 0, 0, R_target(1,1), R_target(2,1), R_target(3,1), 'm', 'LineWidth', 2, 'DisplayName', 'R_{target}');
    quiver3(0, 0, 0, R_target(1,2), R_target(2,2), R_target(3,2), 'c', 'LineWidth', 2);
    quiver3(0, 0, 0, R_target(1,3), R_target(2,3), R_target(3,3), 'y', 'LineWidth', 2);
    
    % Setze die Achsenbeschriftungen
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    % Füge eine Legende hinzu
    legend('R_init_x', 'R_init_y', 'R_init_z', 'R_target_x', 'R_target_y', 'R_target_z', 'Location', 'best');
    
    % Setze das Seitenverhältnis der Achsen auf gleich
    axis equal;
    xlim([-2,2])
    ylim([-2,2])
    zlim([-2,2])
    
    hold off;
elseif(test_val == 2) % casadi python vs Maple 23
    %rmpath('./maple/maple_generated/7_dof_system_fr3_MW19');
    %addpath('./maple/maple_generated/7_dof_system_fr3');
    qpp_fun_maple = @(q, q_p, tau, param) inertia_matrix(q, param)\(tau - coriolis_matrix(q, q_p, param)*q_p - gravitational_forces(q, param));
    qpp_fun_maple_casadi_SX = @(q, q_p, tau, param) inertia_matrix_casadi_SX(q, param)\(tau - coriolis_matrix_casadi_SX(q, q_p, param)*q_p - gravitational_forces_casadi_SX(q, param));

    % load function to workspace
    MM=inertia_matrix(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        MM=inertia_matrix(pi*rand(7,1), param_robot); MC = MM.*rand(1,1);
    end
    disp('Time for 1000 runs M23:'); disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        MM=M(pi*rand(7,1)); MC = (MC + MM.*rand(1,1))/2;
    end
    disp('Time for 1000 runs of M(pi*rand(7,1)):'); disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qp_c = pi*rand(7,1);
    q_c = pi*rand(7,1);
    CC=coriolis_matrix(q_c, qp_c, param_robot)*qp_c + gravitational_forces(q_c, param_robot);
    
    tic;
    for i=1:1:1000
        qp_c = pi*rand(7,1);
        q_c = pi*rand(7,1);
        CC=coriolis_matrix(q_c, qp_c, param_robot)*qp_c + gravitational_forces(q_c, param_robot);
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C23_rnea = C(q, qp)qp + g(q):');
    disp(['   ', num2str(toc), 's']);

    % load function to workspace
    qp_c = pi*rand(7,1);
    q_c = pi*rand(7,1);
    CC=coriolis_rnea(q_c, qp_c, param_robot);
    
    tic;
    for i=1:1:1000
        qp_c = pi*rand(7,1);
        q_c = pi*rand(7,1);
        CC=coriolis_rnea(q_c, qp_c, param_robot);
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C23_rnea_maple');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        CC=C_rnea(pi*rand(7,1), pi*rand(7,1));
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C_rnea(q, qp):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    gg=gravitational_forces(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        gg=gravitational_forces(pi*rand(7,1), param_robot);
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g23');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        gg=g(pi*rand(7,1));
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    HH=hom_transform_endeffector(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        HH=hom_transform_endeffector(pi*rand(7,1), param_robot);
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H23');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        HH=H(pi*rand(7,1));
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJ=geo_jacobian_endeffector(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJ=geo_jacobian_endeffector(pi*rand(7,1), param_robot);
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J23');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJ=J(pi*rand(7,1));
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJp = geo_jacobian_endeffector_p(pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJp=geo_jacobian_endeffector_p(pi*rand(7,1), pi*rand(7,1), param_robot);
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J23_p');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJp=J_p(pi*rand(7,1), pi*rand(7,1));
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q, qp):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qqpp = qpp_fun_maple(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun_maple(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun_maple23');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun (Backslash)');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = sys_fun_qpp(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of sys_fun_qpp (Aba)');
    disp(['   ', num2str(toc), 's']);
    
    
    return
elseif(test_val == 3) % casadi python vs Maple 23
    %rmpath('./maple/maple_generated/7_dof_system_fr3');
    %addpath('./maple/maple_generated/7_dof_system_fr3_MW19');
    qpp_fun_maple = @(q, q_p, tau, param) inertia_matrix(q, param)\(tau - coriolis_matrix(q, q_p, param)*q_p - gravitational_forces(q, param));
    qpp_fun_maple_casadi_SX = @(q, q_p, tau, param) inertia_matrix_casadi_SX(q, param)\(tau - coriolis_matrix_casadi_SX(q, q_p, param)*q_p - gravitational_forces_casadi_SX(q, param));

    % load function to workspace
    MM=inertia_matrix(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        MM=inertia_matrix(pi*rand(7,1), param_robot); MC = MM.*rand(1,1);
    end
    disp('Time for 1000 runs M19:'); disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        MM=M(pi*rand(7,1)); MC = MM.*rand(1,1);
    end
    disp('Time for 1000 runs of M(pi*rand(7,1)):'); disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qp_c = pi*rand(7,1);
    q_c = pi*rand(7,1);
    CC=coriolis_matrix(q_c, qp_c, param_robot)*qp_c + gravitational_forces(q_c, param_robot);
    
    tic;
    for i=1:1:1000
        qp_c = pi*rand(7,1);
        q_c = pi*rand(7,1);
        CC=coriolis_matrix(q_c, qp_c, param_robot)*qp_c + gravitational_forces(q_c, param_robot);
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C19_rnea');
    disp(['   ', num2str(toc), 's']);

    % load function to workspace
    qp_c = pi*rand(7,1);
    q_c = pi*rand(7,1);
    CC=coriolis_rnea(q_c, qp_c, param_robot);
    
    tic;
    for i=1:1:1000
        qp_c = pi*rand(7,1);
        q_c = pi*rand(7,1);
        CC=coriolis_rnea(q_c, qp_c, param_robot);
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C19_rnea_maple');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        CC=C_rnea(pi*rand(7,1), pi*rand(7,1));
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C_rnea(q, qp):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    gg=gravitational_forces(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        gg=gravitational_forces(pi*rand(7,1), param_robot);
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g19');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        gg=g(pi*rand(7,1));
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    HH=hom_transform_endeffector(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        HH=hom_transform_endeffector(pi*rand(7,1), param_robot);
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H19');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        HH=H(pi*rand(7,1));
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJ=geo_jacobian_endeffector(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJ=geo_jacobian_endeffector(pi*rand(7,1), param_robot);
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J19');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJ=J(pi*rand(7,1));
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJp = geo_jacobian_endeffector_p(pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJp=geo_jacobian_endeffector_p(pi*rand(7,1), pi*rand(7,1), param_robot);
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J19_p');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJp=J_p(pi*rand(7,1), pi*rand(7,1));
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q, qp):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qqpp = qpp_fun_maple(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun_maple(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun_maple19');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun (Backslash)');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = sys_fun_qpp(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of sys_fun_qpp (Aba)');
    disp(['   ', num2str(toc), 's']);
    
    
    return
elseif(test_val == 4) % casadi python vs Maple 23 casadi
    %rmpath('./maple/maple_generated/7_dof_system_fr3_MW19');
    %addpath('./maple/maple_generated/7_dof_system_fr3');
    qpp_fun_maple = @(q, q_p, tau, param) inertia_matrix(q, param)\(tau - coriolis_matrix(q, q_p, param)*q_p - gravitational_forces(q, param));
    qpp_fun_maple_casadi_SX = @(q, q_p, tau, param) inertia_matrix_casadi_SX(q, param)\(tau - coriolis_matrix_casadi_SX(q, q_p, param)*q_p - gravitational_forces_casadi_SX(q, param));

    % load function to workspace
    MM=inertia_matrix_casadi_SX(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        MM=inertia_matrix_casadi_SX(pi*rand(7,1), param_robot); MC = MM.*rand(1,1);
    end
    disp('Time for 1000 runs M23:'); disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        MM=M(pi*rand(7,1)); MC = MM.*rand(1,1);
    end
    disp('Time for 1000 runs of M(pi*rand(7,1)):'); disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qp_c = pi*rand(7,1);
    q_c = pi*rand(7,1);
    CC=coriolis_matrix_casadi_SX(q_c, qp_c, param_robot)*qp_c + gravitational_forces_casadi_SX(q_c, param_robot);
    
    tic;
    for i=1:1:1000
        qp_c = pi*rand(7,1);
        q_c = pi*rand(7,1);
        CC=coriolis_matrix_casadi_SX(q_c, qp_c, param_robot)*qp_c + gravitational_forces_casadi_SX(q_c, param_robot);
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C23_rnea');
    disp(['   ', num2str(toc), 's']);

    % load function to workspace
    qp_c = pi*rand(7,1);
    q_c = pi*rand(7,1);
    CC=coriolis_rnea_casadi_SX(q_c, qp_c, param_robot);
    
    tic;
    for i=1:1:1000
        qp_c = pi*rand(7,1);
        q_c = pi*rand(7,1);
        CC=coriolis_rnea_casadi_SX(q_c, qp_c, param_robot);
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C23_rnea_maple_SX');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        CC=C_rnea(pi*rand(7,1), pi*rand(7,1));
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C_rnea(q, qp):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    gg=gravitational_forces_casadi_SX(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        gg=gravitational_forces_casadi_SX(pi*rand(7,1), param_robot);
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g23');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        gg=g(pi*rand(7,1));
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    HH=hom_transform_endeffector_casadi_SX(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        HH=hom_transform_endeffector_casadi_SX(pi*rand(7,1), param_robot);
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H23');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        HH=H(pi*rand(7,1));
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJ=geo_jacobian_endeffector_casadi_SX(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJ=geo_jacobian_endeffector_casadi_SX(pi*rand(7,1), param_robot);
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J23');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJ=J(pi*rand(7,1));
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJp = geo_jacobian_endeffector_p_casadi_SX(pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJp=geo_jacobian_endeffector_p_casadi_SX(pi*rand(7,1), pi*rand(7,1), param_robot);
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J23_p');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJp=J_p(pi*rand(7,1), pi*rand(7,1));
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q, qp):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qqpp = qpp_fun_maple_casadi_SX(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun_maple_casadi_SX(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun_maple_casadi_SX23');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun (Backslash)');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = sys_fun_qpp(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of sys_fun_qpp (Aba)');
    disp(['   ', num2str(toc), 's']);
    
    
    return
elseif(test_val == 5) % casadi python vs Maple 19 casadi
    %rmpath('./maple/maple_generated/7_dof_system_fr3');
    %addpath('./maple/maple_generated/7_dof_system_fr3_MW19');
    qpp_fun_maple = @(q, q_p, tau, param) inertia_matrix(q, param)\(tau - coriolis_matrix(q, q_p, param)*q_p - gravitational_forces(q, param));
    qpp_fun_maple_casadi_SX = @(q, q_p, tau, param) inertia_matrix_casadi_SX(q, param)\(tau - coriolis_matrix_casadi_SX(q, q_p, param)*q_p - gravitational_forces_casadi_SX(q, param));

    % load function to workspace
    MM=inertia_matrix_casadi_SX(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        MM=inertia_matrix_casadi_SX(pi*rand(7,1), param_robot); MC = MM.*rand(1,1);
    end
    disp('Time for 1000 runs M19:'); disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        MM=M(pi*rand(7,1)); MC = MM.*rand(1,1);
    end
    disp('Time for 1000 runs of M(pi*rand(7,1)):'); disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qp_c = pi*rand(7,1);
    q_c = pi*rand(7,1);
    CC=coriolis_matrix_casadi_SX(q_c, qp_c, param_robot)*qp_c + gravitational_forces_casadi_SX(q_c, param_robot);
    
    tic;
    for i=1:1:1000
        qp_c = pi*rand(7,1);
        q_c = pi*rand(7,1);
        CC=coriolis_matrix_casadi_SX(q_c, qp_c, param_robot)*qp_c + gravitational_forces_casadi_SX(q_c, param_robot);
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C19_rnea');
    disp(['   ', num2str(toc), 's']);

    % load function to workspace
    qp_c = pi*rand(7,1);
    q_c = pi*rand(7,1);
    CC=coriolis_rnea_casadi_SX(q_c, qp_c, param_robot);
    
    tic;
    for i=1:1:1000
        qp_c = pi*rand(7,1);
        q_c = pi*rand(7,1);
        CC=coriolis_rnea_casadi_SX(q_c, qp_c, param_robot);
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C19_rnea_maple_SX');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        CC=C_rnea(pi*rand(7,1), pi*rand(7,1));
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C_rnea(q, qp):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    gg=gravitational_forces_casadi_SX(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        gg=gravitational_forces_casadi_SX(pi*rand(7,1), param_robot);
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g19');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        gg=g(pi*rand(7,1));
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    HH=hom_transform_endeffector_casadi_SX(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        HH=hom_transform_endeffector_casadi_SX(pi*rand(7,1), param_robot);
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H19');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        HH=H(pi*rand(7,1));
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJ=geo_jacobian_endeffector_casadi_SX(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJ=geo_jacobian_endeffector_casadi_SX(pi*rand(7,1), param_robot);
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J19');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJ=J(pi*rand(7,1));
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJp = geo_jacobian_endeffector_p_casadi_SX(pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJp=geo_jacobian_endeffector_p_casadi_SX(pi*rand(7,1), pi*rand(7,1), param_robot);
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J19_p');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJp=J_p(pi*rand(7,1), pi*rand(7,1));
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q, qp):');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qqpp = qpp_fun_maple_casadi_SX(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun_maple_casadi_SX(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun_maple_casadi_SX19');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun (Backslash)');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = sys_fun_qpp(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of sys_fun_qpp (Aba)');
    disp(['   ', num2str(toc), 's']);
    
    
    return
elseif(test_val == 6)
    %rmpath('./maple/maple_generated/7_dof_system_fr3_MW19');
    %addpath('./maple/maple_generated/7_dof_system_fr3');

    output_dir = './s_functions/s_functions_7dof/maple_msfun/';
    fun_dir1 = './s_functions/s_functions_7dof/';
    fun_dir2 = './s_functions/s_functions_7dof/maple_msfun/';
    compile_msfunctions(sys_fun_qpp_aba, fun_dir1, fun_dir2, output_dir, param_robot, param_init_pose);

    return
elseif(test_val == 7)
    %rmpath('./maple/maple_generated/7_dof_system_fr3');
    %addpath('./maple/maple_generated/7_dof_system_fr3_MW19');

    output_dir = './s_functions/s_functions_7dof/maple_msfun/';
    fun_dir1 = './s_functions/s_functions_7dof/';
    fun_dir2 = './s_functions/s_functions_7dof/maple_msfun/';
    compile_msfunctions(sys_fun_qpp_aba, fun_dir1, fun_dir2, output_dir, param_robot, param_init_pose);

    return
elseif(test_val == 8)
    %rmpath('./maple/maple_generated/7_dof_system_fr3_MW19');
    %addpath('./maple/maple_generated/7_dof_system_fr3');
    disp('dont forget to compile (test_val = 6)');

    qpp_fun_compiled = @(q, q_p, tau) inertia_matrix_py(q)\(tau - n_q_coriols_qp_plus_g_py(q, q_p));

    qpp_fun_maple = @(q, q_p, tau, param) inertia_matrix(q, param)\(tau - coriolis_matrix(q, q_p, param)*q_p - gravitational_forces(q, param));
    qpp_fun_maple_casadi_SX = @(q, q_p, tau, param) inertia_matrix_casadi_SX(q, param)\(tau - coriolis_matrix_casadi_SX(q, q_p, param)*q_p - gravitational_forces_casadi_SX(q, param));

    % load function to workspace
    MM=inertia_matrix(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        MM=inertia_matrix(pi*rand(7,1), param_robot); MC = MM.*rand(1,1);
    end
    disp('Time for 1000 runs M23:'); disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        MM=inertia_matrix_maple(pi*rand(7,1)); MC = MM.*rand(1,1);
    end
    disp('Time for 1000 runs M23 casadi compiled:'); disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        MM=inertia_matrix_py(pi*rand(7,1)); MC = (MC + MM.*rand(1,1))/2;
    end
    disp('Time for 1000 runs of M(q) compiled:'); disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qp_c = pi*rand(7,1);
    q_c = pi*rand(7,1);
    CC=coriolis_matrix(q_c, qp_c, param_robot)*qp_c + gravitational_forces(q_c, param_robot);
    
    tic;
    for i=1:1:1000
        qp_c = pi*rand(7,1);
        q_c = pi*rand(7,1);
        CC=coriolis_matrix(q_c, qp_c, param_robot)*qp_c + gravitational_forces(q_c, param_robot);
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C23_rnea = C(q, qp)qp + g(q):');
    disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        CC=n_q_coriols_qp_plus_g_py(pi*rand(7,1), pi*rand(7,1));
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C_rnea compiled:');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    gg=gravitational_forces(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        gg=gravitational_forces(pi*rand(7,1), param_robot);
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g23');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        gg=gravitational_forces_maple(pi*rand(7,1));
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g23 compiled');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        gg=gravitational_forces_py(pi*rand(7,1));
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g(q) compiled:');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    HH=hom_transform_endeffector(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        HH=hom_transform_endeffector(pi*rand(7,1), param_robot);
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H23');
    disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        HH=hom_transform_endeffector_maple(pi*rand(7,1));
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H23 compiled');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        HH=hom_transform_endeffector_py(pi*rand(7,1));
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H(q) compiled:');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJ=geo_jacobian_endeffector(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJ=geo_jacobian_endeffector(pi*rand(7,1), param_robot);
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J23');
    disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        JJ=geo_jacobian_endeffector_maple(pi*rand(7,1));
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J23 compiled:');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJ=geo_jacobian_endeffector_py(pi*rand(7,1));
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q) compiled:');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJp = geo_jacobian_endeffector_p(pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJp=geo_jacobian_endeffector_p(pi*rand(7,1), pi*rand(7,1), param_robot);
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J23_p');
    disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        JJp=geo_jacobian_endeffector_p_maple(pi*rand(7,1), pi*rand(7,1));
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J23_p compiled');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJp=geo_jacobian_endeffector_p_py(pi*rand(7,1), pi*rand(7,1));
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q, qp) compiled:');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qqpp = qpp_fun_maple(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun_maple(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun_maple23');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun_compiled(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun compiled (Backslash)');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = sys_fun_qpp_aba_py(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of sys_fun_qpp (Aba)');
    disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        qqpp = sys_fun_qpp_sol_py(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of sys_fun_qpp (sol)');
    disp(['   ', num2str(toc), 's']);

    return
elseif(test_val == 9)
    %rmpath('./maple/maple_generated/7_dof_system_fr3');
    %addpath('./maple/maple_generated/7_dof_system_fr3_MW19');
    disp('dont forget to compile (test_val = 7)');

    qpp_fun_compiled = @(q, q_p, tau) inertia_matrix_py(q)\(tau - n_q_coriols_qp_plus_g_py(q, q_p));

    qpp_fun_maple = @(q, q_p, tau, param) inertia_matrix(q, param)\(tau - coriolis_matrix(q, q_p, param)*q_p - gravitational_forces(q, param));
    qpp_fun_maple_casadi_SX = @(q, q_p, tau, param) inertia_matrix_casadi_SX(q, param)\(tau - coriolis_matrix_casadi_SX(q, q_p, param)*q_p - gravitational_forces_casadi_SX(q, param));

    % load function to workspace
    MM=inertia_matrix(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        MM=inertia_matrix(pi*rand(7,1), param_robot); MC = MM.*rand(1,1);
    end
    disp('Time for 1000 runs M19:'); disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        MM=inertia_matrix_maple(pi*rand(7,1)); MC = MM.*rand(1,1);
    end
    disp('Time for 1000 runs M19 casadi compiled:'); disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        MM=inertia_matrix_py(pi*rand(7,1)); MC = (MC + MM.*rand(1,1))/2;
    end
    disp('Time for 1000 runs of M(q) compiled:'); disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qp_c = pi*rand(7,1);
    q_c = pi*rand(7,1);
    CC=coriolis_matrix(q_c, qp_c, param_robot)*qp_c + gravitational_forces(q_c, param_robot);
    
    tic;
    for i=1:1:1000
        qp_c = pi*rand(7,1);
        q_c = pi*rand(7,1);
        CC=coriolis_matrix(q_c, qp_c, param_robot)*qp_c + gravitational_forces(q_c, param_robot);
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C19_rnea = C(q, qp)qp + g(q):');
    disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        CC=n_q_coriols_qp_plus_g_py(pi*rand(7,1), pi*rand(7,1));
        CM = CC.*rand(1,1);
    end
    disp('Time for 1000 runs of C_rnea compiled:');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    gg=gravitational_forces(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        gg=gravitational_forces(pi*rand(7,1), param_robot);
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g19');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        gg=gravitational_forces_maple(pi*rand(7,1));
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g19 compiled');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        gg=gravitational_forces_py(pi*rand(7,1));
        gc=gg.*rand(1,1);
    end
    disp('Time for 1000 runs of g(q) compiled:');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    HH=hom_transform_endeffector(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        HH=hom_transform_endeffector(pi*rand(7,1), param_robot);
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H19');
    disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        HH=hom_transform_endeffector_maple(pi*rand(7,1));
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H19 compiled');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        HH=hom_transform_endeffector_py(pi*rand(7,1));
        HC=HH.*rand(1,1);
    end
    disp('Time for 1000 runs of H(q) compiled:');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJ=geo_jacobian_endeffector(pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJ=geo_jacobian_endeffector(pi*rand(7,1), param_robot);
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J19');
    disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        JJ=geo_jacobian_endeffector_maple(pi*rand(7,1));
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J19 compiled:');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJ=geo_jacobian_endeffector_py(pi*rand(7,1));
        JC=JJ.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q) compiled:');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    JJp = geo_jacobian_endeffector_p(pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        JJp=geo_jacobian_endeffector_p(pi*rand(7,1), pi*rand(7,1), param_robot);
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J19_p');
    disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        JJp=geo_jacobian_endeffector_p_maple(pi*rand(7,1), pi*rand(7,1));
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J19_p compiled');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        JJp=geo_jacobian_endeffector_p_py(pi*rand(7,1), pi*rand(7,1));
        JCp=JJp.*rand(1,1);
    end
    disp('Time for 1000 runs of J(q, qp) compiled:');
    disp(['   ', num2str(toc), 's']);
    
    % load function to workspace
    qqpp = qpp_fun_maple(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun_maple(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1), param_robot);
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun_maple19');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = qpp_fun_compiled(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of qpp_fun compiled (Backslash)');
    disp(['   ', num2str(toc), 's']);
    
    tic;
    for i=1:1:1000
        qqpp = sys_fun_qpp_aba_py(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of sys_fun_qpp (Aba)');
    disp(['   ', num2str(toc), 's']);

    tic;
    for i=1:1:1000
        qqpp = sys_fun_qpp_sol_py(pi*rand(7,1), pi*rand(7,1), pi*rand(7,1));
        qqppc = qqpp.*rand(1,1);
    end
    disp('Time for 1000 runs of sys_fun_qpp (sol)');
    disp(['   ', num2str(toc), 's']);

    return
elseif(test_val == 10)
    MC=eye(7);tic;
    for i=1:1:1000
        MM=inertia_matrix_py(pi*rand(7,1))\eye(7); MC = (MC + MM.*rand(1,1))/2;
    end
    disp('Time for 1000 runs of inertia_matrix_py\E python compiled:'); disp(['   ', num2str(toc), 's']);
    MC=eye(7);tic;
    for i=1:1:1000
        MM=inertia_matrix_py(pi*rand(7,1))\eye(7); MC = (MC + MM.*rand(1,1))/2;
    end
    disp('Time for 1000 runs of inverse_inertia_matrix_py python compiled:'); disp(['   ', num2str(toc), 's']);
    tic;
    for i=1:1:1000
        MM=inertia_matrix(pi*rand(7,1), param_robot)\eye(7); MC = (MC + MM.*rand(1,1))/2;
    end
    disp('Time for 1000 runs of inertia_matrix\E (maple):'); disp(['   ', num2str(toc), 's']);
     tic;
    for i=1:1:1000
        MM=M_inv(pi*rand(7,1)); MC = (MC + MM.*rand(1,1))/2;
    end
    disp('Time for 1000 runs of M_inv (python):'); disp(['   ', num2str(toc), 's']);
else
    disp('tests.m does nothing')
end

function compile_msfunctions(sys_fun_qpp, fun_dir1, fun_dir2, output_dir, param_robot, param_init_pose)
    qc = sys_fun_qpp.sx_in{1};
    qc_p = sys_fun_qpp.sx_in{2};
    tau = sys_fun_qpp.sx_in{3};

    % Idee: Wenn n < 7 ist dann ersetzt man die joint winkel die nicht im urdf file fixed sind!
    q_SX   = casadi.SX(param_init_pose.q_0_init);
    q_p_SX = casadi.SX(param_init_pose.q_0_p_init);

    q_SX(param_robot.n_indices) = qc;
    q_p_SX(param_robot.n_indices) = qc_p;

    inertia_matrix_maple_casadi = casadi.Function('inertia_matrix_maple', {qc}, {inertia_matrix_casadi_SX(q_SX, param_robot)});
    % Die coriolis matrix is einfach zu groß für casadi, ließe sich
    % höchstens als nlpsol sfunktion kompilieren.
    %coriolis_rnea_maple_casadi = casadi.Function('C_rnea_maple', {qc, qc_p}, {coriolis_rnea_casadi_SX(q_SX, qc_p, param_robot)});
    %coriolis_rnea_maple_casadi_v2 = casadi.Function('C_rnea_maple_v2', {qc, qc_p}, {coriolis_rnea_casadi_SX(q_SX, qc_p, param_robot)});
    gravitational_forces_maple_casadi = casadi.Function('g_maple', {qc}, {gravitational_forces_casadi_SX(q_SX, param_robot)});
    hom_transform_endeffector_maple_casadi = casadi.Function('H_maple', {qc}, {hom_transform_endeffector_casadi_SX(q_SX, param_robot)});
    geo_jacobian_endeffector_maple_casadi = casadi.Function('J_maple', {qc}, {geo_jacobian_endeffector_casadi_SX(q_SX, param_robot)});
    geo_jacobian_endeffector_p_maple_casadi = casadi.Function('J_p_maple', {qc, qc_p}, {geo_jacobian_endeffector_p_casadi_SX(q_SX, q_p_SX, param_robot)});

    inertia_matrix_maple_casadi.save([output_dir, 'inertia_matrix_maple.casadi']);
    %coriolis_rnea_maple_casadi.save([output_dir, 'coriolis_rnea_maple.casadi']);
    %coriolis_rnea_maple_casadi_v2.save([output_dir, 'coriolis_rnea_maple_v2.casadi']);
    gravitational_forces_maple_casadi.save([output_dir, 'gravitational_forces_maple.casadi']);
    hom_transform_endeffector_maple_casadi.save([output_dir, 'hom_transform_endeffector_maple.casadi']);
    geo_jacobian_endeffector_maple_casadi.save([output_dir, 'geo_jacobian_endeffector_maple.casadi']);
    geo_jacobian_endeffector_p_maple_casadi.save([output_dir, 'geo_jacobian_endeffector_p_maple.casadi']);

    fun_arr = { ...
        'sys_fun_qpp_aba_py', ...
        'sys_fun_qpp_sol_py', ...
        'inertia_matrix_py', ...
        'inverse_inertia_matrix_py', ...
        'n_q_coriols_qp_plus_g_py', ...
        'gravitational_forces_py', ...
        'hom_transform_endeffector_py', ...
        'quat_endeffector_py', ...
        'geo_jacobian_endeffector_py', ...
        'geo_jacobian_endeffector_p_py' ...
    };

    % add hom. transformation of joints
    joint_arr = cell(1, param_robot.n_DOF);
    for i = 1:param_robot.n_DOF
        joint_arr{i} = ['hom_transform_joint_', num2str(i), '_py'];
    end

    fun_arr = [fun_arr, joint_arr];

    compile_to_matlabsfun(fun_dir1, output_dir, fun_arr);

    % compile maple19/23
    fun_arr = { ...
        'inertia_matrix_maple', ...
        'gravitational_forces_maple', ...
        'hom_transform_endeffector_maple', ...
        'geo_jacobian_endeffector_maple', ...
        'geo_jacobian_endeffector_p_maple' ...
    };
    compile_to_matlabsfun(fun_dir2, output_dir, fun_arr);
end


function compile_to_matlabsfun(fun_dir, output_dir, fun_arr)
    try
        for i = 1:length(fun_arr)
            fun_name = fun_arr{i};
            
            sys_fun = casadi.Function.load([fun_dir, fun_name, '.casadi']);

            tic;
            casadi_fun_to_mex(sys_fun, output_dir, fun_name, '-O2');
            disp(['Compile time for matlab s-function: ', num2str(toc), ' s']);
        end
    catch ME
        disp('Error in casadi_fun_to_mex.m')
        fprintf(2, 'Fehler: %s\n', getReport(ME));
    end
end