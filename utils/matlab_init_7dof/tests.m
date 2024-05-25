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
9i
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