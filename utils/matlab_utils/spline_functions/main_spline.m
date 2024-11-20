clc;clear;

% Inputdaten:
y = [1/2, -1/4, 1/4, -1/2]; % = Q_0, Q_1, ..., Q_n
z = [1 0 0 0];
x = (0:length(y)-1)/(length(y)-1);
QQ = [x; y; z]'; % e.g. measured data

% Use custom tang vek
T0 = [1 -1 0];
Tk = [1 0 0; 1 1 0];
Tn = [0 1 1];

% or use vektors between the points:
QQvec = [QQ(2:end,:) - QQ(1:end-1,:); QQ(end,:)-QQ(end-1,:)];
QQvec = QQvec./vecnorm(QQvec',2)';

%TT = [T0; Tk; Tn];
%TT = [QQvec(1,:); 1 0 0; 1 0 0; QQvec(4,:)];
TT = QQvec;
%TT = [QQvec(1,:); 1 0 0; -QQvec(4,:)];
%TT = [QQvec(1,:); -QQvec(4,:)];
tangvec_norm = TT./vecnorm(TT',2)';

Ind_deriv = [1, 2]; % Länge ist = l-2
alpha = [3 1 1 1];

% p ... spline Ordnung, n ... Anzahl der Punkte (ohne Q_0)
p = 3;

%% The Code
bspline = bsplineCurveFitting(QQ, TT, Ind_deriv, alpha, p);
PP = bspline.control_points;

if(~any(isnan(PP)))
    inefficient_spline_calc = false;
    if(inefficient_spline_calc)
        N_fun = @(u) arrayfun(@(i) get_basisfun_Ni(i,p,u,knots), 0:r);
        Cx = @(uu) arrayfun(@(u) N_fun(u)*PP(1:r+1, 1), uu);
        Cy = @(uu) arrayfun(@(u) N_fun(u)*PP(1:r+1, 2), uu);
        Cz = @(uu) arrayfun(@(u) N_fun(u)*PP(1:r+1, 3), uu);
        
        dN_fun = @(u) arrayfun(@(i) get_basisfun_dNi(i,p,u,knots), 0:r);
        dCx = @(uu) arrayfun(@(u) dN_fun(u)*PP(1:r+1, 1), uu);
        dCy = @(uu) arrayfun(@(u) dN_fun(u)*PP(1:r+1, 2), uu);
        dCz = @(uu) arrayfun(@(u) dN_fun(u)*PP(1:r+1, 3), uu);
    else
        Cx = @(uu) arrayfun(@(u) C_tj_fun(u, 1, bspline), uu);
        Cy = @(uu) arrayfun(@(u) C_tj_fun(u, 2, bspline), uu);
        Cz = @(uu) arrayfun(@(u) C_tj_fun(u, 3, bspline), uu);
        dCx = @(uu) arrayfun(@(u) dC_tj_fun(u, 1, bspline), uu);
        dCy = @(uu) arrayfun(@(u) dC_tj_fun(u, 2, bspline), uu);
        dCz = @(uu) arrayfun(@(u) dC_tj_fun(u, 3, bspline), uu);
    end

    sigma_fun = @(theta) [Cx(theta); Cy(theta); Cz(theta)];
    dsigma_fun = @(theta) [dCx(theta); dCy(theta); dCz(theta)];

    tt = 0:1e-3:1;
    subplot(3,2,[1 3]);

    plot3(Cx(tt), Cy(tt), Cz(tt));
    hold on;grid on;grid minor;
    plot3(QQ(:,1), QQ(:,2), QQ(:,3), 'rx', 'MarkerSize', 10, 'MarkerFaceColor', 'None'); % chord length parameter t_j
    plot3(PP(:,1)', PP(:,2)', PP(:,3)', 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'None'); % control points P_j

    l = size(PP,1) - size(QQ,1) - 1;
    headsize = 0.5;
    scale = 0.5;
    h=quiver3(QQ(1,1), QQ(1,2), QQ(1,3), tangvec_norm(1,1), tangvec_norm(1,2), tangvec_norm(1,3), '-', 'Color', [1,0.5,0.5], 'LineWidth',1, 'AutoScale','on', 'AutoScaleFactor',scale, 'MaxHeadSize', headsize);
    for i = 2:l
        quiver3(QQ(i,1), QQ(i,2), QQ(i,3), tangvec_norm(i,1), tangvec_norm(i,2), tangvec_norm(i,3), '-', 'Color', [1,0.5,0.5], 'LineWidth',1, 'AutoScale','on', 'AutoScaleFactor',scale, 'MaxHeadSize', headsize);
    end
    quiver3(QQ(end,1), QQ(end,2), QQ(end,3), tangvec_norm(end,1), tangvec_norm(end,2), tangvec_norm(end,3), '-', 'Color', [1,0.5,0.5], 'LineWidth',1, 'AutoScale','on', 'AutoScaleFactor',scale, 'MaxHeadSize', headsize);

    axis equal
    legend("resulting C(\theta)",'desired Q_j','control points P_i','Orientation','horizontal','Location','northoutside')
    %xlim([t_tilde(3)-0.1, t_tilde(3)+0.1]);ylim(sort([C(t_tilde(3)-0.1), C(t_tilde(3)+0.1)]))
    %xlim([0.9995, 1.0005])
    %ylim([-0.5005, -0.4995])
    %subplot(1,3,3);
    %plot(dCx(tt), dC(tt))

    subplot(3,2,5);
    plot3(dCx(tt), dCy(tt), dCz(tt));
    hold on;grid on;grid minor;
    dCx_approx = 1/(tt(2)-tt(1))*diff(Cx(tt));
    dCy_approx = 1/(tt(2)-tt(1))*diff(Cy(tt));
    dCz_approx = 1/(tt(2)-tt(1))*diff(Cz(tt));
    plot3(dCx_approx, dCy_approx, dCz_approx);
    DD = bspline.tangents;
    plot3(DD(:,1), DD(:,2), DD(:,3), 'rx', 'MarkerSize', 10, 'MarkerFaceColor', 'None'); % chord length parameter t_j
    %plot3(PP(:,1)', PP(:,2)', PP(:,3)', 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'None'); % control points P_j

    axis equal
    legend("resulting C'(\theta)",'numerical deviation of C(\theta)','desired D_k', 'Orientation','horizontal', 'Location','southoutside')

    subplot(3,2,2);
    plot(tt, dCx(tt));
    hold on;grid on;
    plot(tt(2:end), dCx_approx);
    legend('d/d\theta C_x(\theta)', 'numerical deviation of C_x(\theta)','Orientation','horizontal', 'Location','southoutside')

    subplot(3,2,4);
    plot(tt, dCy(tt));
    hold on;grid on;
    plot(tt(2:end), dCy_approx);
    legend('d/d\theta C_y(\theta)', 'numerical deviation of C_y(\theta)','Orientation','horizontal','Location','southoutside')

    subplot(3,2,6);
    plot(tt, dCz(tt));
    hold on;grid on;
    plot(tt(2:end), dCz_approx);
    legend('d/d\theta C_z(\theta)', 'numerical deviation of C_z(\theta)','Orientation','horizontal','Location','southoutside')

end

%% TESTING (example from Curve interpolation with directional constraints for engineering design, 2008)
testing = false;
if(testing == true)
    Ind = [0,1,2,5];
    Ind_deriv = [1,2];
    t = [0, 0.1599, 0.3198, 0.5231, 0.6997, 1];
    p=3;
    [a,b] = missing_knots(t, p, Ind, Ind_deriv)
end

%% Idee um ohne rekursion bsplineBasis zu berechnen. Jedoch ist es extrem
% ineffizent im Vergleich zu bspline_basisfunction.m und bspline_findspan.m
% Ich suche nur N_{i,p}. Es gilt N1 = N_{1,p} N2 = N_{2,p} ... N_n =
% N_{n,p} Zur Berechnugn von N1 brauche ich N1
%{
u = 0; N_i_p = zeros(1,n);


for i=1:n
    
end

for i=1:n u_i = knots(1+(i)); u_ip1 = knots(1+(i+1)); u_ip2 =
knots(1+(i+2)); N_i_0 = double((u >= u_i && u <= u_ip1)); N_ip1_0 =
double((u >= u_ip1 && u <= u_ip2)); for j=1:p u_ipj = knots(1+(i+j));
u_ipjp1 = knots(1+(i+j+1)); w1 = (u-u_i)/(u_ipp-u_i); w2 =
(u_ippp1-u)/(u_ippp1 - u_ip1); N_i_j = w1*N_i_0 + w2*N_ip1_0; end end
%}

% Beispiel: n=2, p=1:

% N_{0,0} = (u >= u_0 && u <= u_1) N_{1,0} = (u >= u_1 && u <= u_2) N_{2,0}
% = (u >= u_2 && u <= u_3) N_{3,0} = (u >= u_3 && u <= u_4)

% N_{0,1} = (u-u_0)/(u_1-u_0)*N_{0,0} + (u_2-u)/(u_2-u_1)*N_{1,0} N_{1,1} =
% (u-u_1)/(u_2-u_1)*N_{1,0} + (u_3-u)/(u_3-u_2)*N_{2,0} N_{2,1} =
% (u-u_2)/(u_3-u_2)*N_{2,0} + (u_4-u)/(u_4-u_3)*N_{3,0}

% Beispiel: n=2, p=2: damit sind für 2.42 gesucht: N_{0,2}, N_{1,2} und
% N_{2,2}:

% N_{0,0} = (u >= u_0 && u <= u_1) N_{1,0} = (u >= u_1 && u <= u_2) N_{2,0}
% = (u >= u_2 && u <= u_3) N_{3,0} = (u >= u_3 && u <= u_4) N_{4,0} = (u >=
% u_4 && u <= u_5)

% N_{0,1} = (u-u_0)/(u_1-u_0)*N_{0,0} + (u_2-u)/(u_2-u_1)*N_{1,0} N_{1,1} =
% (u-u_1)/(u_2-u_1)*N_{1,0} + (u_3-u)/(u_3-u_2)*N_{2,0} N_{2,1} =
% (u-u_2)/(u_3-u_2)*N_{2,0} + (u_4-u)/(u_4-u_3)*N_{3,0} N_{3,1} =
% (u-u_3)/(u_4-u_3)*N_{2,0} + (u_5-u)/(u_5-u_4)*N_{4,0}

% N_{0,2} = (u-u_0)/(u_2-u_0)*N_{0,1} + (u_3-u)/(u_3-u_1)*N_{1,1} N_{1,2} =
% (u-u_1)/(u_3-u_1)*N_{1,1} + (u_4-u)/(u_4-u_2)*N_{2,1} N_{2,2} =
% (u-u_2)/(u_4-u_2)*N_{2,1} + (u_5-u)/(u_5-u_3)*N_{3,1}

%%
%{
u=0.5 N = zeros(p, p+n); for i=0:p+n u_i = knots(1+(i)); u_ip1 =
knots(1+(i+1)); N(1+(i), 1+(0)) = (u >= u_i && u <= u_ip1); end

for j=1:n for i=0:p+n-j u_i = knots(1+(i)); u_ip1 = knots(1+(i+1)); u_ipj =
knots(1+(i+j)); u_ipjp1 = knots(1+(i+j+1)); %w1 = (u-u_i)/(u_ipj-u_i); %w2
= (u_ipjp1-u)/(u_ipjp1 - u_ip1); if(u_ipj == u_i) w1 = 0; else w1 =
(u-u_i)/(u_ipj-u_i); end

if(u_ipjp1 == u_ip1) w2 = 0; else w2 = (u_ipjp1-u)/(u_ipjp1 - u_ip1); end
N(1+(i), 1+(j)) = w1*N(1+(i), 1+(j-1)) + w2*N(1+(i), 1+(j-1)); end end
%}

%% Gedanken bezgl diff
%{
omega=2*pi*1;
xxx = sin(omega*tt);
dxxx = diff(xxx)/(tt(2)-tt(1));
plot(tt, xxx);
hold on
plot(tt(2:end), dxxx);
plot(tt, omega*cos(omega*tt))
%}
%%

%  P0       = Q0 -P0 + P1 = gamma0*D0 N_1_p(1)*P1 +  N_2_p(1)*P2 + ... +
%  N_p+1_p(1)*Pp+1 = Q1
% dN_1_p(1)*P1 + dN_2_p(1)*P2 + ... + dN_p+1_p(1)*Pp+1 = gamma1*D1
%                 N_2_p(1)*P2 +  N_3_p(1)*P3 + ... +  N_p+2_p(1)*Pp+2 = Q2
%                dN_2_p(1)*P2 + dN_3_p(1)*P3 + ... + dN_p+2_p(1)*Pp+2 =
%                gamma2*D2
% . . .
%  N_n+l-p_p(1)*Pn+l-p +  N_n+l-p+1_p(1)*Pn+l-p+1 + ... +
%  N_n+l+1_p(1)*Pn+l+1 = Qn+l-p

%% CALCULATE SPLINES WITH bspline_basisfunciton.m und bsplin_findspan
% Effiziente Berechnung!
%{
figure(2)

bspline.degree = p;
bspline.knot = knots;
k=0;
N = 1000;
theta_data = (0:N-1)/(N-1);
datax = zeros(N,1);
datay = zeros(N,1);
for j=1:N
    theta=theta_data(j);
    i = bspline_findspan(theta, bspline);
    datax(j) = PP(i-p+1:i+1,1)' * bspline_basisfunction(theta,i,k,bspline)';
    datay(j) = PP(i-p+1:i+1,2)' * bspline_basisfunction(theta,i,k,bspline)';
end
plot(datax, datay)
hold on;
plot(QQ(:,1), QQ(:,2), 'rx', 'MarkerSize', 10, 'MarkerFaceColor', 'None')
%}
%%

%%

function res = row_fun(data_in, j, n)
    n_data = length(data_in);
    vec = [data_in, zeros(1,n-n_data)];
    res = circshift(vec, j);
end

function Ni = get_basisfun_Ni(i,p,u,knots)
    [Ni, ~] = bsplineBasis(i,p,u,knots);
end

function dNi = get_basisfun_dNi(i,p,u,knots)
    [~, dNi] = bsplineBasis(i,p,u,knots);
end

function [CC] = C_fun(theta, k, bspline)
    i = bspline_findspan(theta, bspline);
    p = bspline.degree;
    control_points = bspline.control_points;
    CC = control_points(1+(i-p):1+(i),:)' * bspline_basisfunction(theta,i,k,bspline)';
end

function [Crow] = C_tj_fun(theta, row, bspline)
    CC = C_fun(theta, 1, bspline);
    Crow = CC(row, 1);
end

function [dCrow] = dC_tj_fun(theta, row, bspline)
    CC = C_fun(theta, 1, bspline);
    dCrow = CC(row, 2);
end

function data = get_qdata(C)
    data = C(1,:);
end