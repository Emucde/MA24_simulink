function plot_spline(bspline, QQ, TT, alpha, Ind_deriv, theta_arr, tangscale)
    % Input:
    %   PP: Control points
    %   DD: Tangents
    %   TT: Spline function
    %   alpha: Parameters
    %   QQ: Chord length parameter
    %   spline_vecfun: Spline function
    clf
    PP = bspline.control_points;
    DD = bspline.tangents;
    tangvec_norm = TT./vecnorm(TT',2)';
    [C_fun, dC_fun] = spline_vecfun(bspline);
    
    subplot(3,2,[1 3]);
    
    C_data = C_fun(theta_arr);
    dC_data = dC_fun(theta_arr);
    
    plot3(C_data(:,1), C_data(:,2), C_data(:,3));
    hold on;grid on;grid minor;
    plot3(QQ(:,1), QQ(:,2), QQ(:,3), 'rx', 'MarkerSize', 10, 'MarkerFaceColor', 'None'); % chord length parameter t_j
    plot3(PP(:,1)', PP(:,2)', PP(:,3)', 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'None'); % control points P_j
    
    l = size(PP,1) - size(QQ,1) - 1;
    headsize = 0.5;
    scale = tangscale;
    if(alpha(1) ~= 0)
        quiver3(QQ(1,1), QQ(1,2), QQ(1,3), tangvec_norm(1,1), tangvec_norm(1,2), tangvec_norm(1,3), '-', 'Color', [1,0.5,0.5], 'LineWidth',1, 'AutoScale','on', 'AutoScaleFactor',scale, 'MaxHeadSize', headsize);
    end
    for i = 2:l
        if(alpha(i) ~=0)
            quiver3(QQ(1+Ind_deriv(i-1),1), QQ(1+Ind_deriv(i-1),2), QQ(1+Ind_deriv(i-1),3), tangvec_norm(i,1), tangvec_norm(i,2), tangvec_norm(i,3), '-', 'Color', [1,0.5,0.5], 'LineWidth',1, 'AutoScale','on', 'AutoScaleFactor',scale, 'MaxHeadSize', headsize);
        end
    end
    if(alpha(end) ~= 0)
        quiver3(QQ(end,1), QQ(end,2), QQ(end,3), tangvec_norm(end,1), tangvec_norm(end,2), tangvec_norm(end,3), '-', 'Color', [1,0.5,0.5], 'LineWidth',1, 'AutoScale','on', 'AutoScaleFactor',scale, 'MaxHeadSize', headsize);
    end
    
    axis equal
    legend("resulting $C(\theta)$",'desired $Q_j$','control points $P_i$','Orientation','horizontal','Location','northoutside')
    view(90,0)
    subplot(3,2,5);
    
    plot3(dC_data(:,1), dC_data(:,2), dC_data(:,3));
    hold on;grid on;grid minor;
    dC_approx = 1/(theta_arr(2)-theta_arr(1))*diff(C_data);
    plot3(dC_approx(:,1), dC_approx(:,2), dC_approx(:,3));
    
    plot3(DD(:,1), DD(:,2), DD(:,3), 'rx', 'MarkerSize', 10, 'MarkerFaceColor', 'None'); % chord length parameter t_j
    
    axis equal
    legend("resulting $C'(\theta)$",'numerical deviation of $C(\theta)$','desired $D_k$', 'Orientation','horizontal', 'Location','southoutside')
    
    subplot(3,2,2);
    plot(theta_arr, dC_data(:,1));
    hold on;grid on;
    plot(theta_arr(2:end), dC_approx(:,1));
    legend('$d/d\theta C_x(\theta)$', 'numerical deviation of $C_x(\theta)$','Orientation','horizontal', 'Location','southoutside')
    
    subplot(3,2,4);
    plot(theta_arr, dC_data(:,2));
    hold on;grid on;
    plot(theta_arr(2:end), dC_approx(:,2));
    legend('$d/d\theta C_y(\theta)$', 'numerical deviation of $C_y(\theta)$','Orientation','horizontal','Location','southoutside')
    
    subplot(3,2,6);
    plot(theta_arr, dC_data(:,3));
    hold on;grid on;
    plot(theta_arr(2:end), dC_approx(:,3));
    legend('$d/d\theta C_z(\theta)$', 'numerical deviation of $C_z(\theta)$','Orientation','horizontal','Location','southoutside', 'interpreter', 'latex')
end