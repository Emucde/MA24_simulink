function plot_path_spline(param_path, points, mode)
%PLOT_PATH_SPLINE Plots a path spline and optionally points.
%   param_path: function handle for the path spline, e.g., @path_spline
%   points: optional 3D points to plot, given as a 3xN matrix
%   mode: optional mode for plotting (1 for detailed, 0 for simple)
    
    % calculate points
    N_points = 1000;
    theta = linspace(0,1,N_points);
    pos = zeros(3,N_points);
    dx = zeros(3,N_points);
    ori = zeros(3, 3, N_points);
    for i = 1:N_points
        [hom_coord, d_hom_coord] = path_spline(theta(i), param_path); 
        pos(:,i) = hom_coord(1:3,4);
        dx(:,i) = d_hom_coord(1:3,4);
        ori(:,:,i) = hom_coord(1:3,1:3);
    end
    
    if(mode == 1)

        % plot position
        figure(1)
        title('path splines')
        %hold off
        plot3(pos(1,:), -pos(2,:), -pos(3,:));
        hold on
        if ~isempty(points)
            for i = 1:size(points,3)
                plot3(points(1,i), -points(2,i), -points(3,i), 'x');
            end
        end
        legend({'path 1', 'path 2', 'path 3'})
        hold off
        axis equal
        
        % orientation plot
        figure(2)
        title('path with orientations')
        %hold off
        for i = 1:N_points
            e1 = 0.1*ori(1:3,1,i);
            e2 = 0.1*ori(1:3,2,i);
            e3 = 0.1*ori(1:3,3,i);
            x = pos(:,i);
            plot3([x(1) x(1)+e1(1)], -[x(2) x(2)+e1(2)], -[x(3) x(3)+e1(3)], 'r');
            hold on
            plot3([x(1) x(1)+e2(1)], -[x(2) x(2)+e2(2)], -[x(3) x(3)+e2(3)], 'g');
            plot3([x(1) x(1)+e3(1)], -[x(2) x(2)+e3(2)], -[x(3) x(3)+e3(3)], 'b');
        end
        hold off
        axis equal
        
        % validate derivatives
        figure(3)
        title('path derivations')
        %hold off
        for i = 1:N_points
            x = pos(:,i);
            plot3([x(1) x(1)+dx(1,i)], [x(2) x(2)+dx(2,i)], [x(3) x(3)+dx(3,i)], 'r');
            hold on
        end
        hold off
        axis equal
    else
        plot3(pos(1,:), pos(2,:), pos(3,:), 'Color', 'magenta');
    end
end

