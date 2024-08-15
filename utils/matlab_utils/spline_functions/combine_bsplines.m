function bspline_comb = combine_bsplines(bspline_arr)
    % combine_bsplines - Combines an array of NPath structs into a single struct.
    %
    % INPUT:
    %   bspline_arr: Array of NPath structs (1xN) with fields 'degree',
    %               'knot', and 'control_points'.
    %
    % OUTPUT:
    %   bspline_comb: Struct containing the combined information with fields
    %                 'degree', 'knot', 'control_points', 'start_knot',
    %                 'end_knot', 'start_control_point', 'end_control_point'.
    %

    N_path = length(bspline_arr);

    start_knot = zeros(1, N_path);
    end_knot = zeros(1, N_path);
    start_control_point = zeros(1, N_path);
    end_control_point = zeros(1, N_path);

    N_knots = 0;
    N_control_points = 0;
    for i = 1:N_path
        start_knot(i) = N_knots+1;
        start_control_point(i) = N_control_points + 1;
        N_knots = N_knots + length(bspline_arr(i).knot);
        N_control_points = N_control_points + length(bspline_arr(i).control_points);
        end_knot(i) = N_knots;
        end_control_point(i) = N_control_points;
    end

    bspline_comb = struct();
    bspline_comb.degrees = zeros(1, N_path);
    bspline_comb.knots = zeros(1, N_knots);
    bspline_comb.control_points = zeros(N_control_points, 3);
    bspline_comb.start_knot = zeros(1, N_path);
    bspline_comb.end_knot = zeros(1, N_path);
    bspline_comb.start_control_point = zeros(1, N_path);
    bspline_comb.end_control_point = zeros(1, N_path);

    for i = 1:N_path
        bspline_comb.degrees(i) = bspline_arr(i).degree;
        bspline_comb.knots(start_knot(i):end_knot(i)) = bspline_arr(i).knot;
        bspline_comb.control_points(start_control_point(i):end_control_point(i), :) = bspline_arr(i).control_points;
        bspline_comb.start_knot(i) = start_knot(i);
        bspline_comb.end_knot(i) = end_knot(i);
        bspline_comb.start_control_point(i) = start_control_point(i);
        bspline_comb.end_control_point(i) = end_control_point(i);
    end

end