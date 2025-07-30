function param_path = prepare_path(H_ee, time)
%PREPARE_PATH Prepare the path for spline interpolation.
%   param_path = prepare_path(H_ee, time) takes the end-effector poses
%   H_ee and the corresponding time vector, and prepares the path for
%   spline interpolation by extracting control points, orientations, and
%   velocities.

    degree = 3;

    param_path = struct();
    param_path.degree = degree;
    
    % add knots at start and end
    knot = time / time(end);
    [knot_num, ~] = size(knot);
    param_path.knot = [zeros(degree-1,1); knot; ones(degree-1,1)];
    
    % extract data
    position = squeeze(H_ee(1:3,4,:))';
    
    % rotation -> store as quaternions for now
    rotation = zeros(knot_num, 4);
    signflip = 1;
    for i = 1:knot_num
        rotation(i,:) = signflip * rotation2quaternion(H_ee(1:3,1:3,i));
        if i<1 && norm(rotation(i,:)-rotation(i-1,:))>1
            signflip = -signflip;
            rotation(i,:) = -rotation(i,:);
        end
    end
    
    % append stuff at beginning and end
    param_path.position = [repmat(position(1,:), [degree-1,1]); position; repmat(position(end,:), [degree-1,1])];
    param_path.rotation = [repmat(rotation(1,:), [degree-1,1]); rotation; repmat(rotation(end,:), [degree-1,1])];
    
    % precalculate the exp values
    param_path.delta = zeros(knot_num-1,4);
    for i = 1: knot_num + 2*(degree-1)-1
        % the difference is given by log(q0^(-1)*q1)
        param_path.delta(i,:) = quatlog( quatmultiply( quatinv( param_path.rotation(i,:)), param_path.rotation(i+1,:)));
    end
end

