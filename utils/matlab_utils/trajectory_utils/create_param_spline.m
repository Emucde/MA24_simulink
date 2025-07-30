function traj_struct_out = create_param_spline(traj_struct, param_global, text1, Ind_deriv, text2, alpha, text3, p)
%CREATE_PARAM_SPLINE Create a parametric spline trajectory from a given trajectory structure.
%   traj_struct_out = CREATE_PARAM_SPLINE(traj_struct, param_global, text1, Ind_deriv, text2, alpha, text3, p)
%   creates a parametric spline trajectory based on the input trajectory structure.
%   The function allows for customization of the spline parameters such as
%   the indices of derivatives, the scaling factors (alpha), and the polynomial order (p).
%%   Inputs:
%       traj_struct - Structure containing the trajectory data.
%       param_global - Global parameters for the trajectory.
%       text1 - Description for the derivative indices (default: 'Ind_deriv').
%       Ind_deriv - Indices of the derivatives to be used in the spline (default: [1 1]).
%       text2 - Description for the scaling factors (default: 'alpha').
%       alpha - Scaling factors for the derivatives (default: [1 1]).
%       text3 - Description for the polynomial order (default: 'order').
%       p - Polynomial order for the spline (default: 3).
%%   Outputs:
%       traj_struct_out - Structure containing the updated trajectory with the spline.
    arguments
        traj_struct struct {mustBeNonempty}
        param_global struct {mustBeNonempty}
        text1 char = 'Ind_deriv'
        Ind_deriv double = [1 1]
        text2 char = 'alpha'
        alpha double = [1 1]
        text3 char = 'order'
        p double = 3
    end
   
    %% Param sinus poly trajectory

    if(p == 0)
        QQ = ones(1,3) .* [1:1:size(traj_struct.pose, 2)]';
        p = 3;
    else
        QQ = traj_struct.pose(1:3, :)';
    end

    if(isempty(Ind_deriv))
        Ind_deriv = 1:1:(size(QQ, 1)-2);
    end
    if(isempty(alpha))
        alpha = zeros(1, size(QQ, 1));
        alpha(2:end-1) = 1;
    end
    if(isempty(p))
        p = 3;
    end

    
    
    TT = [QQ(2:end, :)- QQ(1:end-1, :); QQ(end, :)- QQ(end-1, :)];
    % T1 = [1 0 0] active at Q1 = QQ(2,:), T2 = [1 1 0] active at Q2 = QQ(3,:)
    
    % T0 and Tn are ignored (alpha(1) = alpha(end) = 0), T1 and T2 are scaled
    % with approximated chord length 1*d. This means for this example:
    % alpha(1) is active on T0 = TT(1, :),
    % alpha(2) is active on T1 = TT(1+Ind_deriv(1), :)
    % alpha(3) is active on T2 = TT(1+Ind_deriv(2), :)
    % alpha(end) is active on Tn = TT(end, :)
    
    bspline = bsplineCurveFitting(QQ, TT, Ind_deriv, alpha, p);

    traj_struct.bspline = bspline;

    traj_struct_out = traj_struct;
end