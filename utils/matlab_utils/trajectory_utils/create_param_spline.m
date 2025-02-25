function traj_struct_out = create_param_spline(traj_struct, param_global, text1, Ind_deriv, text2, alpha, text3, p)
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
    QQ = traj_struct.pose(1:3, :)';
    
    TT = [QQ(2:end, :)- QQ(1:end-1, :); QQ(end, :)- QQ(end-1, :)];
    
    % T1 = [1 0 0] active at Q1 = QQ(2,:), T2 = [1 1 0] active at Q2 = QQ(3,:)
    
    % T0 and Tn are ignored (alpha(1) = alpha(end) = 0), T1 and T2 are scaled
    % with approximated chord length 1*d. This means for this example:
    % alpha(1) is active on T0 = TT(1, :),
    % alpha(2) is active on T1 = TT(1+Ind_deriv(1), :)
    % alpha(3) is active on T2 = TT(1+Ind_deriv(2), :)
    % alpha(end) is active on Tn = TT(end, :)
    
    p = 3; % spline order
    
    bspline = bsplineCurveFitting(QQ, TT, Ind_deriv, alpha, p);

    traj_struct.bspline = bspline;

    traj_struct_out = traj_struct;
end