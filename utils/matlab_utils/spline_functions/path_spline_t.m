function [sigma, dsigma, ddsigma, dddsigma] = path_spline_t(theta, param_path)
    %% path_spline_t - Evaluate a B-spline path at a given parameter theta.
    %  Inputs:
    %    theta - parameter value in [0,1] for the path  
    %    param_path - structure containing the B-spline path parameters
    %  Outputs:
    %    sigma - position on the path
    %    dsigma - first derivative (tangent vector)
    %    ddsigma - second derivative (curvature vector)
    %    dddsigma - third derivative (jerk vector)
    theta_limit=max(min(theta,1-1e-6),0);

    degree=3;
    
    %% B-Spline basis functions
    i=bspline_findspan(theta_limit,param_path);
    N=bspline_basisfunction(theta_limit,i,3,param_path);
    
    %% translation
    p=sum(param_path.position(i:i+degree,:).*repmat(N(1,1:degree+1)',[1,3]),1);
    p_p=sum(param_path.position(i:i+degree,:).*repmat(N(2,1:degree+1)',[1,3]),1);
    p_pp=sum(param_path.position(i:i+degree,:).*repmat(N(3,1:degree+1)',[1,3]),1);
    p_ppp=sum(param_path.position(i:i+degree,:).*repmat(N(4,1:degree+1)',[1,3]),1);
    
    if(theta~=theta_limit)
        % Interpolate path beyond the ends linearly and keep the orientation
        % constant. Leave q and p_p unchanged.
        p=p+p_p*(theta-theta_limit);
        p_pp=param_path.position(1,:)*0;
        p_ppp=param_path.position(1,:)*0;
    end
    
    %% set the output
    sigma=p;
    dsigma=p_p;
    ddsigma=p_pp;
    dddsigma=p_ppp;
end

