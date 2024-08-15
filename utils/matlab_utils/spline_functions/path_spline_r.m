function [sigma, dsigma, ddsigma, dddsigma] = path_spline_r(theta, param_path)
    %% 
    theta_limit=max(min(theta,1-1e-6),0);

    degree=3;
    
    %% B-Spline basis functions
    i=bspline_findspan(theta_limit,param_path);
    N=bspline_basisfunction(theta_limit,i,3,param_path);
    
    %% orientation
    % cumsum sums the wrong way (from i to n while we want from 0 to i) -> double flip 
    B_tilde=flip(cumsum(flip(N,2),2),2);
    
    %Quaternion exponentials
    quatexp_delta_B_tilde=zeros(degree+1,4);
    for j=i-degree+1:i
        quatexp_delta_B_tilde(-i+degree+j+1,:)=...
            myquatexp(param_path.delta(j+1,:) * B_tilde(1,-i+degree+j+1));
    end

    %Function value
    tmp=param_path.rotation(i-degree+1+1,:);
    for j=i-degree+1:i
        tmp = myquatmultiply(tmp, quatexp_delta_B_tilde(-i+degree+j+1,:));
    end
    q=tmp;

    %First derivative
    tmp = repmat(param_path.rotation(i-degree+1+1,:),[degree,1]);
    for k=1:degree
        for j=i-degree+1:i
            tmp(k,:)=myquatmultiply(tmp(k,:),quatexp_delta_B_tilde(-i+degree+j+1,:));
            
            % inner derivative of one element -> always the one that matches 
            if((j-i+degree)==k)
                tmp(k,:)=myquatmultiply(tmp(k,:),param_path.delta(j+1,:)*B_tilde(2,-i+degree+j+1));
            end
        end
    end
    q_p=sum(tmp,1);
    
    if(theta==theta_limit)
        %Second derivative
        tmp1=repmat(param_path.rotation(i-degree+1+1,:),[degree,1]);
        %Terms with 2nd order derivative of B_tilde
        for k=1:degree
            for j=i-degree+1:i
                tmp1(k,:)=myquatmultiply(tmp1(k,:),quatexp_delta_B_tilde(-i+degree+j+1,:));
                if((j-i+degree)==k)
                    tmp1(k,:)=myquatmultiply(tmp1(k,:),param_path.delta(j+1,:)*B_tilde(3,-i+degree+j+1));
                end
            end
        end
        %Terms with two first order derivatives of B_tilde
        tmp2=repmat(param_path.rotation(i-degree+1+1,:),[degree,1,degree]);
        for l=1:degree
            for k=1:degree
                for j=i-degree+1:i
                    tmp2(k,:,l)=myquatmultiply(tmp2(k,:,l),quatexp_delta_B_tilde(-i+degree+j+1,:));
                if((j-i+degree)==l)
                    tmp2(k,:,l)=myquatmultiply(tmp2(k,:,l),param_path.delta(j+1,:)*B_tilde(2,-i+degree+j+1));
                end
                if((j-i+degree)==k)
                    tmp2(k,:,l)=myquatmultiply(tmp2(k,:,l),param_path.delta(j+1,:)*B_tilde(2,-i+degree+j+1));
                end
                end
            end
        end
        q_pp=sum(tmp1,1)+squeeze(sum(sum(tmp2,1),3));
    
    else
        % Interpolate path beyond the ends linearly and keep the orientation
        % constant. Leave q and p_p unchanged.
        q_p=param_path.rotation(1,:)*0;
        q_pp=param_path.rotation(1,:)*0;
    end
    
    %% set the output
    % I can not directly set the derivative of the quaternion... I need its
    % angular velocity which in turn can be used in a rotation matrix
    E_q=[-q(2:4);q(1)*eye(3)-skew(q(2:4))];
    E_q_p=[-q_p(2:4);q_p(1)*eye(3)-skew(q_p(2:4))];
    omega=2*E_q'*q_p';
    omega_p=E_q'*(2*q_pp'-E_q_p*omega);
    
    sigma=zeros(3);
    sigma(1:3,1:3)=quaternion2rotation(q);

    % ca
    dsigma=zeros(3);
    dsigma(1:3,1:3)=skew(omega)*sigma(1:3,1:3);

    ddsigma=zeros(3);
    ddsigma(1:3,1:3)=skew(omega_p)*sigma(1:3,1:3)+skew(omega)*dsigma(1:3,1:3);

    dddsigma=zeros(3);
end

