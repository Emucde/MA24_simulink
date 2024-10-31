function q = rotm2quat_v4_casadi(R)
    % ROTM2QUAT_V4 Convert rotation matrix to quaternion
    %
    % This function converts a 3x3 rotation matrix R into a 4x1 quaternion q
    % using an accurate and numerically stable method.
    %
    % Inputs:
    %   R - 3x3 rotation matrix
    %
    % Outputs:
    %   q - 4x1 quaternion [w; x; y; z]
    %
    % Algorithm:
    %   The function uses a method that avoids numerical instabilities
    %   and singularities by computing all four components of the quaternion
    %   and selecting the most accurate one based on the trace of the matrix.
    %
    % Reference:
    %   Sarabandi, S., & Thomas, F. (2018). Accurate computation of quaternions 
    %   from rotation matrices. In International Symposium on Advances in Robot 
    %   Kinematics. Springer International Publishing.
        import casadi.*;

        x = SX.sym('x', 1, 1);
        my_sign = Function('mysign', {x}, {if_else(x >= 0, 1, -1)});
    
        eta = 0;
    
        % Extract the elements of the rotation matrix
        r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
        r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
        r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);
    
        s1 =  r11 + r22 + r33;
        s2 =  r11 - r22 - r33;
        s3 = -r11 + r22 - r33;
        s4 = -r11 - r22 + r33;

        q1_true = sqrt( 1 + s1 ) / 2;
        q1_false = sqrt(  ((r32 - r23)^2 + (r13 - r31)^2 + (r21 - r12)^2) / (3 - r11 - r22 - r33)  ) / 2;

        q1 = if_else(s1 > eta, q1_true, q1_false);
    
        q2_true = sqrt( 1 + s2 ) / 2;
        q2_false = sqrt(  ((r32 - r23)^2 + (r12 + r21)^2 + (r31 + r13)^2) / (3 - r11 + r22 + r33)  ) / 2;

        q2 = if_else(s2 > eta, q2_true, q2_false);
    
        q3_true = sqrt( 1 + s3 ) / 2;
        q3_false = sqrt(  ((r13 - r31)^2 + (r12 + r21)^2 + (r23 + r32)^2) / (3 + r11 - r22 + r33)  ) / 2;

        q3 = if_else(s3 > eta, q3_true, q3_false);
    
        q4_true = sqrt( 1 + s4 ) / 2;
        q4_false = sqrt(  ((r21 - r12)^2 + (r31 + r13)^2 + (r32 + r23)^2) / (3 + r11 + r22 - r33)  ) / 2;
    
        q4 = if_else(s4 > eta, q4_true, q4_false);

        % Assemble the quaternion
        q = [q1; my_sign(r32 - r23) * q2; my_sign(r13 - r31) * q3; my_sign(r21 - r12) * q4];
    end