function s = quat_mult_vec(qq1, qq2)
    q1q2 = quat_mult(qq1, qq2);
        
    s = q1q2(2:4,1);
end
    
    