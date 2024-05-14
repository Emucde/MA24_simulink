function epsilon = rotm2quatvec(R)
    q = rotm2quat_v3(R);
    epsilon = q(2:4);
end