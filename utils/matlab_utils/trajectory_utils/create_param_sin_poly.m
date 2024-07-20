function traj_struct_out = create_param_sin_poly(traj_struct, param_global, text1, T, text2, phi)
    arguments
        traj_struct struct {mustBeNonempty}
        param_global struct {mustBeNonempty}
        text1 char = 'T'
        T double = 1
        text2 char = 'phi'
        phi double = 0
    end
   
    %% Param sinus poly trajectory
    traj_struct.sin_poly.T     = T; % in s
    traj_struct.sin_poly.omega = 2*pi*1/T; % in rad
    traj_struct.sin_poly.phi   = phi; % in rad
    traj_struct_out = traj_struct;
end