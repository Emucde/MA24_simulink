%% trajectory full data bus init (already defined in bus.mat)
bus_temp_struct            = struct;
bus_temp_struct.N              = param_traj_data.N;
bus_temp_struct.t              = param_traj_data.t;
bus_temp_struct.p_d            = param_traj_data.p_d;
bus_temp_struct.p_d_p          = param_traj_data.p_d_p;
bus_temp_struct.p_d_pp         = param_traj_data.p_d_pp;
bus_temp_struct.Phi_d          = param_traj_data.Phi_d;
bus_temp_struct.Phi_d_p        = param_traj_data.Phi_d_p;
bus_temp_struct.Phi_d_pp       = param_traj_data.Phi_d_pp;
bus_temp_struct.R_d            = param_traj_data.R_d;
bus_temp_struct.q_d            = param_traj_data.q_d;
bus_temp_struct.q_d_p          = param_traj_data.q_d;
bus_temp_struct.q_d_pp         = param_traj_data.q_d_pp;
bus_temp_struct.omega_d        = param_traj_data.omega_d;
bus_temp_struct.omega_d_p      = param_traj_data.omega_d_p;
bus_temp_struct.alpha_d        = param_traj_data.alpha_d;
bus_temp_struct.alpha_d_p      = param_traj_data.alpha_d_p;
bus_temp_struct.alpha_d_pp     = param_traj_data.alpha_d_pp;
bus_temp_struct.rot_ax_d       = param_traj_data.rot_ax_d;
bus_temp_struct.alpha_d_offset = param_traj_data.alpha_d_offset;
bus_temp_struct.q_d_rel        = param_traj_data.q_d_rel;

traj_data_bus_init = bus_temp_struct;

traj_data_bus = eval(Simulink.Bus.createObject(bus_temp_struct).busName);
clear -regexp slBus; clear bus_temp_struct;