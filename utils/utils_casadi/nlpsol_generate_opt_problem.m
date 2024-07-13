% wird von nlpsol_opt_problem_SX_v2 aufgerufen und von paramater_7dof.m um den inital guess zu erstellen.

q_0 = param_traj.q_0(:, traj_select_mpc);
q_0_p = param_traj.q_0_p(:, traj_select_mpc);
q_0_pp = param_traj.q_0_pp(:, traj_select_mpc);

if(strcmp(MPC_version, "v1"))
    opt_problem_y_u_MPC_v1;
elseif(strcmp(MPC_version, "v3_rpy"))
    opt_problem_ineq_feasible_traj_MPC_v3_rpy;
elseif(strcmp(MPC_version, "v3_quat"))
    opt_problem_ineq_feasible_traj_MPC_v3_quat;
elseif(strcmp(MPC_version, "v4_kin_thelen"))
    opt_problem_MPC_v4_kin_int;
elseif(strcmp(MPC_version, "v4_kin_ref_dev"))
    opt_problem_MPC_v5_kinmpc_dev_refsys;
else
    error('Only MPC version ( v1 | v3_rpy | v3_quat | v4_kin) implemented!');
end