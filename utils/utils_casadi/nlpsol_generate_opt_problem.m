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
elseif(strcmp(MPC_version, "v4_kin_int"))
    opt_problem_MPC_v4_kin_int;
elseif(strcmp(MPC_version, "v4_kin_int_refsys"))
    opt_problem_MPC_v4_kin_int_refsys;
elseif(strcmp(MPC_version, "v5_kin_dev"))
    opt_problem_MPC_v5_kin_dev;
elseif(strcmp(MPC_version, "v6_kin_int_path_following"))
    opt_problem_MPC_v6_kin_int_path_following;
elseif(strcmp(MPC_version, "v7_kin_int_planner"))
    opt_problem_MPC_v7_kin_int_planner;
elseif(strcmp(MPC_version, "v8_kin_dev_planner"))
    opt_problem_MPC_v8_kin_dev_planner;
else
    error(['MPC_version ''', MPC_version, ''' not found: Only MPC version (v1 | v3_rpy | v3_quat | v4_kin_int | v4_kin_int_refsys | v5_kin_dev | v6_kin_int_path_following | v7_kin_int_planner | v8_kin_dev_planner)']);
end