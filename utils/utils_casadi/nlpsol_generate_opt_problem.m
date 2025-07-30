% nlpsol_generate_opt_problem - Generates the optimization problem for the MPC solver.
% wird von nlpsol_opt_problem_SX_v2 aufgerufen und von paramater_7dof.m um den inital guess zu erstellen.

q_0 = param_traj.q_0(:, traj_select_mpc);
q_0_p = param_traj.q_0_p(:, traj_select_mpc);
q_0_pp = param_traj.q_0_pp(:, traj_select_mpc);

if(strcmp(MPC_version, "opt_problem_y_u_MPC_v1"))
    opt_problem_y_u_MPC_v1;
    kinematic_mpc = 0;
    planner_mpc = 0;
elseif(strcmp(MPC_version, "opt_problem_ineq_feasible_traj_MPC_v3_quat"))
    opt_problem_ineq_feasible_traj_MPC_v3_quat;
    kinematic_mpc = 0;
    planner_mpc = 0;
elseif(strcmp(MPC_version, "opt_problem_ineq_feasible_traj_MPC_v3_rpy"))
    opt_problem_ineq_feasible_traj_MPC_v3_rpy;
    kinematic_mpc = 0;
    planner_mpc = 0;
elseif(strcmp(MPC_version, "opt_problem_MPC_v4_kin_int"))
    opt_problem_MPC_v4_kin_int;
    kinematic_mpc = 1;
    planner_mpc = 0;
elseif(strcmp(MPC_version, "opt_problem_MPC_v4_kin_int_2dof"))
    opt_problem_MPC_v4_kin_int_2dof;
    kinematic_mpc = 1;
    planner_mpc = 0;
elseif(strcmp(MPC_version, "opt_problem_MPC_v4_kin_int_refsys"))
    opt_problem_MPC_v4_kin_int_refsys;
    kinematic_mpc = 1;
    planner_mpc = 0;
elseif(strcmp(MPC_version, "opt_problem_MPC_v5_kin_dev"))
    opt_problem_MPC_v5_kin_dev;
    kinematic_mpc = 1;
    planner_mpc = 0;
elseif(strcmp(MPC_version, "opt_problem_MPC_v6_kin_int_path_following"))
    opt_problem_MPC_v6_kin_int_path_following;
    kinematic_mpc = 1;
    planner_mpc = 0;
elseif(strcmp(MPC_version, "opt_problem_MPC_v6_kin_dev_path_following"))
    opt_problem_MPC_v6_kin_dev_path_following;
    kinematic_mpc = 1;
    planner_mpc = 0;
elseif(strcmp(MPC_version, "opt_problem_MPC_v7_kin_int_planner"))
    opt_problem_MPC_v7_kin_int_planner;
    kinematic_mpc = 1;
    planner_mpc = 1;
elseif(strcmp(MPC_version, "opt_problem_MPC_v8_kin_dev_planner"))
    opt_problem_MPC_v8_kin_dev_planner;
    kinematic_mpc = 1;
    planner_mpc = 1;
elseif(strcmp(MPC_version, "opt_problem_MPC_v9_parametric_kin_int"))
    opt_problem_MPC_v9_parametric_kin_int;
    kinematic_mpc = 1;
    planner_mpc = 0;
elseif(strcmp(MPC_version, "opt_problem_y_u_MPC_v10_parametric"))
    opt_problem_y_u_MPC_v10_parametric;
    kinematic_mpc = 0;
    planner_mpc = 0;
elseif(strcmp(MPC_version, "opt_problem_y_u_MPC_v11_parametric_reduced"))
    opt_problem_y_u_MPC_v11_parametric_reduced;
    kinematic_mpc = 0;
    planner_mpc = 0;
else
    error(['MPC_version ''', MPC_version, ''' not found: Only MPC version (opt_problem_y_u_MPC_v1, opt_problem_ineq_feasible_traj_MPC_v3_rpy, opt_problem_ineq_feasible_traj_MPC_v3_quat, opt_problem_MPC_v4_kin_int, opt_problem_MPC_v4_kin_int_2dof (depr), opt_problem_MPC_v4_kin_int_refsys, opt_problem_MPC_v5_kin_dev, opt_problem_MPC_v6_kin_int_path_following, opt_problem_MPC_v6_kin_dev_path_following, opt_problem_MPC_v7_kin_int_planner, opt_problem_MPC_v8_kin_dev_planner) are available.']);
end