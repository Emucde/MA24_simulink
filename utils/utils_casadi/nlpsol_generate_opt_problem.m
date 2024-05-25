% wird von nlpsol_opt_problem_SX_v2 aufgerufen und von paramater_7dof.m um den inital guess zu erstellen.

if(strcmp(MPC_version, "v1"))
    opt_problem_y_u_MPC_v1;
elseif(strcmp(MPC_version, "v2"))
    opt_problem_alldeviations_MPC_v2;
elseif(strcmp(MPC_version, "v3"))
    opt_problem_ineq_feasible_traj_MPC_v3;
elseif(strcmp(MPC_version, "v3_quat"))
    opt_problem_ineq_feasible_traj_MPC_v3_quat;
elseif(strcmp(MPC_version, "v4"))
    opt_problem_ineq_feasible_block_MPC_v4;
elseif(strcmp(MPC_version, "traj_feasible"))
    opt_problem_feasible_trajectory;
else
    error('Only MPC version ( v1 | v2 | v3 | v3_quat | v4 | traj_feasible ) implemented!');
end