% wird von nlpsol_opt_problem_SX_v2 aufgerufen und von paramater_7dof.m um den inital guess zu erstellen.

if(strcmp(MPC_version, "v1"))
    opt_problem_y_u_MPC_v1;
elseif(strcmp(MPC_version, "v3_rpy"))
    opt_problem_ineq_feasible_traj_MPC_v3_rpy;
elseif(strcmp(MPC_version, "v3_quat"))
    opt_problem_ineq_feasible_traj_MPC_v3_quat;
else
    error('Only MPC version ( v1 | v3_rpy | v3_quat ) implemented!');
end