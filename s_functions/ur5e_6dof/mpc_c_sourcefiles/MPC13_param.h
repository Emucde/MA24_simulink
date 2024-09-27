#ifndef MPC13_PARAM_H
#define MPC13_PARAM_H

#include <stdint.h>
#include <math.h>

#ifndef Inf
#define Inf INFINITY
#endif

#ifndef TRAJ_DATA_PATH
#define TRAJ_DATA_PATH "../s_functions/ur5e_6dof/trajectory_data/param_traj_data.bin"
#endif

#ifndef X0_INIT_PATH
#define X0_INIT_PATH "../s_functions/ur5e_6dof/trajectory_data/param_x0_init.bin"
#endif

#define MPC13_INIT_GUESS_PATH "../s_functions/ur5e_6dof/initial_guess/param_MPC13_init_guess.bin"

//MPC_SETTINGS:
#define MPC13_N 5
#define MPC13_N_step 250
#define MPC13_Ts 0.25
#define MPC13_T_horizon 1.25
#define MPC13_rk_iter 1
#define MPC13_variant "nlpsol"
#define MPC13_solver "qrqp"
#define MPC13_version "v8_kin_dev_planner"
#define MPC13_name "MPC13"
#define MPC13_int_method "Euler"
#define MPC13_fixed_parameter 0
#define MPC13_traj_data_per_horizon 6
static const uint32_t MPC13_traj_indices[] = {0,1,2,250,500,750};

//MPC_WEIGHTS:
const casadi_real MPC13_param_weight[144] = {
    /* Q_y : 6x6 matrix values */
    100, 0, 0, 0, 0, 0, 
    0, 100, 0, 0, 0, 0, 
    0, 0, 100, 0, 0, 0, 
    0, 0, 0, 100, 0, 0, 
    0, 0, 0, 0, 100, 0, 
    0, 0, 0, 0, 0, 100, 

    /* Q_yN : 6x6 matrix values */
    100000, 0, 0, 0, 0, 0, 
    0, 100000, 0, 0, 0, 0, 
    0, 0, 100000, 0, 0, 0, 
    0, 0, 0, 100000, 0, 0, 
    0, 0, 0, 0, 100000, 0, 
    0, 0, 0, 0, 0, 100000, 

    /* R_q_pp : 6x6 matrix values */
    1e-10, 0, 0, 0, 0, 0, 
    0, 1e-10, 0, 0, 0, 0, 
    0, 0, 1e-10, 0, 0, 0, 
    0, 0, 0, 1e-10, 0, 0, 
    0, 0, 0, 0, 1e-10, 0, 
    0, 0, 0, 0, 0, 1e-10, 

    /* x_min : [12 1] array values */
    -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, 

    /* x_max : [12 1] array values */
    Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, 

    /* u_min : [6 1] array values */
    -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, 

    /* u_max : [6 1] array values */
    Inf, Inf, Inf, Inf, Inf, Inf, 
};


#endif