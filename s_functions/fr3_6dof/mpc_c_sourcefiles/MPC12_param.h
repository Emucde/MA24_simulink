#ifndef MPC12_PARAM_H
#define MPC12_PARAM_H

#include <stdint.h>
#include <math.h>

#ifndef Inf
#define Inf INFINITY
#endif

#ifndef TRAJ_DATA_PATH
#define TRAJ_DATA_PATH "../s_functions/fr3_6dof/trajectory_data/param_traj_data.bin"
#endif

#ifndef X0_INIT_PATH
#define X0_INIT_PATH "../s_functions/fr3_6dof/trajectory_data/param_x0_init.bin"
#endif

#define MPC12_INIT_GUESS_PATH "../s_functions/fr3_6dof/initial_guess/param_MPC12_init_guess.bin"

//MPC_SETTINGS:
#define MPC12_N 5
#define MPC12_N_step 250
#define MPC12_Ts 0.25
#define MPC12_T_horizon 1.25
#define MPC12_rk_iter 1
#define MPC12_variant "nlpsol"
#define MPC12_solver "qrqp"
#define MPC12_version "v7_kin_int_planner"
#define MPC12_name "MPC12"
#define MPC12_int_method "Euler"
#define MPC12_fixed_parameter 0
#define MPC12_traj_data_per_horizon 6
static const uint32_t MPC12_traj_indices[] = {0,1,2,249,499,749};

//MPC_WEIGHTS:
const casadi_real MPC12_param_weight[144] = {
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