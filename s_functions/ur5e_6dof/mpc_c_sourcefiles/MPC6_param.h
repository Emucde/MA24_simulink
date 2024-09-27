#ifndef MPC6_PARAM_H
#define MPC6_PARAM_H

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

#define MPC6_INIT_GUESS_PATH "../s_functions/ur5e_6dof/initial_guess/param_MPC6_init_guess.bin"

//MPC_SETTINGS:
#define MPC6_N 5
#define MPC6_N_step 1
#define MPC6_Ts 0.001
#define MPC6_T_horizon 0.005
#define MPC6_rk_iter 1
#define MPC6_variant "nlpsol"
#define MPC6_solver "qrqp"
#define MPC6_version "v3_quat"
#define MPC6_name "MPC6"
#define MPC6_int_method "Euler"
#define MPC6_fixed_parameter 0
#define MPC6_traj_data_per_horizon 6
static const uint32_t MPC6_traj_indices[] = {0,1,2,3,4,5};

//MPC_WEIGHTS:
const casadi_real MPC6_param_weight[182] = {
    /* Q_y : 6x6 matrix values */
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

    /* Q_y_p_ref : 6x6 matrix values */
    1, 0, 0, 0, 0, 0, 
    0, 1, 0, 0, 0, 0, 
    0, 0, 1, 0, 0, 0, 
    0, 0, 0, 1, 0, 0, 
    0, 0, 0, 0, 1, 0, 
    0, 0, 0, 0, 0, 1, 

    /* Q_y_ref : 6x6 matrix values */
    0.25, 0, 0, 0, 0, 0, 
    0, 0.25, 0, 0, 0, 0, 
    0, 0, 0.25, 0, 0, 0, 
    0, 0, 0, 0.25, 0, 0, 
    0, 0, 0, 0, 0.25, 0, 
    0, 0, 0, 0, 0, 0.25, 

    /* epsilon_t : [1 1] array values */
    1e-05, 

    /* epsilon_r : [1 1] array values */
    1e-05, 

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