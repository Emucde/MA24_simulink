#ifndef MPC9_PARAM_H
#define MPC9_PARAM_H

#include <stdint.h>
#include <math.h>

#ifndef Inf
#define Inf INFINITY
#endif

#ifndef TRAJ_DATA_PATH
#define TRAJ_DATA_PATH "../s_functions/fr3_no_hand_6dof/trajectory_data/param_traj_data.bin"
#endif

#ifndef X0_INIT_PATH
#define X0_INIT_PATH "../s_functions/fr3_no_hand_6dof/trajectory_data/param_x0_init.bin"
#endif

#define MPC9_INIT_GUESS_PATH "../s_functions/fr3_no_hand_6dof/initial_guess/param_MPC9_init_guess.bin"

//MPC_SETTINGS:
#define MPC9_N 5
#define MPC9_N_step 5
#define MPC9_Ts 0.005
#define MPC9_T_horizon 0.025
#define MPC9_rk_iter 1
#define MPC9_variant "nlpsol"
#define MPC9_solver "qrqp"
#define MPC9_version "v4_kin_int_refsys"
#define MPC9_name "MPC9"
#define MPC9_int_method "Euler"
#define MPC9_fixed_parameter 0
#define MPC9_traj_data_per_horizon 6
static const uint32_t MPC9_traj_indices[] = {0,1,4,9,14,19};

//MPC_WEIGHTS:
const casadi_real MPC9_param_weight[233] = {
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

    /* R_q_pp : 7x7 matrix values */
    1e-10, 0, 0, 0, 0, 0, 0, 
    0, 1e-10, 0, 0, 0, 0, 0, 
    0, 0, 1e-10, 0, 0, 0, 0, 
    0, 0, 0, 1e-10, 0, 0, 0, 
    0, 0, 0, 0, 1e-10, 0, 0, 
    0, 0, 0, 0, 0, 1e-10, 0, 
    0, 0, 0, 0, 0, 0, 1e-10, 

    /* R_v : 7x7 matrix values */
    1e-10, 0, 0, 0, 0, 0, 0, 
    0, 1e-10, 0, 0, 0, 0, 0, 
    0, 0, 1e-10, 0, 0, 0, 0, 
    0, 0, 0, 1e-10, 0, 0, 0, 
    0, 0, 0, 0, 1e-10, 0, 0, 
    0, 0, 0, 0, 0, 1e-10, 0, 
    0, 0, 0, 0, 0, 0, 1e-10, 

    /* lambda_u : [7 1] array values */
    10, 10, 10, 10, 10, 10, 10, 

    /* x_min : [14 1] array values */
    -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, 

    /* x_max : [14 1] array values */
    Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, 

    /* u_min : [7 1] array values */
    -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, 

    /* u_max : [7 1] array values */
    Inf, Inf, Inf, Inf, Inf, Inf, Inf, 

    /* v_min : [7 1] array values */
    -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, 

    /* v_max : [7 1] array values */
    Inf, Inf, Inf, Inf, Inf, Inf, Inf, 
};


#endif