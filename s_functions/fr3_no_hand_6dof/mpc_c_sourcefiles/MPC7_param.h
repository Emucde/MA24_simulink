#ifndef MPC7_PARAM_H
#define MPC7_PARAM_H

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

#define MPC7_INIT_GUESS_PATH "../s_functions/fr3_no_hand_6dof/initial_guess/param_MPC7_init_guess.bin"

//MPC_SETTINGS:
#define MPC7_N 5
#define MPC7_N_step 5
#define MPC7_Ts 0.005
#define MPC7_T_horizon 0.025
#define MPC7_rk_iter 1
#define MPC7_variant "nlpsol"
#define MPC7_solver "qrqp"
#define MPC7_version "v3_rpy"
#define MPC7_name "MPC7"
#define MPC7_int_method "Euler"
#define MPC7_fixed_parameter 0
#define MPC7_traj_data_per_horizon 6
static const uint32_t MPC7_traj_indices[] = {0,1,5,10,15,20};

//MPC_WEIGHTS:
const casadi_real MPC7_param_weight[201] = {
    /* Q_y : 6x6 matrix values */
    1000, 0, 0, 0, 0, 0, 
    0, 1000, 0, 0, 0, 0, 
    0, 0, 1000, 0, 0, 0, 
    0, 0, 0, 100, 0, 0, 
    0, 0, 0, 0, 100, 0, 
    0, 0, 0, 0, 0, 100, 

    /* R_q_pp : 7x7 matrix values */
    1e-10, 0, 0, 0, 0, 0, 0, 
    0, 1e-10, 0, 0, 0, 0, 0, 
    0, 0, 1e-10, 0, 0, 0, 0, 
    0, 0, 0, 1e-10, 0, 0, 0, 
    0, 0, 0, 0, 1e-10, 0, 0, 
    0, 0, 0, 0, 0, 1e-10, 0, 
    0, 0, 0, 0, 0, 0, 1e-10, 

    /* Q_y_p_ref : 6x6 matrix values */
    100, 0, 0, 0, 0, 0, 
    0, 100, 0, 0, 0, 0, 
    0, 0, 100, 0, 0, 0, 
    0, 0, 0, 10, 0, 0, 
    0, 0, 0, 0, 10, 0, 
    0, 0, 0, 0, 0, 10, 

    /* Q_y_ref : 6x6 matrix values */
    100, 0, 0, 0, 0, 0, 
    0, 100, 0, 0, 0, 0, 
    0, 0, 100, 0, 0, 0, 
    0, 0, 0, 10, 0, 0, 
    0, 0, 0, 0, 10, 0, 
    0, 0, 0, 0, 0, 10, 

    /* epsilon_t : [1 1] array values */
    1e-05, 

    /* epsilon_r : [1 1] array values */
    1e-05, 

    /* x_min : [14 1] array values */
    -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, 

    /* x_max : [14 1] array values */
    Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, 

    /* u_min : [7 1] array values */
    -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, 

    /* u_max : [7 1] array values */
    Inf, Inf, Inf, Inf, Inf, Inf, Inf, 
};


#endif