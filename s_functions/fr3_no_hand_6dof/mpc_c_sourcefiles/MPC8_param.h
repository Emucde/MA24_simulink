#ifndef MPC8_PARAM_H
#define MPC8_PARAM_H

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

#define MPC8_INIT_GUESS_PATH "../s_functions/fr3_no_hand_6dof/initial_guess/param_MPC8_init_guess.bin"

//MPC_SETTINGS:
#define MPC8_N 5
#define MPC8_N_step 5
#define MPC8_Ts 0.005
#define MPC8_T_horizon 0.025
#define MPC8_rk_iter 1
#define MPC8_variant "nlpsol"
#define MPC8_solver "qrqp"
#define MPC8_version "v4_kin_int"
#define MPC8_name "MPC8"
#define MPC8_int_method "Euler"
#define MPC8_fixed_parameter 0
#define MPC8_traj_data_per_horizon 6
static const uint32_t MPC8_traj_indices[] = {0,1,5,10,15,20};
static const uint32_t MPC8_int_times[] = {1.000000e-03,4.000000e-03,5.000000e-03,5.000000e-03,5.000000e-03};

//MPC_WEIGHTS:
const casadi_real MPC8_param_weight[591] = {
    /* Q_y : 6x6 matrix values */
    100, 0, 0, 0, 0, 0, 
    0, 100, 0, 0, 0, 0, 
    0, 0, 100, 0, 0, 0, 
    0, 0, 0, 100, 0, 0, 
    0, 0, 0, 0, 100, 0, 
    0, 0, 0, 0, 0, 100, 

    /* Q_ykp1 : 6x6 matrix values */
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

    /* R_x0 : 14x14 matrix values */
    1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 

    /* R_x : 14x14 matrix values */
    1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-09, 

    /* x_min : [14 1] array values */
    -2.3093, -1.5133, -2.4937, -2.7478, -2.48, 0.8521, -2.6895, -2, -1, -1.5, -1.25, -3, -1.5, -3, 

    /* x_max : [14 1] array values */
    2.3093, 1.5133, 2.4937, -0.4461, 2.48, 4.2094, 2.6895, 2, 1, 1.5, 1.25, 3, 1.5, 3, 

    /* u_min : [7 1] array values */
    -10, -10, -10, -10, -10, -10, -10, 

    /* u_max : [7 1] array values */
    10, 10, 10, 10, 10, 10, 10, 
};


#endif