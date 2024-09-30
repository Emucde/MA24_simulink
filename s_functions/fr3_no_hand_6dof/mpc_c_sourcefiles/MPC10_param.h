#ifndef MPC10_PARAM_H
#define MPC10_PARAM_H

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

#define MPC10_INIT_GUESS_PATH "../s_functions/fr3_no_hand_6dof/initial_guess/param_MPC10_init_guess.bin"

//MPC_SETTINGS:
#define MPC10_N 9
#define MPC10_N_step 250
#define MPC10_Ts 0.25
#define MPC10_T_horizon 2.25
#define MPC10_rk_iter 1
#define MPC10_variant "nlpsol"
#define MPC10_solver "qrqp"
#define MPC10_version "v5_kin_dev"
#define MPC10_name "MPC10"
#define MPC10_int_method "RK4"
#define MPC10_fixed_parameter 0
#define MPC10_traj_data_per_horizon 10
static const uint32_t MPC10_traj_indices[] = {0,1,249,499,749,999,1249,1499,1749,1999};

//MPC_WEIGHTS:
const casadi_real MPC10_param_weight[144] = {
    /* Q_y : 6x6 matrix values */
    1, 0, 0, 0, 0, 0, 
    0, 1, 0, 0, 0, 0, 
    0, 0, 1, 0, 0, 0, 
    0, 0, 0, 1, 0, 0, 
    0, 0, 0, 0, 1, 0, 
    0, 0, 0, 0, 0, 1, 

    /* Q_yN : 6x6 matrix values */
    100, 0, 0, 0, 0, 0, 
    0, 100, 0, 0, 0, 0, 
    0, 0, 100, 0, 0, 0, 
    0, 0, 0, 100, 0, 0, 
    0, 0, 0, 0, 100, 0, 
    0, 0, 0, 0, 0, 100, 

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