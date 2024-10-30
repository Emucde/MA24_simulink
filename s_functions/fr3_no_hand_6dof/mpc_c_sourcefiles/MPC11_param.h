#ifndef MPC11_PARAM_H
#define MPC11_PARAM_H

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

#define MPC11_INIT_GUESS_PATH "../s_functions/fr3_no_hand_6dof/initial_guess/param_MPC11_init_guess.bin"

//MPC_SETTINGS:
#define MPC11_N 5
#define MPC11_N_step 10
#define MPC11_Ts 0.01
#define MPC11_T_horizon 0.05
#define MPC11_rk_iter 1
#define MPC11_variant "nlpsol"
#define MPC11_solver "qrqp"
#define MPC11_version "v6_kin_int_path_following"
#define MPC11_name "MPC11"
#define MPC11_int_method "Euler"
#define MPC11_fixed_parameter 0
#define MPC11_traj_data_per_horizon 6
static const uint32_t MPC11_traj_indices[] = {0,1,10,20,30,40};

//MPC_WEIGHTS:
const casadi_real MPC11_param_weight[363] = {
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

    /* Q_theta : [1 1] array values */
    100, 

    /* Q_thetaN : [1 1] array values */
    100000, 

    /* lambda_theta : [1 1] array values */
    1, 

    /* R_q_pp : 7x7 matrix values */
    0.01, 0, 0, 0, 0, 0, 0, 
    0, 0.01, 0, 0, 0, 0, 0, 
    0, 0, 0.01, 0, 0, 0, 0, 
    0, 0, 0, 0.01, 0, 0, 0, 
    0, 0, 0, 0, 0.01, 0, 0, 
    0, 0, 0, 0, 0, 0.01, 0, 
    0, 0, 0, 0, 0, 0, 0.01, 

    /* R_x : 14x14 matrix values */
    1e-05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 1e-05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 1e-05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 1e-05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 1e-05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 1e-05, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 1e-05, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 1e-05, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 1e-05, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-05, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-05, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-05, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-05, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-05, 

    /* R_theta_prev : [1 1] array values */
    1e-05, 

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