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
#define MPC11_N_step 1000
#define MPC11_Ts 1
#define MPC11_T_horizon 5
#define MPC11_rk_iter 1
#define MPC11_variant "nlpsol"
#define MPC11_solver "qrqp"
#define MPC11_version "v4_kin_int_refsys"
#define MPC11_name "MPC9"
#define MPC11_int_method "Euler"
#define MPC11_fixed_parameter 0
#define MPC11_traj_data_per_horizon 6
static const uint32_t MPC11_traj_indices[] = {0,1,999,1999,2999,3999};

//MPC_WEIGHTS:
const casadi_real MPC11_param_weight[166] = {
    /* Q_y : 6x6 matrix values */
    1, 0, 0, 0, 0, 0, 
    0, 1, 0, 0, 0, 0, 
    0, 0, 1, 0, 0, 0, 
    0, 0, 0, 1, 0, 0, 
    0, 0, 0, 0, 1, 0, 
    0, 0, 0, 0, 0, 1, 

    /* Q_yN : 6x6 matrix values */
    1000, 0, 0, 0, 0, 0, 
    0, 1000, 0, 0, 0, 0, 
    0, 0, 1000, 0, 0, 0, 
    0, 0, 0, 1000, 0, 0, 
    0, 0, 0, 0, 1000, 0, 
    0, 0, 0, 0, 0, 1000, 

    /* Q_theta : [1 1] array values */
    100, 

    /* Q_thetaN : [1 1] array values */
    100000, 

    /* lambda_theta : [1 1] array values */
    1, 

    /* R_q_pp : 7x7 matrix values */
    1e-10, 0, 0, 0, 0, 0, 0, 
    0, 1e-10, 0, 0, 0, 0, 0, 
    0, 0, 1e-10, 0, 0, 0, 0, 
    0, 0, 0, 1e-10, 0, 0, 0, 
    0, 0, 0, 0, 1e-10, 0, 0, 
    0, 0, 0, 0, 0, 1e-10, 0, 
    0, 0, 0, 0, 0, 0, 1e-10, 

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