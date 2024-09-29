#ifndef MPC8_PARAM_H
#define MPC8_PARAM_H

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

#define MPC8_INIT_GUESS_PATH "../s_functions/fr3_6dof/initial_guess/param_MPC8_init_guess.bin"

//MPC_SETTINGS:
#define MPC8_N 5
#define MPC8_N_step 10
#define MPC8_Ts 0.01
#define MPC8_T_horizon 0.05
#define MPC8_rk_iter 1
#define MPC8_variant "nlpsol"
#define MPC8_solver "qrqp"
#define MPC8_version "v4_kin_int"
#define MPC8_name "MPC8"
#define MPC8_int_method "Euler"
#define MPC8_fixed_parameter 0
#define MPC8_traj_data_per_horizon 6
static const uint32_t MPC8_traj_indices[] = {0,1,2,9,19,29};

//MPC_WEIGHTS:
const casadi_real MPC8_param_weight[180] = {
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