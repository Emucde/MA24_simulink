#ifndef MPC01_PARAM_H
#define MPC01_PARAM_H

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

#define MPC01_INIT_GUESS_PATH "../s_functions/fr3_no_hand_6dof/initial_guess/param_MPC01_init_guess.bin"

//MPC_SETTINGS:
#define MPC01_N 5
#define MPC01_N_step 5
#define MPC01_Ts 0.005
#define MPC01_T_horizon 0.025
#define MPC01_rk_iter 1
#define MPC01_variant "nlpsol"
#define MPC01_solver "qrqp"
#define MPC01_version "v1"
#define MPC01_name "MPC01"
#define MPC01_int_method "RK4"
#define MPC01_fixed_parameter 0
#define MPC01_traj_data_per_horizon 6
static const uint32_t MPC01_traj_indices[] = {0,1,5,10,15,20};

//MPC_WEIGHTS:
const casadi_real MPC01_param_weight[359] = {
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

    /* R_u : 7x7 matrix values */
    1e-10, 0, 0, 0, 0, 0, 0, 
    0, 1e-10, 0, 0, 0, 0, 0, 
    0, 0, 1e-10, 0, 0, 0, 0, 
    0, 0, 0, 1e-10, 0, 0, 0, 
    0, 0, 0, 0, 1e-10, 0, 0, 
    0, 0, 0, 0, 0, 1e-10, 0, 
    0, 0, 0, 0, 0, 0, 1e-10, 

    /* R_x : 14x14 matrix values */
    1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-10, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-10, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-10, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-10, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-10, 

    /* x_min : [14 1] array values */
    -2.3093, -1.5133, -2.4937, -2.7478, -2.48, 0.8521, -2.6895, -2, -1, -1.5, -1.25, -3, -1.5, -3, 

    /* x_max : [14 1] array values */
    2.3093, 1.5133, 2.4937, -0.4461, 2.48, 4.2094, 2.6895, 2, 1, 1.5, 1.25, 3, 1.5, 3, 

    /* u_min : [7 1] array values */
    -87, -87, -87, -87, -12, -12, -12, 

    /* u_max : [7 1] array values */
    87, 87, 87, 87, 12, 12, 12, 
};


#endif