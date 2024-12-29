#ifndef MPC_CONFIG_H
#define MPC_CONFIG_H

#include <stdint.h>

typedef struct {
    const char* x0_init_path;
    const char* init_guess_path;
    const char* traj_data_path;
    const uint32_t traj_data_per_horizon;
    uint32_t y_d_len;
    uint32_t init_guess_len;
    uint32_t x_k_addr;
    uint32_t y_d_addr;
    uint32_t in_init_guess_addr;
    uint32_t in_param_weight_addr;
    const casadi_real* param_weight;
} mpc_config_t;

#endif // MPC_CONFIG_H
