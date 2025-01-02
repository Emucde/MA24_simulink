#ifndef CASADIMPC_HPP
#define CASADIMPC_HPP

#include <vector>
#include <iostream>
#include "mpc_config.h"
#include <memory>

// #define DEBUG 1

class CasadiMPC
{
public:
    // Constructor that accepts parameters for configuration
    CasadiMPC(const std::string &mpc_name);

    // // Method to initialize and run the MPC
    int solve();

    // Method to output the optimal control
    void get_optimal_control(casadi_real *&u_opt_out, int &u_opt_len);

    // Method to get the length of the trajectory data
    uint32_t get_traj_data_len();

    // Destructor
    ~CasadiMPC();

private:
    const std::string mpc_name; // MPC name
    mpc_config_t mpc_config;
    CasadiFunPtr_t casadi_fun;            // MPC Function pointer
    const casadi_real **arg;              // Pointer to arguments
    casadi_real **res;                    // Pointer to results
    casadi_int *iw;                       // Workspace integer
    casadi_real *w;                       // Workspace real
    casadi_real *u_opt;                   // Optimal control result
    casadi_real *w_end;                   // End address for w
    casadi_real *in_init_guess;           // Initial guess in
    casadi_real *out_init_guess;          // Initial guess out
    casadi_real *x_k;                     // Initial state
    casadi_real *y_d;                     // Desired trajectory
    casadi_real *param_weight;            // Parameter weights
    std::streamoff traj_data_startbyte;   // Trajectory data start byte
    uint32_t traj_rows;                   // Trajectory rows (normally 7, xyzquat)
    uint32_t traj_data_total_len;         // Total length of trajectory data
    uint32_t traj_data_real_len;          // Real length of trajectory data
    uint32_t traj_amount;                 // Trajectory amount
    const uint32_t *mpc_traj_indices;     // MPC stepwidth indices for sampling trajectory data
    const uint32_t horizon_len;           // Needed trajectory samples in a prediction horizon.
    const uint32_t init_guess_len;        // Needed trajectory samples in a prediction horizon.
    const std::string traj_file;          // Path to trajectory data file
    const uint32_t traj_data_per_horizon; // Trajectory data per horizon
    const uint32_t n_dof;                 // Number of degrees of freedom
    const uint32_t n_red;                 // Number of reduced degrees of freedom
    int traj_count;                       // Trajectory count
    int traj_select;                      // Trajectory selection
    int mem;                              // Memory

    void read_file(std::ifstream &file, std::streampos data_start, casadi_real *data, int data_len);
    int load_initial_guess(const std::string &init_guess_path, casadi_real *init_guess_data);
    void read_x0_init(const std::string &q0_init_file, casadi_real *x0_arr);
    std::streamoff get_traj_dims();
    void read_trajectory_block();
};

#endif // CASADIMPC_HPP