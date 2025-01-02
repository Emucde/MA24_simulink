#ifndef CASADIMPC_HPP
#define CASADIMPC_HPP

#include <iostream>
#include "mpc_config.h"
#include "casadi_types.h" // Include for casadi types
#include <vector>
#include "MPC8.h"
#include "MPC8_addressdef.h"
#include "param_robot.h"
#include "MPC01_param.h" // version: 'v1'
#include "MPC6_param.h"  // version: 'v3_quat'
#include "MPC7_param.h"  // version: 'v3_rpy'
#include "MPC8_param.h"  // version: 'v4_kin_int'
#include "MPC9_param.h"  // version: 'v4_kin_int_refsys'
#include "MPC10_param.h" // version: 'v5_kin_dev'
#include "MPC11_param.h" // version: 'v6_kin_int_path_following'
#include "MPC12_param.h" // version: 'v7_kin_int_planner'
#include "MPC13_param.h" // version: 'v8_kin_dev_planner'
#include "MPC14_param.h" // version: 'v6_kin_dev_path_following'

// #define DEBUG 1

class CasadiMPC
{
private:
    const std::string mpc_name; // MPC name
    mpc_config_t mpc_config;

public:
    const uint32_t nq;                    // Number of degrees of freedom
    const uint32_t nx;                    // Number of reduced degrees of freedom
    const uint32_t nq_red;                // Number of reduced degrees of freedom
    const uint32_t nx_red;                // Number of reduced degrees of freedom

    // Constructor that accepts parameters for configuration
    CasadiMPC(const std::string &mpc_name);

    // // Method to initialize and run the MPC
    int solve(casadi_real *x_k_in); // closed loop mpc

    int solve(); // mpc planner: open loop mpc

    ///////////////////////// GETTER METHODS /////////////////////////

    // Method to output the optimal control
    casadi_real *get_optimal_control();

    // Method to output the initial state
    casadi_real * get_x0();

    // Method to get the length of the trajectory data
    uint32_t get_traj_data_len();

    // Method to get n_indices
    const uint32_t *get_n_indices();

    // Method to get n_x_indices
    const uint32_t *get_n_x_indices();

    // Method to get n_indices_fixed
    const uint32_t *get_n_indices_fixed();

    // Method to get n_x_indices_fixed
    const uint32_t *get_n_x_indices_fixed();

    // Method to get x_ref_nq
    const std::vector<casadi_real> &get_x_ref_nq();

    ///////////////////////// SETTER METHODS /////////////////////////

    // Method to set the initial state
    void set_x0(casadi_real *x0_in);

    ///////////////////////// DESTRUCTOR /////////////////////////
    ~CasadiMPC();

private:
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
    const uint32_t *n_indices;            // Indices of reduced degrees of freedom for q
    const uint32_t *n_x_indices;          // Indices of reduced degrees of freedom for x
    const uint32_t *n_indices_fixed;      // Indices of fixed degrees of freedom for q
    const uint32_t *n_x_indices_fixed;    // Indices of fixed degrees of freedom for x
    int traj_count;                       // Trajectory count
    int traj_select;                      // Trajectory selection
    int mem;                              // Memory
    std::vector<casadi_real> x_ref_nq;         // Reference state

    void read_file(std::ifstream &file, std::streampos data_start, casadi_real *data, int data_len);
    int load_initial_guess(const std::string &init_guess_path, casadi_real *init_guess_data);
    void read_x0_init(const std::string &q0_init_file, casadi_real *x0_arr);
    std::streamoff get_traj_dims();
    void read_trajectory_block();
};

#endif // CASADIMPC_HPP