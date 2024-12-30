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

    // Destructor
    ~CasadiMPC();

private:
    std::unique_ptr<mpc_config_t> mpc_config;
    CasadiFunPtr_t casadi_fun;          // Function pointer
    const casadi_real **arg;            // Pointer to arguments
    casadi_real **res;                  // Pointer to results
    casadi_int *iw;                     // Workspace integer
    casadi_real *w;                     // Workspace real
    casadi_real *u_opt;                 // Optimal control result
    casadi_real *w_end;                 // End address for w
    casadi_real *init_guess;            // Initial guess
    casadi_real *x_k;                   // Initial state
    casadi_real *y_d;                   // Desired trajectory
    casadi_real *param_weight;          // Parameter weights
    std::streamoff traj_data_startbyte; // Trajectory data start byte
    uint32_t traj_rows;                 // Trajectory rows (normally 7, xyzquat)
    uint32_t traj_data_total_len;       // Total length of trajectory data
    uint32_t traj_data_real_len;        // Real length of trajectory data
    uint32_t traj_amount;               // Trajectory amount
    int traj_count;                     // Trajectory count
    int traj_select;                    // Trajectory selection
    int mem;                            // Memory

    void read_file(std::ifstream &file, std::streampos data_start, casadi_real *data, int data_len);
    int load_initial_guess(const std::string &init_guess_path, casadi_real *init_guess_data);
    std::streamoff get_traj_dims(uint32_t &rows, uint32_t &cols, uint32_t &traj_amount, const std::string &traj_file);
    void read_trajectory_block(const std::string &traj_file, unsigned int traj_data_startbyte, uint32_t rows, uint32_t cols, double *data, const uint32_t *indices);
    void read_x0_init(const std::string &q0_init_file, casadi_real *x0_arr);
};

#endif // CASADIMPC_HPP