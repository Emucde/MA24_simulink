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
    mpc_config_t const* mpc_config;
    robot_config_t &robot_config;

public:
    const bool is_kinematic_mpc; // Kinematic MPC flag
    const casadi_uint nq;        // Number of degrees of freedom
    const casadi_uint nx;        // Number of reduced degrees of freedom
    const casadi_uint nq_red;    // Number of reduced degrees of freedom
    const casadi_uint nx_red;    // Number of reduced degrees of freedom

    // Constructor that accepts parameters for configuration
    CasadiMPC(const std::string &mpc_name, robot_config_t &robot_config);

    // // Method to run the MPC
    int solve();                    // closed loop mpc without copying
    int solve(casadi_real *x_k_in); // closed loop mpc with copying x_k_in to x_k

    int solve_planner(); // mpc planner: open loop mpc

    // Method to switch the trajectory
    void switch_traj(casadi_uint traj_sel);

    // Method to read the next trajectory block
    void read_trajectory_block();

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////// GETTER METHODS ////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////

    // Method to output the optimal control
    casadi_real *get_optimal_control()
    {
        return u_opt;
    }

    // Method to get the initial state address at the begin of the prediction horizon
    casadi_real *get_x_k()
    {
        return x_k;
    }

    // Method to get the desired trajectory address at the begin of the prediction horizon
    casadi_real *get_y_d()
    {
        return y_d;
    }

    //Method to get the workspace real pointer
    casadi_real *get_w()
    {
        return w;
    }

    // Method to get the length of the trajectory data per prediction horizon
    casadi_uint get_traj_data_per_horizon_len()
    {
        return traj_data_per_horizon;
    }

    // Method to get the length of the trajectory data
    casadi_uint get_traj_data_len()
    {
        return traj_data_real_len;
    }

    // Method to get n_indices
    const casadi_uint *get_n_indices()
    {
        return n_indices;
    }

    // Method to get n_x_indices
    const casadi_uint *get_n_x_indices()
    {
        return n_x_indices;
    }

    // Method to get n_indices_fixed
    const casadi_uint *get_n_indices_fixed()
    {
        return n_indices_fixed;
    }

    // Method to get n_x_indices_fixed
    const casadi_uint *get_n_x_indices_fixed()
    {
        return n_x_indices_fixed;
    }

    // Method to get x_ref_nq (it is x0, the initial state of each trajectory)
    const std::vector<casadi_real> &get_x_ref_nq()
    {
        return x_ref_nq;
    }

    // Method to get the control sampling time dt
    casadi_real get_dt()
    {
        return dt;
    }

    // Method to get the mpc_config
    mpc_config_t const* get_mpc_config()
    {
        return mpc_config;
    }

    // Method to get mpc_traj_indices
    const casadi_uint *get_mpc_traj_indices()
    {
        return mpc_traj_indices;
    }

    // Method to get the trajectory file path
    const std::string get_traj_file()
    {
        return traj_file;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////// SETTER METHODS ////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////

    void set_x0(casadi_real *x0_in);

    ///////////////////////// DESTRUCTOR /////////////////////////
    ~CasadiMPC();

private:
    CasadiFunPtr_t casadi_fun;               // MPC Function pointer
    const casadi_real **arg;                 // Pointer to arguments
    casadi_real **res;                       // Pointer to results
    casadi_int *iw;                          // Workspace integer
    casadi_real *w;                          // Workspace real
    casadi_real *u_opt;                      // Optimal control result
    casadi_real *w_end;                      // End address for w
    casadi_real *in_init_guess;              // Initial guess in
    casadi_real *out_init_guess;             // Initial guess out
    casadi_real *x_k;                        // Initial state
    casadi_real *y_d;                        // Desired trajectory
    casadi_real *param_weight;               // Parameter weights
    std::streamoff traj_data_startbyte;      // Trajectory data start byte
    casadi_uint traj_rows;                   // Trajectory rows (normally 7, xyzquat)
    casadi_uint traj_data_total_len;         // Total length of trajectory data
    casadi_uint traj_data_real_len;          // Real length of trajectory data
    casadi_uint traj_amount;                 // Trajectory amount
    const casadi_uint *mpc_traj_indices;     // MPC stepwidth indices for sampling trajectory data
    const casadi_uint horizon_len;           // Needed trajectory samples in a prediction horizon.
    const casadi_uint init_guess_len;        // Needed trajectory samples in a prediction horizon.
    const std::string traj_file;             // Path to trajectory data file
    const casadi_uint traj_data_per_horizon; // Trajectory data per horizon
    const casadi_uint *n_indices;            // Indices of reduced degrees of freedom for q
    const casadi_uint *n_x_indices;          // Indices of reduced degrees of freedom for x
    const casadi_uint *n_indices_fixed;      // Indices of fixed degrees of freedom for q
    const casadi_uint *n_x_indices_fixed;    // Indices of fixed degrees of freedom for x
    casadi_uint traj_count;                  // Trajectory count
    int traj_select;                         // Trajectory selection
    int mem;                                 // Memory
    casadi_real dt;                          // Control sampling time
    std::vector<casadi_real> x_ref_nq;       // Reference state

    void read_file(std::ifstream &file, std::streampos data_start, casadi_real *data, int data_len);
    int load_initial_guess(const std::string &init_guess_path, casadi_real *init_guess_data);
    void read_x0_init(const std::string &q0_init_file, casadi_real *x0_arr);
    std::streamoff get_traj_dims();
};

#endif // CASADIMPC_HPP