#ifndef CASADIMPC_HPP
#define CASADIMPC_HPP

#include <iostream>
#include "mpc_config.h"
#include "casadi_types.h" // Include for casadi types
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
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
    robot_config_t &robot_config;
    const Eigen::MatrixXd *traj_data; // Trajectory data
    casadi_uint traj_data_real_len;   // Real length of the singular trajectory data without additional samples for last prediction horizon

public:
    const bool is_kinematic_mpc; // Kinematic MPC flag
    const casadi_uint nq;        // Number of degrees of freedom
    const casadi_uint nx;        // Number of reduced degrees of freedom
    const casadi_uint nq_red;    // Number of reduced degrees of freedom
    const casadi_uint nx_red;    // Number of reduced degrees of freedom

private:
    CasadiFunPtr_t casadi_fun;                // MPC Function pointer
    std::vector<CasadiIOPtr_t> casadi_io_fun; // MPC Reference Function pointer list
    const casadi_real **arg;                  // Pointer to arguments
    casadi_real **res;                        // Pointer to results
    casadi_int *iw;                           // Workspace integer
    casadi_real *w;                           // Workspace real
    casadi_real *u_opt;                       // Optimal control result
    casadi_real *w_end;                       // End address for w
    std::streamoff traj_data_startbyte;       // Trajectory data start byte
    casadi_uint traj_rows;                    // Trajectory rows (normally 7, xyzquat)
    casadi_uint traj_cols;                    // Total length of trajectory data (transient traj + singular traj)
    const casadi_uint horizon_len;            // Needed trajectory samples in a prediction horizon.
    const Eigen::VectorXi mpc_traj_indices;   // MPC stepwidth indices for sampling trajectory data
    const std::string traj_file;              // Path to trajectory data file
    const casadi_uint traj_data_per_horizon;  // Trajectory data per horizon
    casadi_uint traj_count;                   // Trajectory count
    int traj_select;                          // Trajectory selection
    int mem;                                  // Memory
    casadi_real dt;                           // Control sampling time

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////// PUBLIC METHODS ////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    /////                                                                             /////
    /////                ||||||   ||   ||  ||||    ||      ||    |||||                /////
    /////                ||   ||  ||   ||  ||  ||  ||      ||  ||                     /////
    /////                ||||||   ||   ||  ||||||  ||      ||  ||                     /////
    /////                ||       ||   ||  ||  ||  ||      ||  ||                     /////
    /////                ||        |||||   ||||    ||||||  ||    |||||                /////
    /////                                                                             /////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////

public:
    // Constructor that accepts parameters for configuration
    CasadiMPC(const std::string &mpc_name,
              robot_config_t &robot_config,
              const Eigen::MatrixXd *traj_data,
              const casadi_uint traj_real_len);

    // Method to run the MPC
    int solve(casadi_real *x_k_in); // closed loop mpc with copying x_k_in to x_k

    int solve_planner(); // mpc planner: open loop mpc

    // Method to switch the trajectory
    void switch_traj(const Eigen::MatrixXd *traj_data_new, const casadi_real *const x_k_ptr, casadi_uint traj_data_real_len_new);

    // if it solves too slow.
    void increase_traj_count()
    {
        traj_count++;
    }

    // Method to read the next trajectory block
    // void read_trajectory_block();

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////// PUBLIC GETTER METHODS //////////////////////////////////
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
        return mpc_config.in.x_k.ptr;
    }

    // Method to get the desired trajectory address at the begin of the prediction horizon
    casadi_real *get_y_d()
    {
        return mpc_config.in.y_d.ptr;
    }

    // Method to get the workspace real pointer
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
    casadi_uint get_traj_length()
    {
        return traj_cols * traj_rows;
    }

    // Method to get the control sampling time dt
    casadi_real get_dt()
    {
        return dt;
    }

    // Method to get the mpc_config
    mpc_config_t *get_mpc_config()
    {
        return &mpc_config;
    }

    // Method to get mpc_traj_indices
    const casadi_uint *get_mpc_traj_indices()
    {
        return (const casadi_uint *)mpc_traj_indices.data();
    }

    // Method to get the trajectory file path
    const std::string &get_traj_file()
    {
        return traj_file;
    }

    // Method to get the current trajectory data
    const casadi_real *get_act_traj_data()
    {
        return traj_data->col(traj_count).data();
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////// PUBLIC SETTER METHODS ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////// DESTRUCTOR /////////////////////////
    ~CasadiMPC();

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////// PRIVATE METHODS ///////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    /////                                                                             /////
    /////          ||||||   ||||||   ||  ||    ||   ||||    ||||||||  ||||||          /////
    /////          ||   ||  ||   ||  ||  ||    ||  ||   ||     ||     ||              /////
    /////          ||||||   ||||||   ||   ||  ||   |||||||     ||     ||||||          /////
    /////          ||       || ||    ||    ||||    ||   ||     ||     ||              /////
    /////          ||       ||   ||  ||     ||     ||   ||     ||     ||||||          /////
    /////                                                                             /////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////

private:
    // void read_file(std::ifstream &file, std::streampos data_start, casadi_real *data, int data_len);
    // int load_initial_guess(const std::string &init_guess_path, casadi_real *init_guess_data);
    // void read_x0_init(const std::string &q0_init_file, casadi_real *x0_arr);
    // std::streamoff get_traj_dims();

    void set_row_vector(casadi_real *matrix_data, casadi_real *row_data, casadi_uint rows, casadi_uint length);
    void set_references(casadi_real *x_k_in);

    void set_coldstart_init_guess(const casadi_real *const x_k_ptr);

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////// PRIVATE GETTER METHODS /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////// PRIVATE SETTER METHODS /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////

    void set_x_k(const casadi_real *x_k_in)
    {
        memcpy(mpc_config.in.x_k.ptr, x_k_in, nx_red * sizeof(casadi_real));
    }
};

#endif // CASADIMPC_HPP