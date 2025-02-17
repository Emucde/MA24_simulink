#ifndef CASADIMPC_HPP
#define CASADIMPC_HPP

#include "json.hpp"
#include <iostream>
#include "mpc_config_types.h"
#include "casadi_types.h" // Include for casadi types
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "param_robot.h"
#include "mpc_configs.h"
#include "TrajectoryGenerator.hpp"
#include "FullSystemTorqueMapper.hpp"

// #define DEBUG 1

class CasadiMPC
{
private:
    const std::string mpc_name; // MPC name
    mpc_config_t mpc_config;
    robot_config_t &robot_config;
    TrajectoryGenerator &trajectory_generator;    // Trajectory generator
    const Eigen::MatrixXd *traj_data;        // Trajectory data
    casadi_uint traj_data_real_len;          // Real length of the singular trajectory data without additional samples for last prediction horizon
public:
    bool is_kinematic_mpc; // Kinematic MPC flag
    const bool is_planner_mpc; // Kinematic MPC flag
    const casadi_uint nq;        // Number of degrees of freedom
    const casadi_uint nx;        // Number of reduced degrees of freedom
    const casadi_uint nq_red;    // Number of reduced degrees of freedom
    const casadi_uint nx_red;    // Number of reduced degrees of freedom

private:
    CasadiFunPtr_t casadi_fun;                                                                  // MPC Function pointer
    const casadi_real **arg;                                                                    // Pointer to arguments
    casadi_real **res;                                                                          // Pointer to results
    casadi_int *iw;                                                                             // Workspace integer
    casadi_real *w;                                                                             // Workspace real
    casadi_real *w_end;                                                                         // End address for w
    std::streamoff traj_data_startbyte;                                                         // Trajectory data start byte
    casadi_uint traj_rows;                                                                      // Trajectory rows (normally 7, xyzquat)
    casadi_uint traj_cols;                                                                      // Total length of trajectory data (transient traj + singular traj)        Eigen::VectorXi selected_rows(7);

    const casadi_uint horizon_len;           // Needed trajectory samples in a prediction horizon.
    Eigen::VectorXi mpc_traj_indices;        // MPC stepwidth indices for sampling trajectory data
    casadi_uint N_step;
    const std::string traj_file;             // Path to trajectory data file
    const casadi_uint traj_data_per_horizon; // Trajectory data per horizon
    casadi_uint traj_count;                  // Trajectory count
    int traj_select;                         // Trajectory selection
    int mem;                                 // Memory
    casadi_real dt;                          // Control sampling time

    const Eigen::VectorXi y_d_quat_rows = (Eigen::VectorXi(7) << trajectory_generator.p_d_rows, trajectory_generator.q_d_rows).finished(); // Selecting p_d (0-2) and q_d (9-12)
    const Eigen::VectorXi y_d_p_quat_rows = (Eigen::VectorXi(6) << trajectory_generator.p_d_p_rows, trajectory_generator.omega_d_rows).finished();  // Selecting p_d_p (3-5) and omega_d (13-15)
    const Eigen::VectorXi y_d_pp_quat_rows = (Eigen::VectorXi(6) << trajectory_generator.p_d_pp_rows, trajectory_generator.omega_d_p_rows).finished(); // Selecting p_d_pp (6-8) and omega_d_p (16-18)
    
    const Eigen::VectorXi y_d_rpy_rows = (Eigen::VectorXi(6) << trajectory_generator.p_d_rows, trajectory_generator.phi_d_rows).finished(); // Selecting p_d (0-2) and Phi_d (19-21)
    const Eigen::VectorXi y_d_p_rpy_rows = (Eigen::VectorXi(6) << trajectory_generator.p_d_p_rows, trajectory_generator.phi_d_p_rows).finished();  // Selecting p_d_p (3-5) and Phi_d_p (22-24)
    const Eigen::VectorXi y_d_pp_rpy_rows = (Eigen::VectorXi(6) << trajectory_generator.p_d_pp_rows, trajectory_generator.phi_d_pp_rows).finished(); // Selecting p_d_pp (6-8) and Phi_d_pp (25-27)
    

    const double* x_k_ptr = 0; // - Initial state (12 x 1);
    const double* t_k_ptr = 0; // - Time (1 x 1)
    const double* y_d_ptr = 0; // - Desired trajectory (7 x horizon_len)
    const double* y_d_p_ptr = 0; // - Desired trajectory derivative (6 x horizon_len)
    const double* y_d_pp_ptr = 0; // - Desired trajectory second derivative (6 x horizon_len)
    std::vector<const double **> mpc_data;      // - Array of pointers to the data
    std::vector<CasadiSetPtr_t> mpc_set_funcs; // - Array of pointers to the functions

    bool use_quat;

    std::vector<Eigen::MatrixXd> y_d_blocks;
    std::vector<Eigen::MatrixXd> y_d_p_blocks;
    std::vector<Eigen::MatrixXd> y_d_pp_blocks;
    Eigen::VectorXd t_d_arr;

    std::vector<double *> y_d_blocks_data;
    std::vector<double *> y_d_p_blocks_data;
    std::vector<double *> y_d_pp_blocks_data;

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
    CasadiMPC(CasadiMPCType mpc,
              robot_config_t &robot_config,
              TrajectoryGenerator &trajectory_generator);

    // Method to run the MPC
    int solve(const casadi_real *const x_k_in); // closed loop mpc with copying x_k_in to x_k

    int solve_planner(); // mpc planner: open loop mpc

    void update_mpc_weights(nlohmann::json param_weight);


    // Method to switch the trajectory
    void switch_traj(const casadi_real *const x_k_ptr);

    // if it solves too slow.
    void increase_traj_count()
    {
        traj_count++;
    }

    // if it solves too slow.
    void set_traj_count(casadi_uint new_traj_count)
    {
        traj_count = new_traj_count;
    }

    void reset();

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////// PUBLIC GETTER METHODS //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////

    // Method to output the optimal control
    casadi_real *get_optimal_control()
    {
        return mpc_config.out.u_opt.ptr;
    }

    casadi_real *get_u_next()
    {
        return mpc_config.in.u.ptr+nq_red;
    }

    casadi_uint get_N_step()
    {
        return N_step;
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

    casadi_uint get_traj_count()
    {
        return traj_count;
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

    const std::string get_mpc_name()
    {
        return mpc_name;
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
        return traj_data->col((traj_count > 0 ? traj_count-1 : traj_count)).data();
    }

    casadi_real* get_param_ptr(std::string key);

    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////// PUBLIC SETTER METHODS ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////

    // Method to set a parameter in the MPC configuration
    void set_param(std::string, const casadi_real *param_data);

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
    void set_references(const casadi_real *const x_k_in);

    void set_coldstart_init_guess(const casadi_real *const x_k_ptr);
    void init_references_and_pointers();

    void generate_trajectory_blocks();

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