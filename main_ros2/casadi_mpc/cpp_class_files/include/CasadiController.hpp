#ifndef CASADICONTROLLER_HPP
#define CASADICONTROLLER_HPP

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry> // For rotations and quaternions
#include "casadi_types.h"
#include "param_robot.h"
#include "mpc_config.h"
#include "FullSystemTorqueMapper.hpp"
#include "CasadiMPC.hpp"
#include "casadi_controller_types.hpp"

class CasadiController
{
private:
    robot_config_t robot_config;                 // Robot configuration
    std::vector<CasadiMPC> casadi_mpcs;          // MPC objects
    MPCType selected_mpc_type;                   // Active MPC type
    CasadiMPC *active_mpc;                       // Active MPC object
    mpc_config_t *active_mpc_config;             // Active MPC configuration
    const mpc_input_config_t *active_mpc_input_config; // Active MPC input configuration

public:
    const casadi_uint nq;     // Number of degrees of freedom
    const casadi_uint nx;     // Number of reduced degrees of freedom
    const casadi_uint nq_red; // Number of reduced degrees of freedom
    const casadi_uint nx_red; // Number of reduced degrees of freedom

private:
    const casadi_uint *n_x_indices;
    FullSystemTorqueMapper torque_mapper; // Torque mapper
    casadi_real *x_k_ptr;                 // initial state address at the begin of the prediction horizon
    casadi_real *u_k_ptr;                 // optimal control address
    casadi_real *y_d_ptr;                 // desired trajectory address at the begin of the prediction horizon
    casadi_real *w_ptr;                   // Workspace real address
    casadi_uint traj_data_per_horizon;    // Trajectory data per horizon
    casadi_uint *mpc_traj_indices;        // Trajectory indices for not equidistant sampling
    casadi_real dt;
    std::map<std::string, double> param_transient_traj_poly;
    casadi_uint traj_rows;
    casadi_uint transient_traj_cnt;
    Eigen::MatrixXd transient_traj_data;
    casadi_uint transient_traj_len;       // number of columns of the transient trajectory data
    Eigen::MatrixXd singularity_traj_data;
    casadi_uint singularity_traj_len;     // number of columns of the singularity trajectory data
    std::vector<Eigen::MatrixXd> all_traj_data;
    Eigen::MatrixXd current_traj_data;
    Eigen::VectorXd tau_full_prev;
    ErrorFlag error_flag = ErrorFlag::NO_ERROR;
    double tau_max_jump = 5.0;

public:
    // Constructor
    CasadiController(const std::string &urdf_path, const std::string &tcp_frame_name, bool use_gravity);

    // solve the MPC
    Eigen::VectorXd solveMPC(const casadi_real *const x_k_ndof_ptr);

    // Method to generate a transient trajectory
    void generate_transient_trajectory(const casadi_real *const x_k_ndof_ptr,
                                       double T_start, double T_poly, double T_end);
    void generate_transient_trajectory(const casadi_real *const x_k_ndof_ptr);

    // Method to simulate the robot model
    void simulateModel(casadi_real *const x_k_ndof_ptr, const casadi_real *const tau_ptr, double dt);

    // Getters and setters
    void setActiveMPC(MPCType mpc);
    void setTransientTrajParams(double T_start, double T_poly, double T_end);

    const casadi_uint *get_n_indices()
    {
        return robot_config.n_indices;
    }

    // Method to switch the trajectory
    void switch_traj(casadi_uint traj_sel)
    {
        active_mpc->switch_traj(traj_sel);
    }

    // Method to get n_x_indices
    const casadi_uint *get_n_x_indices()
    {
        return robot_config.n_x_indices;
    }

    // Method to get n_indices_fixed
    const casadi_uint *get_n_indices_fixed()
    {
        return robot_config.n_indices_fixed;
    }

    // Method to get n_x_indices_fixed
    const casadi_uint *get_n_x_indices_fixed()
    {
        return robot_config.n_x_indices_fixed;
    }

    // Method to output the optimal control
    casadi_real *get_optimal_control()
    {
        return active_mpc->get_optimal_control();
    }

    // Method to get x_k
    casadi_real *get_x_k()
    {
        return active_mpc->get_x_k();
    }

    // Method to get the length of the trajectory data
    casadi_uint get_traj_data_len()
    {
        return active_mpc->get_traj_data_len();
    }

    // Method to get x_ref_nq
    const std::vector<casadi_real> &get_x_ref_nq()
    {
        return active_mpc->get_x_ref_nq();
    }

    // Method to get transient trajectory data
    const Eigen::MatrixXd &get_transient_traj_data()
    {
        return transient_traj_data;
    }

    // Method to get the transient trajectory length
    casadi_uint get_transient_traj_len()
    {
        return transient_traj_len;
    }

    // Method to get the is kinematic mpc flag
    bool get_is_kinematic_mpc()
    {
        return active_mpc->is_kinematic_mpc;
    }

    // Method to get the error flag
    ErrorFlag get_error_flag()
    {
        return error_flag;
    }

    // Method to set the maximum torque jump
    void set_tau_max_jump(double tau_jump)
    {
        tau_max_jump = tau_jump;
    }

private:
    // Private methods
    std::string mpcToString(MPCType mpc);

    // Functions for transient polynomial trajectory generation
    Eigen::Vector4d trajectory_poly(double t, const Eigen::Vector4d &y0, const Eigen::Vector4d &yT, double T);

    Eigen::VectorXd create_poly_traj(const Eigen::Vector3d &yT, const Eigen::Vector3d &y0, double t,
                                     const Eigen::Matrix3d &R_init, const Eigen::Vector3d &rot_ax,
                                     double rot_alpha_scale, const std::map<std::string, double> &param_traj_poly);

    Eigen::MatrixXd generate_trajectory(double dt, const Eigen::Vector3d &xe0, const Eigen::Vector3d &xeT,
                                        const Eigen::Matrix3d &R_init, const Eigen::Matrix3d &R_target,
                                        const std::map<std::string, double> &param_traj_poly);

    std::vector<Eigen::MatrixXd> readTrajectoryData(const std::string& traj_file);
};
#endif // CASADICONTROLLER_HPP