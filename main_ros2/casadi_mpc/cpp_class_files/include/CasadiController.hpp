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
#include "FullSystemTorqueMapper.hpp"
#include "CasadiMPC.hpp"

enum class MPCType
{
    MPC01,
    MPC6,
    MPC7,
    MPC8,
    MPC9,
    MPC10,
    MPC11,
    MPC12,
    MPC13,
    MPC14,
    INVALID, // Consider using INVALID for out-of-bounds
    COUNT    // This can denote the number of valid enum values
};

class CasadiController
{
private:
    const std::string &urdf_path;       // URDF file path
    robot_config_t robot_config;        // Robot configuration
    std::vector<CasadiMPC> casadi_mpcs; // MPC objects
    MPCType selected_mpc_type;          // Active MPC type
    CasadiMPC *active_mpc;              // Active MPC object

public:
    const casadi_uint nq;     // Number of degrees of freedom
    const casadi_uint nx;     // Number of reduced degrees of freedom
    const casadi_uint nq_red; // Number of reduced degrees of freedom
    const casadi_uint nx_red; // Number of reduced degrees of freedom

private:
    const casadi_uint *n_x_indices;
    FullSystemTorqueMapper torque_mapper; // Torque mapper
    casadi_real *x_k_ptr;
    casadi_real *u_k_ptr;
    casadi_uint dt;

public:
    // Constructor
    CasadiController(const std::string &urdf_path, bool use_gravity);

    // solve the MPC
    Eigen::VectorXd solveMPC(const casadi_real *const x_k_ndof_ptr);

    // Getters and setters
    void setActiveMPC(MPCType mpc);
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

private:
    // Private methods
    std::string mpcToString(MPCType mpc);
    
    // Functions for transient polynomial trajectory generation
    Eigen::Vector4d trajectory_poly(double t, const Eigen::Vector4d &y0, const Eigen::Vector4d &yT, double T);

    Eigen::VectorXd create_poly_traj(const Eigen::Vector4d &yT, const Eigen::Vector4d &y0, double t,
                                     const Eigen::Matrix3d &R_init, const Eigen::Vector3d &rot_ax,
                                     double rot_alpha_scale, const std::map<std::string, double> &param_traj_poly);

    Eigen::MatrixXd generate_trajectory(double dt, const Eigen::Vector4d &xe0, const Eigen::Vector4d &xeT,
                                        const Eigen::Matrix3d &R_init, const Eigen::Matrix3d &R_target,
                                        const std::map<std::string, double> &param_traj_poly);
};
#endif // CASADICONTROLLER_HPP