#ifndef CASADICONTROLLER_HPP
#define CASADICONTROLLER_HPP

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
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

public:
    // Constructor
    CasadiController(const std::string &urdf_path, bool use_gravity);

    // solve the MPC
    Eigen::VectorXd solveMPC(casadi_real *x_k_ndof);

    // Getters and setters
    void setActiveMPC(MPCType mpc);
private:
    // Private methods
    std::string mpcToString(MPCType mpc);
};

#endif // CASADICONTROLLER_HPP