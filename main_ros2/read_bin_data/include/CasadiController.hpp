#ifndef CASADICONTROLLER_HPP
#define CASADICONTROLLER_HPP

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include "casadi_types.h"
#include "pinocchio_utils.hpp"
#include "FullSystemTorqueMapper.hpp"
#include "CasadiMPC.hpp"

class CasadiController {
public:
    // Constructor with URDF file path and options for gravity and kinematic model
    CasadiController(const std::string& urdfFilePath, bool useGravity);

private:
    // // Methods for initialization and processing
    // void setupIndicesAndMaps();
    // void writeDataToFiles(std::ofstream& x_k_ndof_file, std::ofstream& tau_full_file);
    // void printCurrentState();

    // // Member variables
    // bool useGravity;
    // bool fr3KinematicModel;
    // std::unordered_map<std::string, CasadiMPC> mpcObjects; // Store all MPC instances
    // FullSystemTorqueMapper torqueMapper; // Torque mapper object
    
    // casadi_real* u_opt; // Optimal control output pointer
    // casadi_real* x_k; // Current state pointer
    // unsigned int trajDataLength; // Length of trajectory data
    // Eigen::VectorXd tau_full; // Output torque vector
    // Eigen::VectorXd x_k_ndof; // Updated state vector
    // Eigen::VectorXd x_ref_nq_map; // Reference state vector
    // Eigen::VectorXd x_k_map; // Current state map
    // Eigen::Map<Eigen::VectorXi> n_x_indices; // Indices of x_k
    // std::vector<casadi_real*> x_k_ndof_ptr; // Pointers to elements of x_k_ndof
};

#endif // CASADICONTROLLER_HPP