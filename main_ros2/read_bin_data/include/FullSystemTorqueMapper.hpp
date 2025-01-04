#ifndef FULLSYSTEMTORQUEMAPPER_HPP
#define FULLSYSTEMTORQUEMAPPER_HPP

#include <vector>
#include "CasadiMPC.hpp"
#include "casadi_types.h"
#include <iostream>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Dense>

class FullSystemTorqueMapper
{
public:
    // Configuration parameters structure
    struct Config
    {
        Eigen::MatrixXd K_d; // Proportional gain matrix
        Eigen::MatrixXd D_d; // Derivative gain matrix
        double torque_limit; // Max allowable torque
    };

    // Constructor
    FullSystemTorqueMapper(const std::string &urdf_filename,
                           CasadiMPC &mpc_obj, bool use_gravity);

    // Functions
    Eigen::VectorXd calculateNdofTorqueWithFeedforward(const Eigen::VectorXd &u,
                                                                           const Eigen::VectorXd &q,
                                                                           const Eigen::VectorXd &q_p,
                                                                           int n, bool kin_model);

    Eigen::VectorXd applyPDControl(const Eigen::VectorXd &q_fixed,
                                   const Eigen::VectorXd &q_p_fixed,
                                   const Eigen::VectorXd &q_0_ref_fixed,
                                   const Eigen::MatrixXd &K_d_fixed,
                                   const Eigen::MatrixXd &D_d_fixed);

    Eigen::VectorXd calc_full_torque(const Eigen::VectorXd &u, const Eigen::VectorXd &x_k_ndof, int n, bool kin_model);

    // Getters and setters
    const Eigen::VectorXd &getTauFull() const { return tau_full; }
    void setTauFull(const Eigen::VectorXd &tau) { tau_full = tau; }

    void setConfiguration(const Config &new_config) { config = new_config; }

private:
    const std::string urdf_filename;
    CasadiMPC &mpc_obj;
    casadi_uint nq;       // Number of degrees of freedom
    casadi_uint nx;       // Number of reduced degrees of freedom
    casadi_uint nq_red;   // Number of reduced degrees of freedom
    casadi_uint nx_red;   // Number of reduced degrees of freedom
    casadi_uint nq_fixed; // Number of reduced degrees of freedom
    pinocchio::Model robot_model_full;
    pinocchio::Data robot_data_full;
    Eigen::VectorXi n_indices;
    Eigen::VectorXi n_indices_fixed;
    Eigen::VectorXd x_ref_nq;
    Eigen::VectorXd q_ref_nq;
    Eigen::VectorXd q_ref_fixed;

    Eigen::MatrixXd K_d;       // Stiffness matrix
    Eigen::MatrixXd D_d;       // Damping matrix
    Eigen::MatrixXd K_d_fixed; // For fixed proportional gain
    Eigen::MatrixXd D_d_fixed; // For fixed derivative gain
    Eigen::VectorXd tau_full;
    Eigen::VectorXd q_pp;

    Config config;                     // Configuration parameters

    void initRobot(const std::string &urdf_filename, pinocchio::Model &robot_model, pinocchio::Data &robot_data, bool use_gravity);
    Eigen::VectorXd enforceTorqueLimits(const Eigen::VectorXd &tau);
};

#endif // FULLSYSTEMTORQUEMAPPER_HPP