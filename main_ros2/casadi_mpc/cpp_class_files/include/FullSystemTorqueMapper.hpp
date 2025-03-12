#ifndef FULLSYSTEMTORQUEMAPPER_HPP
#define FULLSYSTEMTORQUEMAPPER_HPP

#include <vector>
#include "param_robot.h"
#include <iostream>
#include "json.hpp"
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/algorithm/frames.hpp"
#include <pinocchio/algorithm/model.hpp>
#include "pinocchio/algorithm/aba.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> // For rotations and quaternions

class FullSystemTorqueMapper
{
public:
    // Constructor
    FullSystemTorqueMapper(const std::string &urdf_filename,
                           robot_config_t &robot_config,
                           const std::string &general_config_file);

    // Function pointer to feedforward function
    using FeedforwardTorqueFunc = Eigen::VectorXd (FullSystemTorqueMapper::*)(const Eigen::VectorXd &, const Eigen::VectorXd &, const Eigen::VectorXd &);
    FeedforwardTorqueFunc calcFeedforwardTorqueFunPtr; // Function pointer to torque calculation method

    // Configuration parameters structure
    struct Config
    {
        Eigen::MatrixXd K_d;            // Proportional gain matrix
        Eigen::MatrixXd D_d;            // Derivative gain matrix
        Eigen::MatrixXd K_d_fixed;      // For fixed proportional gain
        Eigen::MatrixXd D_d_fixed;      // For fixed derivative gain
        Eigen::VectorXd q_ref_nq;       // Reference joint positions
        Eigen::VectorXd q_ref_nq_fixed; // Reference joint positions for fixed PD control
        double torque_limit;            // Max allowable torque
    };

    void update_config();
    void update_config(double dt_new);

    // Functions
    Eigen::VectorXd calcFeedforwardTorqueKinematic(const Eigen::VectorXd &u,
                                                   const Eigen::VectorXd &q,
                                                   const Eigen::VectorXd &q_p);

    Eigen::VectorXd calcFeedforwardTorqueDynamic(const Eigen::VectorXd &u,
                                                 const Eigen::VectorXd &q,
                                                 const Eigen::VectorXd &q_p);

    Eigen::VectorXd calcFeedforwardTorqueKinematicAlternative(const Eigen::VectorXd &u,
                                                   const Eigen::VectorXd &q,
                                                   const Eigen::VectorXd &q_p);

    Eigen::VectorXd calcFeedforwardTorqueDynamicAlternative(const Eigen::VectorXd &u,
                                                 const Eigen::VectorXd &q,
                                                 const Eigen::VectorXd &q_p);

    void setFeedforwardTorqueFunction(bool is_kinematic_mpc);

    Eigen::VectorXd applyPDControl(const Eigen::VectorXd &q_fixed,
                                   const Eigen::VectorXd &q_p_fixed);

    Eigen::VectorXd calc_full_torque(const Eigen::VectorXd &u, const Eigen::VectorXd &x_k_ndof);

    // Method to calculate the pose (p, R) by a given state
    void calcPose(const double* x, Eigen::Vector3d &p, Eigen::Matrix3d &R);
    void calcPose(const Eigen::VectorXd &q, Eigen::Vector3d &p, Eigen::Matrix3d &R);

    // Method to simulate the robot model
    void simulateModelEuler(Eigen::Map<Eigen::VectorXd> &x_k_ndof, Eigen::Map<const Eigen::VectorXd> &tau, double dt);
    void simulateModelRK4(Eigen::Map<Eigen::VectorXd> &x_k_ndof, Eigen::Map<const Eigen::VectorXd> &tau, double dt);

    // Getters and setters
    const Eigen::VectorXd &getTauFull() const { return tau_full; }
    void setTauFull(const Eigen::VectorXd &tau) { tau_full = tau; }

    void setConfiguration(const Config &new_config) { config = new_config; }
    void set_kinematic_mpc_flag(bool is_kinematic)
    {
        is_kinematic_mpc = is_kinematic;
        setFeedforwardTorqueFunction(is_kinematic_mpc); // Update the feedforward torque function pointer
    }

    // Method to get the robot config
    robot_config_t &getRobotConfig()
    {
        return robot_config;
    }

private:
    const std::string urdf_filename;
    const std::string general_config_filename;
    std::string tcp_frame_name;
    robot_config_t &robot_config;
    bool is_kinematic_mpc; // Kinematic MPC flag
    bool use_gravity;      // Flag to use gravity in the simulation
    uint nq;        // Number of degrees of freedom
    uint nx;        // Number of degrees of freedom
    uint nq_red;    // Number of reduced degrees of freedom
    uint nx_red;    // Number of reduced degrees of freedom
    uint nq_fixed;  // Number of reduced degrees of freedom
    pinocchio::Model robot_model_full;
    pinocchio::Data robot_data_full;
    pinocchio::Model robot_model_reduced;
    pinocchio::Data robot_data_reduced;
    pinocchio::Model::FrameIndex tcp_frame_id;
    const Eigen::VectorXi n_indices;
    const Eigen::VectorXi n_indices_fixed;
    const Eigen::VectorXd q_ref_nq;
    const Eigen::VectorXd q_ref_fixed;
    Eigen::MatrixXd M_fixed;
    const Eigen::MatrixXd J_psi;
    const Eigen::MatrixXd J_psi_T;
    const Eigen::MatrixXd A;

    Eigen::VectorXd tau_full;
    Eigen::VectorXd q_pp;
    double dt;

    Config config; // Configuration parameters

    void initRobot();
    Eigen::MatrixXd calc_reduced_mapping_matrix();
    Eigen::MatrixXd calc_coercive_condition_matrix();
    Eigen::VectorXd enforceTorqueLimits(const Eigen::VectorXd &tau);
    Eigen::VectorXd RK4(const Eigen::VectorXd& state, double dt, const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f);
};

#endif // FULLSYSTEMTORQUEMAPPER_HPP