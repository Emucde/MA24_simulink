#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include <Eigen/Dense>
// #include "eigen_templates.hpp"
#include "robot_data.hpp" // Include the previously defined structs
#include "param_robot.h"
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>

class RobotModel
{
public:
    // Constructor for full model
    RobotModel(const std::string &urdf_filename,
               const std::string &tcp_frame_name,
               robot_config_t &robot_config,
               bool use_gravity,
               bool reduced_model = false);

    // Update state with joint positions and velocities
    void updateState(const Eigen::VectorXd &x);

    // Member variables
    pinocchio::Model robot_model;            // Pinocchio model
    pinocchio::Data robot_data;              // Pinocchio data
    pinocchio::Model::FrameIndex tcp_frame_id; // TCP frame index

    int nq;        // Number of degrees of freedom
    int nx;        // Number of degrees of freedom

    JointData jointData;               // Joint data structure
    KinematicsData kinematicsData;     // Kinematics data structure
    DynamicsData dynamicsData;         // Dynamics data structure

    const std::string &urdf_filename; // TCP frame name
    const std::string &tcp_frame_name; // TCP frame name
    robot_config_t &robot_config;

    // Private methods for calculations
    void computeKinematics();
    void computeDynamics();
};

#endif // ROBOT_MODEL_HPP