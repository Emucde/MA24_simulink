/*
This file is used in RobotModel.hpp to define the data structures
for joint, kinematics, and dynamics information of the robot.
*/

#ifndef ROBOT_DATA_HPP
#define ROBOT_DATA_HPP

#include <Eigen/Dense> // Include for matrix types

struct JointData {
    Eigen::VectorXd q; // Joint positions
    Eigen::VectorXd q_p; // Joint velocities
};

struct KinematicsData {
    Eigen::MatrixXd J; // Jacobian of forward kinematics
    Eigen::MatrixXd J_p; // Time derivative of Jacobian
    Eigen::MatrixXd H; // Homogeneous transformation 4x4 Matrix
    Eigen::Matrix3d R; // Rotation matrix
    Eigen::Vector3d p; // Position vector
    Eigen::Quaterniond quat; // wxyz Quaternion
};

struct DynamicsData {
    Eigen::MatrixXd M; // Mass matrix
    Eigen::MatrixXd C; // Coriolis matrix
    Eigen::MatrixXd C_rnea; // Gravitational effects
    Eigen::VectorXd g; // Gravitational forces
};

enum class ControllerType { CT, PDPlus, InverseDynamics };

#endif // ROBOT_DATA_HPP