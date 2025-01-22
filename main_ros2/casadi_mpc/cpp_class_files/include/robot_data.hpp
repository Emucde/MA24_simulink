#ifndef ROBOT_DATA_HPP
#define ROBOT_DATA_HPP

#include <Eigen/Dense> // Include for matrix types

struct JointData {
    double q; // Joint position
    double q_p; // Joint velocity
};

struct KinematicsData {
    Eigen::MatrixXd J; // Jacobian of forward kinematics
    Eigen::MatrixXd J_p; // Time derivative of Jacobian
    Eigen::MatrixXd H; // Homogeneous transformation 4x4 Matrix
};

struct DynamicsData {
    Eigen::MatrixXd M; // Mass matrix
    Eigen::MatrixXd C; // Coriolis matrix
    Eigen::MatrixXd C_rnea; // Gravitational effects
    Eigen::MatrixXd g; // Gravitational forces
};

enum class ControllerType { CT, PDPlus, InverseDynamics };

#endif // ROBOT_DATA_HPP