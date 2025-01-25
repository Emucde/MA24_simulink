#ifndef WORKSPACE_CONTROLLER_CONFIG_HPP
#define WORKSPACE_CONTROLLER_CONFIG_HPP

#include <Eigen/Dense> // Include Eigen for matrix types

struct PDSettings {
    Eigen::MatrixXd D_d;                // Damping matrix (not used)
    Eigen::MatrixXd K_d;                // PD gain for Cartesian PD Control (not used)
};

struct CTSettings {
    Eigen::MatrixXd Kd1;                // Control gains diagonal (combined)
    Eigen::MatrixXd Kp1;                // Control gains proportional (combined)
};

struct RegularizationSettings {
    int mode;                           // Regularization mode
    double k;                           // Regularization parameter J_pinv = (J^T J + k I)^-1 J^T
    Eigen::MatrixXd W_bar_N;            // Parameter for singularity robustness (sugihara)
    Eigen::MatrixXd W_E;                // Weight matrix for the error (sugihara)
    double eps;                         // Regularization parameter for singular values
    double eps_collinear;               // Threshold for collinear joints
    double lambda_min;                  // Minimum eigenvalue for regularization
};

struct ControllerSettings {
    PDSettings pd_plus_settings;        // PD controller settings
    CTSettings ct_settings;              // CT controller settings
    RegularizationSettings regularization_settings; // Regularization settings
};

#endif // WORKSPACE_CONTROLLER_CONFIG_HPP