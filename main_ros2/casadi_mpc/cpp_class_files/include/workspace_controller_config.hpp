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

enum class SingularityRobustnessMode {
    None = 0,                            // No singularity robustness
    Damping,                             // Add damping to the matrix
    CompleteOrthogonalDecomposition,     // Complete orthogonal decomposition method
    JacobiSVD,                           // Jacobi SVD method
    ThresholdSmallSingularValues,        // Remove small singular values
    TikhonovRegularization,              // Tikhonov regularization (adding epsilon^2)
    SimpleCollinearity,                  // Simple collinearity check
    SteinboeckCollinearity,              // Steinboeck's collinearity approach
    RegularizationBySingularValues       // Regularization by singular values (handle small singular values)
};;

#endif // WORKSPACE_CONTROLLER_CONFIG_HPP