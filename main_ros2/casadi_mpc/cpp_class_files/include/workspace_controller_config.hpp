#ifndef WORKSPACE_CONTROLLER_CONFIG_HPP
#define WORKSPACE_CONTROLLER_CONFIG_HPP

#include <Eigen/Dense> // Include Eigen for matrix types

struct PDSettings {
    Eigen::MatrixXd D_d;                // Damping matrix
    Eigen::MatrixXd K_d;                // Stiffness matrix for Cartesian PD Control
};

struct CTSettings {
    Eigen::MatrixXd Kd1;                // Control gains diagonal (combined)
    Eigen::MatrixXd Kp1;                // Control gains proportional (combined)
};

struct IDSettings {
    Eigen::MatrixXd Kd1;                // Control gains Inverse dynamics diagonal (combined)
    Eigen::MatrixXd Kp1;                // Control gains Inverse dynamics proportional (combined)
    Eigen::MatrixXd D_d;                // Damping matrix for Joint space PD Control
    Eigen::MatrixXd K_d;                // Stiffness matrix for Joint space PD Control
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
    IDSettings id_settings;             // Inverse Dynamics controller settings
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