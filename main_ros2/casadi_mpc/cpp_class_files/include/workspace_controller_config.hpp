/*
This file defines the configuration for the WorkspaceController.
It includes settings for different controller types, regularization modes,
and various parameters related to the robot's dynamics and kinematics.
*/

#ifndef WORKSPACE_CONTROLLER_CONFIG_HPP
#define WORKSPACE_CONTROLLER_CONFIG_HPP

#include <Eigen/Dense> // Include Eigen for matrix types

enum class RegularizationMode {
    None = 0,                            // No singularity robustness
    Damping,                             // Add damping to the matrix
    CompleteOrthogonalDecomposition,     // Complete orthogonal decomposition method
    JacobiSVD,                           // Jacobi SVD method
    ThresholdSmallSingularValues,        // Remove small singular values
    TikhonovRegularization,              // Tikhonov regularization (adding epsilon^2)
    SimpleCollinearity,                  // Simple collinearity check
    SteinboeckCollinearity,              // Steinboeck's collinearity approach
    RegularizationBySingularValues       // Regularization by singular values (handle small singular values)
};

template <typename T>
std::string regularizationModeToString(T mode)
{
    switch (mode)
    {
    case RegularizationMode::None:
        return "None";
    case RegularizationMode::Damping:
        return "Damping";
    case RegularizationMode::CompleteOrthogonalDecomposition:
        return "CompleteOrthogonalDecomposition";
    case RegularizationMode::JacobiSVD:
        return "JacobiSVD";
    case RegularizationMode::ThresholdSmallSingularValues:
        return "ThresholdSmallSingularValues";
    case RegularizationMode::TikhonovRegularization:
        return "TikhonovRegularization";
    case RegularizationMode::SimpleCollinearity:
        return "SimpleCollinearity";
    case RegularizationMode::SteinboeckCollinearity:
        return "SteinboeckCollinearity";
    case RegularizationMode::RegularizationBySingularValues:
        return "RegularizationBySingularValues";
    default:
        return "Unknown";
    }
}
template <typename T>
RegularizationMode stringToRegularizationMode(const T& str)
{
    if (str == "None")
        return RegularizationMode::None;
    else if (str == "Damping")
        return RegularizationMode::Damping;
    else if (str == "CompleteOrthogonalDecomposition")
        return RegularizationMode::CompleteOrthogonalDecomposition;
    else if (str == "JacobiSVD")
        return RegularizationMode::JacobiSVD;
    else if (str == "ThresholdSmallSingularValues")
        return RegularizationMode::ThresholdSmallSingularValues;
    else if (str == "TikhonovRegularization")
        return RegularizationMode::TikhonovRegularization;
    else if (str == "SimpleCollinearity")
        return RegularizationMode::SimpleCollinearity;
    else if (str == "SteinboeckCollinearity")
        return RegularizationMode::SteinboeckCollinearity;
    else if (str == "RegularizationBySingularValues")
        return RegularizationMode::RegularizationBySingularValues;
    else
        throw std::invalid_argument("Unknown regularization mode string");
}

struct PDSettings {
    Eigen::MatrixXd K_I;                // Integral gains diagonal (combined)
    Eigen::MatrixXd D_d;                // Damping matrix
    Eigen::MatrixXd K_d;                // Stiffness matrix for Cartesian PD Control
};

struct CTSettings {
    Eigen::MatrixXd K_I;                // Integral gains diagonal (combined)
    Eigen::MatrixXd Kd1;                // Control gains diagonal (combined)
    Eigen::MatrixXd Kp1;                // Control gains proportional (combined)
};

struct IDSettings {
    Eigen::MatrixXd Kd1;                // Control gains Inverse dynamics diagonal (combined)
    Eigen::MatrixXd Kp1;                // Control gains Inverse dynamics proportional (combined)
    Eigen::MatrixXd D_d;                // Damping matrix for Joint space PD Control
    Eigen::MatrixXd K_d;                // Stiffness matrix for Joint space PD Control
};

struct StateObserverSettings {
    Eigen::MatrixXd Kd1;                // Control gains Inverse dynamics diagonal (combined)
    Eigen::MatrixXd Kp1;                // Control gains Inverse dynamics proportional (combined)
};

struct RegularizationSettings {
    RegularizationMode mode;            // Regularization mode
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
    StateObserverSettings state_observer_settings; // State observer settings
    RegularizationSettings regularization_settings; // Regularization settings
};

#endif // WORKSPACE_CONTROLLER_CONFIG_HPP