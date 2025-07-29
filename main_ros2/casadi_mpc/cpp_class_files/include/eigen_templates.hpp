/*
This file contains template definitions and utility functions for the Eigen library.
It includes definitions for matrix and vector operations, JSON handling, and utility functions
for reading configuration files.
*/

#ifndef EIGEN_TEMPLATES_HPP
#define EIGEN_TEMPLATES_HPP

#include <Eigen/Dense>
#include <cmath>
#include <array>
#include <type_traits>
#include "json.hpp"
#include <fstream>
#include <stdexcept>

template<typename T = nlohmann::json>
T read_config(const std::string& file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        throw std::runtime_error("Error: Could not open JSON file.");
    }
    T jsonData;
    file >> jsonData; // Parse JSON file
    file.close();
    return jsonData;
}

// Template function to safely get a config value from nlohmann::json
template<typename T>
T get_config_value(const nlohmann::json& config, const std::string& key) {
    if (config.find(key) == config.end()) {
        throw std::runtime_error("JSON object key '" + key + "' does not exist!");
    }
    try {
        return config[key].get<T>();
    } catch (const nlohmann::detail::type_error& e) {
        throw std::runtime_error("JSON object key '" + key + "' exists but has incompatible type. Expected type: " + std::string(typeid(T).name()) + ". Error: " + e.what());
    }
}

template <typename T>
Eigen::Map<const Eigen::VectorXi> ConstIntVectorMap(T *ptr, int size)
{
    return Eigen::Map<const Eigen::VectorXi>(reinterpret_cast<int *>(const_cast<uint32_t *>(ptr)), size);
}

template <typename T>
Eigen::Map<const Eigen::VectorXd> ConstDoubleVectorMap(T *ptr, int size)
{
    return Eigen::Map<const Eigen::VectorXd>(const_cast<double *>(ptr), size);
}

template <typename T = double>
Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& v) {
    Eigen::Matrix<T, 3, 3> S;
    S << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
    return S;
}

// Reference:
//   Sarabandi, S., & Thomas, F. (2018). Accurate computation of quaternions
//   from rotation matrices. In International Symposium on Advances in Robot
//   Kinematics. Springer International Publishing.
template <typename T = double>
Eigen::Vector4d rotm2quat_v4(const Eigen::Matrix3d &R)
{
    static_assert(std::is_floating_point<T>::value, "Template parameter must be a floating-point type");

    const T eta = static_cast<T>(0);

    // Extract elements of the rotation matrix
    T r11 = R(0, 0), r12 = R(0, 1), r13 = R(0, 2);
    T r21 = R(1, 0), r22 = R(1, 1), r23 = R(1, 2);
    T r31 = R(2, 0), r32 = R(2, 1), r33 = R(2, 2);

    // Compute intermediate sums
    T s1 = r11 + r22 + r33;
    T s2 = r11 - r22 - r33;
    T s3 = -r11 + r22 - r33;
    T s4 = -r11 - r22 + r33;

    // Pre-compute differences
    T d3223 = r32 - r23;
    T d1331 = r13 - r31;
    T d2112 = r21 - r12;

    T p1221 = r12 + r21;
    T p2332 = r23 + r32;
    T p3113 = r31 + r13;

    // Helper lambda for sign computation
    auto my_sign = [](T x)
    { return (x >= 0) ? static_cast<T>(1) : static_cast<T>(-1); };

    // Lambda for quaternion calculation
    auto computeQuatComponent = [&](T s, T a, T b, T c)
    {
        if (s > eta)
        {
            return std::sqrt(static_cast<T>(1) + s) / static_cast<T>(2);
        }
        else
        {
            return std::sqrt(((a * a) + (b * b) + (c * c)) / (static_cast<T>(3) - s)) / static_cast<T>(2);
        }
    };

    // Compute quaternion components
    T q1 = computeQuatComponent(s1, d3223, d1331, d2112);
    T q2 = computeQuatComponent(s2, d3223, p1221, p3113);
    T q3 = computeQuatComponent(s3, d1331, p1221, p2332);
    T q4 = computeQuatComponent(s4, d2112, p3113, p2332);

    // Assemble the quaternion with proper signs
    return Eigen::Vector4d(q1, my_sign(d3223) * q2, my_sign(d1331) * q3, my_sign(d2112) * q4);

    /* MATLAB:
    eta = 0;

    % Extract the elements of the rotation matrix
    r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
    r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
    r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);

    s1 =  r11 + r22 + r33;
    s2 =  r11 - r22 - r33;
    s3 = -r11 + r22 - r33;
    s4 = -r11 - r22 + r33;

    if(s1 > eta)
        q1 = sqrt( 1 + s1 ) / 2;
    else
        q1 = sqrt(  ((r32 - r23)^2 + (r13 - r31)^2 + (r21 - r12)^2) / (3 - r11 - r22 - r33)  ) / 2;
    end

    if(s2 > eta)
        q2 = sqrt( 1 + s2 ) / 2;
    else
        q2 = sqrt(  ((r32 - r23)^2 + (r12 + r21)^2 + (r31 + r13)^2) / (3 - r11 + r22 + r33)  ) / 2;
    end

    if(s3 > eta)
        q3 = sqrt( 1 + s3 ) / 2;
    else
        q3 = sqrt(  ((r13 - r31)^2 + (r12 + r21)^2 + (r23 + r32)^2) / (3 + r11 - r22 + r33)  ) / 2;
    end

    if(s4 > eta)
        q4 = sqrt( 1 + s4 ) / 2;
    else
        q4 = sqrt(  ((r21 - r12)^2 + (r31 + r13)^2 + (r32 + r23)^2) / (3 + r11 + r22 - r33)  ) / 2;
    end

    % Assemble the quaternion
    q = [q1; my_sign(r32 - r23) * q2; my_sign(r13 - r31) * q3; my_sign(r21 - r12) * q4];
    */
}

template <typename T = double>
Eigen::Vector3d rotm2rpy(const Eigen::Matrix3d &R)
{
    static_assert(std::is_floating_point<T>::value, "Template parameter must be a floating-point type");
    // Extract elements of the rotation matrix
    T r11 = R(0, 0);
    T r12 = R(0, 1);
    T r13 = R(0, 2);
    T r21 = R(1, 0);
    //T r22 = R(1, 1);
    //T r23 = R(1, 2);
    T r31 = R(2, 0);
    T r32 = R(2, 1);
    T r33 = R(2, 2);

    // Calculate the roll, pitch, and yaw angles (Craig, S. 47f)
    // alpha = yaw, beta = pitch, gamma = roll
    T beta = std::atan2(-r31, std::sqrt(r32 * r32 + r33 * r33));
    T cos_beta = std::cos(beta);

    T gamma, alpha;

    if (std::abs(beta - M_PI / 2) < 1e-6) { // beta == pi/2  Using a tolerance for comparison
        gamma = std::atan2(r12, r13);
        alpha = 0.0;
    } else if (std::abs(beta + M_PI / 2) < 1e-6) { // beta == -pi/2 Using a tolerance for comparison
        gamma = std::atan2(-r12, -r13);
        alpha = 0.0;
    } else {
        gamma = std::atan2(r32 / cos_beta, r33 / cos_beta);
        alpha = std::atan2(r21 / cos_beta, r11 / cos_beta);
    }

    Eigen::Vector3d rpy(alpha, beta, gamma);
    return rpy;
}

template <typename T = double>
Eigen::Matrix3d rpy2rotm(const Eigen::Vector3d &rpy)
{
    static_assert(std::is_floating_point<T>::value, "Template parameter must be a floating-point type");

    T alpha = rpy(0); // yaw
    T beta = rpy(1);  // pitch
    T gamma = rpy(2); // roll

    Eigen::Matrix3d Rz;
    Rz << std::cos(alpha), -std::sin(alpha), 0,
          std::sin(alpha),  std::cos(alpha), 0,
          0,                0,               1;

    Eigen::Matrix3d Ry;
    Ry << std::cos(beta), 0, std::sin(beta),
          0,              1, 0,
          -std::sin(beta), 0, std::cos(beta);

    Eigen::Matrix3d Rx;
    Rx << 1, 0,                0,
          0, std::cos(gamma), -std::sin(gamma),
          0, std::sin(gamma),  std::cos(gamma);

    return Rz * Ry * Rx;
}

/**
 * @brief Computes a specific 3x3 matrix based on roll and pitch angles.
 *
 * This function takes a vector of Euler angles (specifically, roll and pitch)
 * and calculates a 3x3 matrix according to the formula implemented
 * in the original MATLAB code.  The yaw angle is not used.
 *
 * @param Phi A 3D vector where:
 *            - Phi(0) is the roll angle (around the x-axis).
 *            - Phi(1) is the pitch angle (around the y-axis).
 *            - Phi(2) is the yaw angle (around the z-axis), *which is ignored by this function*.
 *
 * @return A 3x3 Eigen matrix calculated based on the input angles.
 */
template <typename T = double>
Eigen::Matrix3d T_rpy(const Eigen::Vector3d& Phi)
{
    static_assert(std::is_floating_point<T>::value, "Template parameter must be a floating-point type");
   
   Eigen::Matrix3d m = Eigen::Matrix3d::Zero(); // Initialize to a zero matrix

    // Extract roll and pitch angles
    T roll  = Phi(0);  // Rotation around the x-axis
    T pitch = Phi(1);  // Rotation around the y-axis

    // Calculate trigonometric values
    T sin_roll  = std::sin(roll);
    T cos_roll  = std::cos(roll);
    T cos_pitch = std::cos(pitch);
    T sin_pitch = std::sin(pitch);

    // Populate the matrix elements according to the original MATLAB code
    m(0, 1) = -sin_roll;
    m(0, 2) = cos_pitch * cos_roll;
    m(1, 1) = cos_roll;
    m(1, 2) = cos_pitch * sin_roll;
    m(2, 0) = 1.0;
    m(2, 2) = -sin_pitch;

    return m;
}


/**
 * @brief Computes the derivative of the T_rpy matrix.
 *
 * This function calculates the derivative of the T_rpy matrix with respect to time,
 * given the current RPY angles (Phi) and their corresponding angular velocities (Phi_p).
 * It directly translates the formula from the original MATLAB code.
 *
 * @param Phi   A 3D vector representing the RPY angles (roll, pitch, yaw - yaw is ignored).
 * @param Phi_p A 3D vector representing the RPY angular velocities (roll_dot, pitch_dot, yaw_dot - yaw_dot is ignored).
 *
 * @return The 3x3 matrix representing the time derivative of T_rpy.
 */
template <typename T = double>
Eigen::Matrix3d T_rpy_p(const Eigen::Vector3d& Phi, const Eigen::Vector3d& Phi_p) {
    static_assert(std::is_floating_point<T>::value, "Template parameter must be a floating-point type");
    
    Eigen::Matrix3d m = Eigen::Matrix3d::Zero(); // Initialize to a zero matrix

    // Extract roll, pitch, roll_dot, and pitch_dot
    T roll      = Phi(0);
    T pitch     = Phi(1);
    T roll_dot  = Phi_p(0);
    T pitch_dot = Phi_p(1);

    // Calculate trigonometric values
    T sin_roll  = std::sin(roll);
    T cos_roll  = std::cos(roll);
    T cos_pitch = std::cos(pitch);
    T sin_pitch = std::sin(pitch);

    // Populate the matrix elements based on the MATLAB code
    m(0, 1) = -cos_roll * roll_dot;
    m(0, 2) = -cos_roll * sin_pitch * pitch_dot - cos_pitch * sin_roll * roll_dot;
    m(1, 1) = -sin_roll * roll_dot;
    m(1, 2) = -sin_roll * sin_pitch * pitch_dot + cos_pitch * cos_roll * roll_dot;
    m(2, 2) = -cos_pitch * pitch_dot;

    return m;
}

/**
 * @brief Calculates angular velocities and accelerations in RPY coordinates.
 *
 * This function takes a rotation matrix, desired angular velocity, and desired
 * angular acceleration as input and computes the corresponding RPY angles,
 * angular velocities, and angular accelerations.  It avoids direct matrix inversion
 * by solving a linear system instead.
 *
 * @param R_act     The current rotation matrix.
 * @param omega_d   The desired angular velocity in the world frame.
 * @param omega_d_p The desired angular acceleration in the world frame.
 */
template <typename T = double>
Eigen::VectorXd calculateRPYVelocitiesAndAccelerations(
    const Eigen::Matrix3d& R_act,
    const Eigen::Vector3d& omega_d,
    const Eigen::Vector3d& omega_d_p) {

    Eigen::Vector3d Phi_act;
    Eigen::Vector3d Phi_act_p;
    Eigen::Vector3d Phi_act_pp;

    // Calculate RPY angles from the rotation matrix
    Phi_act = rotm2rpy(R_act);

    // Solve T_rpy(Phi_act) * Phi_act_p = omega_d for Phi_act_p
    Phi_act_p = T_rpy<T>(Phi_act).colPivHouseholderQr().solve(omega_d);

    // Compute T_rpy_p(Phi_act, Phi_act_p) * Phi_act_p
    Eigen::Vector3d T_rpy_p_times_Phi_act_p = T_rpy_p<T>(Phi_act, Phi_act_p) * Phi_act_p;

    // Solve T_rpy(Phi_act) * Phi_act_pp = omega_d_p - T_rpy_p(Phi_act, Phi_act_p) * Phi_act_p for Phi_act_pp
    Phi_act_pp = T_rpy<T>(Phi_act).colPivHouseholderQr().solve(omega_d_p - T_rpy_p_times_Phi_act_p);

    Eigen::VectorXd result(9);
    result << Phi_act, Phi_act_p, Phi_act_pp;
    return result;
}

template <typename T = double>
Eigen::MatrixXd T_ext_inv(const Eigen::Vector3d& Phi) {
    Eigen::MatrixXd m(6, 6);

    T phi   = Phi(0);
    T theta = Phi(1);
    
    // First 3 rows: identity block
    m << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         
    // Fourth row
         0, 0, 0, cos(phi)*sin(theta)/cos(theta), sin(phi)*sin(theta)/cos(theta), 1,
         
    // Fifth row
         0, 0, 0, -sin(phi), cos(phi), 0,
         
    // Sixth row
         0, 0, 0, cos(phi)/cos(theta), sin(phi)/cos(theta), 0;
    
    return m;
}

template <typename T = double>
Eigen::MatrixXd dT_ext(const Eigen::Vector3d& Phi, const Eigen::Vector3d& Phi_p) {
    static_assert(std::is_floating_point<T>::value, "Template parameter must be a floating-point type");

    Eigen::MatrixXd dT_mat(6, 6);
    dT_mat.setZero(); // Initialize to a zero matrix

    // Compute the T_rpy_p matrix
    Eigen::Matrix3d T_rpy_p_mat = T_rpy_p<T>(Phi, Phi_p);

    // Populate the lower-right block with T_rpy_p
    dT_mat.block<3, 3>(3, 3) = T_rpy_p_mat;

    return dT_mat;
}

template <typename T = double>
Eigen::Matrix3d quat2rotm(Eigen::Matrix<T, 4, 1> q)
{
    static_assert(std::is_floating_point<T>::value, "Template parameter must be a floating-point type");

    // Extract quaternion components
    const T q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    // Precompute reusable terms
    const T q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
    const T q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    const T q1q2 = q1 * q2, q1q3 = q1 * q3, q2q3 = q2 * q3;

    // Directly initialize the rotation matrix using Eigen's comma initializer
    Eigen::Matrix<T, 3, 3> R;
    R << (q0q0 + q1q1 - q2q2 - q3q3), 2.0 * (q1q2 - q0q3), 2.0 * (q1q3 + q0q2),
        2.0 * (q1q2 + q0q3), (q0q0 - q1q1 + q2q2 - q3q3), 2.0 * (q2q3 - q0q1),
        2.0 * (q1q3 - q0q2), 2.0 * (q2q3 + q0q1), (q0q0 - q1q1 - q2q2 + q3q3);

    return R;
}

template<typename T = double>
Eigen::Quaternion<T> vec2quat(const Eigen::Matrix<T, 4, 1>& q) {
    // Ensure q is a 4-dimensional vector
    static_assert(Eigen::Matrix<T, 4, 1>::RowsAtCompileTime == 4,
                  "Input must be a 4-element vector (w, x, y, z).");

    // Extract components
    T w = q(0); // First element (w)
    T x = q(1); // Second element (x)
    T y = q(2); // Third element (y)
    T z = q(3); // Fourth element (z)

    // Create a quaternion using Eigen's constructor
    return Eigen::Quaternion<T>(w, x, y, z);
}

template <typename State, typename Derivative>
class RK4Integrator {
public:
    /**
     * @brief Perform a single RK4 integration step.
     * 
     * @param state Current state of the system (e.g., positions and velocities).
     * @param dt Time step for integration.
     * @param computeDerivative A function that computes the derivative of the state.
     *        It should have the signature:
     *        `Derivative computeDerivative(const State& state, double t)`
     *        where:
     *          - `state` is the current state.
     *          - `t` is the current time.
     * @return The updated state after one RK4 step.
     */
    static State integrate(const State& state, double dt,
                           const std::function<Derivative(const State&, double)>& computeDerivative) {
        // Compute k1 (initial derivative)
        Derivative k1 = computeDerivative(state, 0.0);

        // Compute k2 (midpoint derivative using k1)
        Derivative k2 = computeDerivative(state + 0.5 * dt * k1, 0.5 * dt);

        // Compute k3 (midpoint derivative using k2)
        Derivative k3 = computeDerivative(state + 0.5 * dt * k2, 0.5 * dt);

        // Compute k4 (final derivative using k3)
        Derivative k4 = computeDerivative(state + dt * k3, dt);

        // Update the state using weighted sum of derivatives
        return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
};

// Template function for constructing the Q matrix for d/dt quat = Q(quat)
template <typename T = double>
Eigen::Matrix<T, 4, 3> constructQ(const Eigen::Quaternion<T>& quat) {
    Eigen::Matrix<T, 4, 3> Q_mat;
    Eigen::Matrix<T, 3, 1> v = quat.vec();
    T w = quat.w();

    Q_mat << -v.x(), -v.y(), -v.z(),
              w, v.z(), -v.y(),
             -v.z(), w, v.x(),
              v.y(), -v.x(), w;

    return 0.5 * Q_mat;
}

template <typename T = double>
Eigen::Matrix<T, 4, 3> constructQ(const Eigen::Vector4d& quat) {
    Eigen::Matrix<T, 4, 3> Q_mat;
    Eigen::Vector3d v(quat(1), quat(2), quat(3));
    T w = quat(0);

    Q_mat << -v.x(), -v.y(), -v.z(),
              w, v.z(), -v.y(),
             -v.z(), w, v.x(),
              v.y(), -v.x(), w;

    return 0.5 * Q_mat;
}

template <typename T = double>
Eigen::Vector4d d_dt_quat(const Eigen::Vector4d& quat, const Eigen::Vector3d& omega) {
    return constructQ(quat) * omega;
}

template <typename T = double>
Eigen::Vector4d d_dt2_quat(const Eigen::Vector4d& quat, const Eigen::Vector3d& omega, const Eigen::Vector3d& omega_p) {
    return constructQ(d_dt_quat(quat, omega)) * omega + constructQ(quat) * omega_p;
}

#endif // EIGEN_TEMPLATES_HPP