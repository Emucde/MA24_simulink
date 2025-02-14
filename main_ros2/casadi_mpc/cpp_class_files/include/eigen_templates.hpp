#ifndef EIGEN_TEMPLATES_HPP
#define EIGEN_TEMPLATES_HPP

#include <Eigen/Dense>
#include <cmath>
#include <array>
#include <type_traits>


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
    T r22 = R(1, 1);
    T r23 = R(1, 2);
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

#endif // EIGEN_TEMPLATES_HPP