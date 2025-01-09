#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy
#include <chrono>  // for time measurement

#include "include/pinocchio_utils.hpp"
#include "include/FullSystemTorqueMapper.hpp"
#include "include/CasadiMPC.hpp"
#include "include/CasadiController.hpp"
#include "param_robot.h"
#include "casadi_types.h"
#include <Eigen/Dense>

#define TRAJ_SELECT 1
// #define PLOT_DATA
#define TRANSIENT_TRAJ_TESTS

void set_x_k_ndof(casadi_real *const x_k_ndof_ptr, casadi_real *const x_k, const std::vector<casadi_real> &x_ref_nq_vec, casadi_uint nx, const casadi_uint *n_x_indices)
{
    int cnt = 0;
    for (casadi_uint i = 0; i < nx; ++i)
    {
        if (i == n_x_indices[cnt])
        {
            x_k_ndof_ptr[i] = x_k[cnt];
            cnt++;
        }
        else
        {
            x_k_ndof_ptr[i] = x_ref_nq_vec[i];
        }
    }
}

#include <Eigen/Dense>
#include <Eigen/Geometry> // For rotations and quaternions
#include <iostream>
#include <vector>

Eigen::Vector4d trajectory_poly(double t, const Eigen::Vector4d &y0, const Eigen::Vector4d &yT, double T)
{
    Eigen::Vector4d y_d;

    // y_d = y0 + (6.0 / (T * T * T * T * T) * t * t * t * t * t -
    //              15.0 / (T * T * T * T) * t * t * t * t +
    //              10.0 * t * t * t / (T * T * T)) * (yT - y0);

    double t_T = t / T;
    double t_T2 = t_T * t_T;
    double t_T3 = t_T2 * t_T;
    double t_T4 = t_T3 * t_T;
    double t_T5 = t_T4 * t_T;

    y_d = y0 + (6.0 * t_T5 - 15.0 * t_T4 + 10.0 * t_T3) * (yT - y0);
    return y_d;
}

Eigen::VectorXd create_poly_traj(const Eigen::Vector4d &yT, const Eigen::Vector4d &y0, double t,
                          const Eigen:: Matrix3d &R_init, const Eigen::Vector3d &rot_ax,
                          double rot_alpha_scale, const std::map<std::string, double> &param_traj_poly)
{
    double T_start = param_traj_poly.at("T_start");
    double T_poly = param_traj_poly.at("T_poly");
    Eigen::Vector4d p_d;

    if (t - T_start < 0)
    {
        p_d << y0[0], y0[1], y0[2], 0.0;
    }
    else if (t - T_start > T_poly)
    {
        p_d << yT[0], yT[1], yT[2], rot_alpha_scale;
    }
    else
    {
        p_d = trajectory_poly(t - T_start,
                              Eigen::Vector4d(y0[0], y0[1], y0[2], 0.0),
                              Eigen::Vector4d(yT[0], yT[1], yT[2], rot_alpha_scale),
                              T_poly);
    }

    double alpha = p_d[3];

    Eigen::Matrix3d skew_ew;
    skew_ew << 0, -rot_ax[2], rot_ax[1],
        rot_ax[2], 0, -rot_ax[0],
        -rot_ax[1], rot_ax[0], 0;

    Eigen::Matrix3d R_act = (Eigen::Matrix3d::Identity() + sin(alpha) * skew_ew +
                      (1 - cos(alpha)) * skew_ew * skew_ew) *
                     R_init;

    Eigen::Quaterniond q_d(R_act);

    // Create the final vector to hold both position and quaternion
    Eigen::VectorXd result(7);
    result << p_d.head<3>(), q_d.coeffs(); // q_d.coeffs() returns the quaternion in the form {x, y, z, w}

    return result;
}

Eigen::MatrixXd generate_trajectory(double dt, const Eigen::Vector4d &xe0, const Eigen::Vector4d &xeT,
                             const Eigen::Matrix3d &R_init, const Eigen::Matrix3d &R_target,
                             const std::map<std::string, double> &param_traj_poly)
{
    double T_end = param_traj_poly.at("T_end");

    int N = static_cast<int>(T_end / dt);
    Eigen::MatrixXd traj_data(7, N);

    Eigen::Matrix3d RR = R_target * R_init.transpose();
    Eigen::Quaterniond rot_quat(RR);

    // Convert quaternion to rotation matrix, then get axis and angle
    Eigen::Vector3d rot_vec = rot_quat.vec(); // The vector part (x, y, z)
    double rot_rho = rot_quat.w();     // The scalar part (w)

    double rot_alpha_scale = 2 * acos(rot_rho); // Angle from the quaternion
    Eigen::Vector3d rot_ax;

    if (rot_alpha_scale == 0)
    {
        rot_ax = Eigen::Vector3d(0, 0, 0); // Random axis because rotation angle is 0
    }
    else
    {
        rot_ax = rot_vec / sin(rot_alpha_scale / 2); // Normalize the axis
    }

    if (rot_alpha_scale > M_PI)
    {
        rot_alpha_scale = 2 * M_PI - rot_alpha_scale;
        rot_ax = -rot_ax;
    }

    double current_time = 0.0;

    for (int i = 0; i < N; i++)
    {
        current_time = i * dt;
        Eigen::VectorXd x_d = create_poly_traj(xeT, xe0, current_time, R_init, rot_ax, rot_alpha_scale, param_traj_poly);
        traj_data.col(i) = x_d;
    }

    return traj_data;
}

int main()
{
    // Configuration flags
    bool use_gravity = false;
    const std::string urdf_filename = "../../../urdf_creation/fr3_no_hand_7dof.urdf";

    CasadiController controller(urdf_filename, use_gravity);
    controller.setActiveMPC(MPCType::MPC8);
    controller.switch_traj(TRAJ_SELECT);

    const casadi_uint nq = controller.nq;
    const casadi_uint nx = controller.nx;
    const casadi_uint nq_red = controller.nq_red;
    const casadi_uint nx_red = controller.nx_red;
    const casadi_uint *n_x_indices_ptr = controller.get_n_x_indices();
    const casadi_uint traj_data_real_len = controller.get_traj_data_len();
    const std::vector<casadi_real> x_ref_nq_vec = controller.get_x_ref_nq();
    casadi_real *u_opt = controller.get_optimal_control();
    casadi_real *x_k = controller.get_x_k();
    casadi_real x_k_ndof[nx];
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);
    Eigen::VectorXd x_k_ndof_eig;

    // Measure execution time
    auto start = std::chrono::high_resolution_clock::now();

    set_x_k_ndof(x_k_ndof, x_k, x_ref_nq_vec, nx, n_x_indices_ptr);

#ifdef PLOT_DATA
    std::ofstream x_k_ndof_file("x_k_ndof_data.txt");
    std::ofstream tau_full_file("tau_full_data.txt");
#endif

#ifdef TRANSIENT_TRAJ_TESTS
    // Time step
    double dt = 0.01;

    // Initial and target states
    Eigen::Vector4d xe0(0.0, 0.0, 0.0, 0.0); // Starting at origin with no rotation
    Eigen::Vector4d xeT(1.0, 1.0, 1.0, 0.0); // Target point

    // Rotation matrices: identity and 90 degrees around z-axis
    Eigen::Matrix3d R_init = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_target;
    R_target = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix(); // 90 degrees around Z-axis

    // Trajectory parameters
    std::map<std::string, double> param_traj_poly;
    param_traj_poly["T_start"] = 0.0;
    param_traj_poly["T_poly"] = 1.0; // Duration for polynomial trajectory
    param_traj_poly["T_end"] = 2.0;  // Total duration

    // Generate the trajectory
    Eigen::MatrixXd trajectory = generate_trajectory(dt, xe0, xeT, R_init, R_target, param_traj_poly);

    // Output the trajectory results
    for (int i = 0; i < trajectory.cols(); ++i)
    {
        std::cout << "Step " << i << ": " << trajectory.col(i).transpose() << std::endl;
    }
#endif

    // Main loop for trajectory processing
    for (casadi_uint i = 0; i < traj_data_real_len; i++)
    {
        tau_full = controller.solveMPC(x_k_ndof);
        x_k_ndof_eig = Eigen::Map<Eigen::VectorXd>(x_k_ndof, nq);

        if (i % 100 == 0)
        {
            std::cout << "q_k: " << x_k_ndof_eig.transpose() << std::endl;
            // std::cout << "Full torque: " << tau_full.transpose() << std::endl;
        }

#ifdef PLOT_DATA
        x_k_ndof_file << x_k_ndof_eig.transpose() << std::endl;
        tau_full_file << tau_full.transpose() << std::endl;
#endif

        memcpy(x_k, u_opt + nq_red, nx_red * sizeof(casadi_real));
        set_x_k_ndof(x_k_ndof, x_k, x_ref_nq_vec, nx, n_x_indices_ptr);
    }
    // Measure and print execution time
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: " << (double)duration.count() / 1000000 << " s" << std::endl;

#ifdef PLOT_DATA
    x_k_ndof_file.close();
    tau_full_file.close();
#endif

    return 0;
}

/*
% Read the torque data
torque_data = dlmread('./main_ros2/read_bin_data/tau_full_data.txt');

% Read the state data
state_data = dlmread('./main_ros2/read_bin_data/x_k_ndof_data.txt');

% Create a figure with subplots
figure;

% Plot torque data
subplot(3,1,1);
plot(torque_data);
title('7-Dimensional Torque');
xlabel('Time step');
ylabel('Torque (Nm)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

% Plot joint angles
subplot(3,1,2);
plot(state_data(:,1:7));
title('Joint Angles');
xlabel('Time step');
ylabel('Angle (rad)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

% Plot joint velocities
subplot(3,1,3);
plot(state_data(:,8:14));
title('Joint Velocities');
xlabel('Time step');
ylabel('Velocity (rad/s)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

% Adjust the layout
sgtitle('Robot Joint Data');

*/