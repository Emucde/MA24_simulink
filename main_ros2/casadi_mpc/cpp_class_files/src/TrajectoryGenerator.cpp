#include "TrajectoryGenerator.hpp"
#include "eigen_templates.hpp"
#include <iostream>
#include <fstream>
#include <stdexcept>

TrajectoryGenerator::TrajectoryGenerator(FullSystemTorqueMapper &torque_mapper, double dt)
    : torque_mapper(torque_mapper),
      robot_config(torque_mapper.getRobotConfig()),
      nq(robot_config.nq), nx(robot_config.nx), nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
      dt(dt)
{
    traj_file = robot_config.traj_data_path;
    x0_init_file = robot_config.x0_init_path;
    all_traj_data_file = read_trajectory_data(traj_file);
    all_traj_data_x0_init = read_x0_init(x0_init_file);
    
    selected_trajectory = 1; // Default trajectory selection
    traj_data_file = all_traj_data_file[selected_trajectory - 1];
    traj_data_file_x0_init = all_traj_data_x0_init[selected_trajectory - 1];

    traj_rows = traj_data_file.rows();
    traj_data_out_len = traj_data_file.cols();
    traj_data_out.resize(traj_rows, traj_data_out_len);
    traj_data_out << traj_data_file;
    update_traj_values();

    // It is suggested that no transient trajectory is used
    // therefore traj_data_transient is set to zero columns

    traj_data_out_x0_init = traj_data_file_x0_init;
    traj_data_file_real_len = robot_config.traj_data_real_len;

    param_poly_traj.T_start = 0.0;
    param_poly_traj.T_poly = 0.0;
    param_poly_traj.T_end = 0.0;

    param_poly_traj.p_init = traj_data_file(p_d_rows, 0);
    param_poly_traj.R_init = quat2rotm<double>(traj_data_file(q_d_rows, 0));;

    param_poly_traj.p_target = traj_data_file(p_d_rows, 0);
    param_poly_traj.R_target = quat2rotm<double>(traj_data_file(q_d_rows, 0));;
}

void TrajectoryGenerator::switch_traj(int traj_select)
{
    if (traj_select < 1 || traj_select > static_cast<int>(all_traj_data_file.size()))
    {
        std::cerr << "Invalid trajectory selection. Selecting Trajectory 1" << std::endl;
        traj_select = 1;
    }

    if (traj_select != selected_trajectory)
    {
        traj_data_file = all_traj_data_file[traj_select - 1];
        traj_data_file_x0_init = all_traj_data_x0_init[traj_select - 1];

        selected_trajectory = traj_select;
    }
}

void TrajectoryGenerator::check_param_poly_traj(ParamPolyTrajectory param)
{
    double T_start = param.T_start;
    double T_poly = param.T_poly;
    double T_end = param.T_end;
    if (T_start < 0)
    {
        throw std::invalid_argument("T_start must be greater than or equal to 0 (start time of the transient trajectory)");
    }
    if (T_start > T_poly)
    {
        throw std::invalid_argument("T_start must be less than or equal to T_poly (total time of the polynomial trajectory)");
    }
    if (T_poly > T_end)
    {
        throw std::invalid_argument("T_poly must be less than or equal to T_end (total time of the transient trajectory)");
    }
}

// In this case only a transient trajectory to a custom target is generated
void TrajectoryGenerator::init_custom_trajectory(ParamPolyTrajectory param)
{
    check_param_poly_traj(param);

    if (param.T_end == 0)
    {
        throw std::runtime_error("Eigen::MatrixXd TrajectoryGenerator::init_custom_trajectory(): T_end must be greater than 0");
    }

    traj_data_transient = Eigen::MatrixXd::Zero(traj_rows, 0);

    traj_data_out_x0_init = param.x_init;
    param_poly_traj = param; // param_poly_traj needed in generate_poly_trajectory

    traj_data_out = generate_poly_trajectory();

    traj_data_out_len = traj_data_out.cols();
    traj_data_file_real_len = traj_data_out_len - static_cast<int>(param_poly_traj.T_horizon_max / dt);

    update_traj_values();
}

void TrajectoryGenerator::init_file_trajectory(int traj_select,
                                               const double *x_k_ndof_ptr,
                                               double T_start, double T_poly, double T_end)
{
    switch_traj(traj_select);

    Eigen::Vector3d p_init;
    Eigen::Matrix3d R_init;

    torque_mapper.calcPose(x_k_ndof_ptr, p_init, R_init); // set p_init, R_init = fwdkin(q_init)

    param_poly_traj.T_start = T_start;
    param_poly_traj.T_poly = T_poly;
    param_poly_traj.T_end = T_end;
    param_poly_traj.T_horizon_max = 0;

    param_poly_traj.x_init = Eigen::Map<const Eigen::VectorXd>(x_k_ndof_ptr, nx);
    param_poly_traj.p_init = p_init;
    param_poly_traj.R_init = R_init;

    param_poly_traj.p_target = traj_data_file(p_d_rows, 0);
    param_poly_traj.R_target = quat2rotm<double>(traj_data_file(q_d_rows, 0));
    // prints
    std::cout << "traj_data_file(q_d_rows, 0)\n" << traj_data_file(q_d_rows, 0) << std::endl;
    std::cout << "R_init\n" << R_init << std::endl;
    std::cout << "param_poly_traj.R_target\n" << param_poly_traj.R_target << std::endl;

    check_param_poly_traj(param_poly_traj);

    // set param_poly_traj to begin of trajectory from file
    if (T_start == 0 && T_poly == 0 && T_end == 0) // no transient trajectory
    {
        traj_data_transient = Eigen::MatrixXd::Zero(traj_rows, 0);
    }
    else
    {
        // extra param_poly_traj.T_horizon_max samples for last prediction horizon are
        // already considered in the trajectory from file!
        traj_data_transient = generate_poly_trajectory();
    }

    traj_data_out.resize(traj_rows, traj_data_transient.cols() + traj_data_file.cols());
    traj_data_out << traj_data_transient, traj_data_file;
    traj_data_out_x0_init = param_poly_traj.x_init;

    traj_data_out_len = traj_data_out.cols();
    traj_data_file_real_len = robot_config.traj_data_real_len + traj_data_transient.cols();

    update_traj_values();
}

Eigen::MatrixXd TrajectoryGenerator::generate_poly_trajectory()
{
    Eigen::Map<const Eigen::Matrix3d> R_init(param_poly_traj.R_init.data());
    Eigen::Map<const Eigen::Matrix3d> R_target(param_poly_traj.R_target.data());

    std::cout << "R_init" << R_init << std::endl;
    std::cout << "R_target" << R_target << std::endl;

    double T_end = param_poly_traj.T_end;

    // Per default param_poly_traj.T_horizon_max is already considered in the trajectory from file, see 'create_trajectories.m'
    // Only for the custom target trajectory it is important because extra samples for the
    // last prediction horizon are necessary to calulate all MPC control values
    int N = static_cast<int>((T_end + param_poly_traj.T_horizon_max) / dt);
    Eigen::MatrixXd traj_data_temp(19, N); // p_d, p_d_p, p_d_pp, q_d, omega_d, omega_d_p

    Eigen::Matrix3d RR = R_target * R_init.transpose();
    Eigen::Quaterniond rot_quat(RR);

    Eigen::Vector3d rot_vec = rot_quat.vec();
    double rot_rho = rot_quat.w();

    double rot_alpha_scale = 2 * acos(rot_rho);
    Eigen::Vector3d rot_ax;

    if (rot_alpha_scale == 0)
    {
        rot_ax = Eigen::Vector3d(0, 0, 0);
    }
    else
    {
        rot_ax = rot_vec / sin(rot_alpha_scale / 2);
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
        Eigen::VectorXd x_d = get_poly_traj_point(current_time, rot_ax, rot_alpha_scale);
        traj_data_temp.col(i) = x_d;
    }

    return traj_data_temp;
}

// This function generates a 5th order polynomial trajectory
Eigen::VectorXd TrajectoryGenerator::poly_5th_order(double t, const Eigen::Vector4d &y0, const Eigen::Vector4d &yT, double T)
{
    double t_T = t / T;
    double t_T2 = t_T * t_T;
    double t_T3 = t_T2 * t_T;
    double t_T4 = t_T3 * t_T;
    double t_T5 = t_T4 * t_T;

    Eigen::Vector4d y_d = y0 + (6.0 * t_T5 - 15.0 * t_T4 + 10.0 * t_T3) * (yT - y0);
    Eigen::Vector4d y_d_p = (30.0 * t_T4 - 60.0 * t_T3 + 30.0 * t_T2) / T * (yT - y0);
    Eigen::Vector4d y_d_pp = (120.0 * t_T3 - 180.0 * t_T2 + 60.0 * t_T) / (T * T) * (yT - y0);

    Eigen::VectorXd result(12);
    result << y_d, y_d_p, y_d_pp;
    return result;
}

Eigen::VectorXd TrajectoryGenerator::get_poly_traj_point(double t, const Eigen::Vector3d &rot_ax, double rot_alpha_scale)
{
    Eigen::Map<const Eigen::VectorXd> p_init(param_poly_traj.p_init.data(), 3);
    Eigen::Map<const Eigen::VectorXd> p_target(param_poly_traj.p_target.data(), 3);

    Eigen::Map<const Eigen::Matrix3d> R_init(param_poly_traj.R_init.data());
    Eigen::Map<const Eigen::Matrix3d> R_target(param_poly_traj.R_target.data());

    double T_start = param_poly_traj.T_start;
    double T_poly = param_poly_traj.T_poly;

    Eigen::VectorXd yy_d;

    if (t - T_start < 0)
    {
        yy_d = poly_5th_order(0,
                              Eigen::Vector4d(p_init[0], p_init[1], p_init[2], 0.0),
                              Eigen::Vector4d(p_target[0], p_target[1], p_target[2], rot_alpha_scale),
                              T_poly);
    }
    else if (t - T_start > T_poly)
    {
        yy_d = poly_5th_order(T_poly,
                              Eigen::Vector4d(p_init[0], p_init[1], p_init[2], 0.0),
                              Eigen::Vector4d(p_target[0], p_target[1], p_target[2], rot_alpha_scale),
                              T_poly);
    }
    else
    {
        yy_d = poly_5th_order(t - T_start,
                              Eigen::Vector4d(p_init[0], p_init[1], p_init[2], 0.0),
                              Eigen::Vector4d(p_target[0], p_target[1], p_target[2], rot_alpha_scale),
                              T_poly);
    }

    double alpha = yy_d[3];
    double alpha_p = yy_d[7];
    double alpha_pp = yy_d[11];

    Eigen::Matrix3d skew_ew;
    skew_ew << 0, -rot_ax[2], rot_ax[1],
        rot_ax[2], 0, -rot_ax[0],
        -rot_ax[1], rot_ax[0], 0;

    Eigen::Matrix3d R_act = (Eigen::Matrix3d::Identity() + sin(alpha) * skew_ew +
                             (1 - cos(alpha)) * skew_ew * skew_ew) *
                            R_init;

    Eigen::Quaterniond q_d(R_act);

    // std::cout << "q_d: " << q_d.w() << " " << q_d.x() << " " << q_d.y() << " " << q_d.z() << std::endl;
    // std::cout << "alpha: " << alpha << std::endl;

    omega_d = alpha_p * rot_ax;
    omega_d_p = alpha_pp * rot_ax;

    Eigen::VectorXi pose_rows = (Eigen::VectorXi(9) << 0, 1, 2, 4, 5, 6, 8, 9, 10).finished();

    Eigen::VectorXd result(19); // p_d, p_d_p, p_d_pp, q_d, omega_d, omega_d_p
    result << yy_d(pose_rows), q_d.w(), q_d.x(), q_d.y(), q_d.z(), omega_d, omega_d_p;

    return result;
}

// Functions for reading trajectory from file

std::vector<Eigen::MatrixXd> TrajectoryGenerator::read_trajectory_data(const std::string &traj_file)
{
    std::ifstream file(traj_file, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Error opening file: " + traj_file);
    }

    uint32_t rows, cols, traj_amount;
    file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
    file.read(reinterpret_cast<char *>(&cols), sizeof(cols));
    file.read(reinterpret_cast<char *>(&traj_amount), sizeof(traj_amount));

    std::vector<Eigen::MatrixXd> trajectories(traj_amount, Eigen::MatrixXd(rows, cols));

    for (size_t i = 0; i < traj_amount; ++i)
    {
        file.read(reinterpret_cast<char *>(trajectories[i].data()), rows * cols * sizeof(double));
        if (!file)
        {
            throw std::runtime_error("Error reading trajectory data from file: " + traj_file);
        }
    }

    return trajectories;
}

std::vector<Eigen::VectorXd> TrajectoryGenerator::read_x0_init(const std::string &x0_init_file)
{
    std::ifstream file(x0_init_file, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Error opening file: " + x0_init_file);
    }

    uint32_t rows, cols;
    file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
    file.read(reinterpret_cast<char *>(&cols), sizeof(cols));

    std::vector<Eigen::VectorXd> all_x0_init;

    for (size_t i = 0; i < cols; ++i)
    {
        Eigen::VectorXd x0(rows);
        file.read(reinterpret_cast<char *>(x0.data()), rows * sizeof(double));
        if (!file)
        {
            throw std::runtime_error("Error reading initial condition data from file: " + x0_init_file);
        }
        all_x0_init.push_back(x0);
    }

    return all_x0_init;
}

void TrajectoryGenerator::update_traj_values()
{
    /*
        traj_data_file = [
        param_traj_data.p_d;
        param_traj_data.p_d_p;
        param_traj_data.p_d_pp;
        param_traj_data.q_d;
        param_traj_data.omega_d;
        param_traj_data.omega_d_p
        ];
    */
    p_d = traj_data_out.topRows(3);
    p_d_p = traj_data_out.block(3, 0, 3, traj_data_out.cols());
    p_d_pp = traj_data_out.block(6, 0, 3, traj_data_out.cols());
    q_d = traj_data_out.block(9, 0, 4, traj_data_out.cols());
    omega_d = traj_data_out.block(13, 0, 3, traj_data_out.cols());
    omega_d_p = traj_data_out.block(16, 0, 3, traj_data_out.cols());
}