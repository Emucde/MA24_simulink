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
    all_traj_data = readTrajectoryData(traj_file);
    all_traj_x0_init = read_x0_init(x0_init_file);

    selected_trajectory = 1;
    traj_rows = all_traj_data[selected_trajectory-1].rows();
    traj_len = all_traj_data[selected_trajectory-1].cols();
    traj_data.resize(traj_rows, traj_len);
    traj_data << all_traj_data[selected_trajectory-1];
    traj_file_x0_init = all_traj_x0_init[selected_trajectory-1];
    traj_x0_init = traj_file_x0_init;

    traj_real_len = robot_config.traj_data_real_len;

    setTransientTrajParams(0.0, 0.0, 0.0);
    transient_traj_cnt = 0;
    transient_traj_len = 0;
    traj_rows = 7;
}

void TrajectoryGenerator::init_trajectory(int traj_select, const double *x_k_ndof_ptr, double T_start, double T_poly, double T_end)
{
    setTransientTrajParams(T_start, T_poly, T_end);
    init_trajectory(traj_select, x_k_ndof_ptr);
}

void TrajectoryGenerator::init_trajectory(int traj_select, const double *x_k_ndof_ptr)
{
    if (traj_select < 1 || traj_select > all_traj_data.size())
    {
        std::cerr << "Invalid trajectory selection. Selecting Trajectory 1" << std::endl;
        traj_select = 1;
    }

    selected_trajectory = traj_select;
    traj_file_x0_init = all_traj_x0_init[selected_trajectory-1];

    if (x_k_ndof_ptr != nullptr)
    {
        generate_transient_trajectory(x_k_ndof_ptr);
    }
    else
    {
        transient_traj_data = Eigen::MatrixXd::Zero(traj_rows, 0);
        x_k_ndof_ptr = traj_file_x0_init.data();
    }

    traj_data.resize(traj_rows, transient_traj_data.cols() + all_traj_data[traj_select - 1].cols());
    traj_data << transient_traj_data, all_traj_data[traj_select - 1];
    traj_len = traj_data.cols();
    traj_real_len = robot_config.traj_data_real_len + transient_traj_data.cols();

    traj_x0_init = Eigen::Map<const Eigen::VectorXd>(x_k_ndof_ptr, nx);
}

void TrajectoryGenerator::init_trajectory(int traj_select)
{
    init_trajectory(traj_select, nullptr);
}

void TrajectoryGenerator::generate_transient_trajectory(const double *const x_k_ndof_ptr, double T_start, double T_poly, double T_end)
{
    setTransientTrajParams(T_start, T_poly, T_end);
    generate_transient_trajectory(x_k_ndof_ptr);
}

void TrajectoryGenerator::generate_transient_trajectory(const double *const x_k_ndof_ptr)
{
    if (x_k_ndof_ptr == nullptr)
    {
        throw std::invalid_argument("TrajectoryGenerator::generate_transient_trajectory(const double *const x_k_ndof_ptr): x_k_ndof_ptr is a nullptr.");
    }

    Eigen::Vector3d p_init, p_target;
    Eigen::Matrix3d R_init, R_target;

    Eigen::Map<const Eigen::VectorXd> q_init_nq(x_k_ndof_ptr, nq);
    Eigen::Map<const Eigen::VectorXd> q_target_nq(traj_file_x0_init.data(), nq);

    torque_mapper.calcPose(q_init_nq, p_init, R_init);
    torque_mapper.calcPose(q_target_nq, p_target, R_target);

    transient_traj_data = generate_trajectory(dt, p_init, p_target, R_init, R_target, param_transient_traj_poly);
    transient_traj_len = transient_traj_data.cols();
    traj_rows = transient_traj_data.rows();
    transient_traj_cnt = 0;
}

void TrajectoryGenerator::setTransientTrajParams(double T_start, double T_poly, double T_end)
{
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

    param_transient_traj_poly["T_start"] = T_start;
    param_transient_traj_poly["T_poly"] = T_poly;
    param_transient_traj_poly["T_end"] = T_end;
}

void TrajectoryGenerator::switch_traj(int traj_select)
{
    if (traj_select < 1 || traj_select > all_traj_data.size())
    {
        std::cerr << "Invalid trajectory selection. Selecting Trajectory 1" << std::endl;
        traj_select = 1;
    }
    selected_trajectory = traj_select;
}

Eigen::Vector4d TrajectoryGenerator::trajectory_poly(double t, const Eigen::Vector4d &y0, const Eigen::Vector4d &yT, double T)
{
    Eigen::Vector4d y_d;

    double t_T = t / T;
    double t_T2 = t_T * t_T;
    double t_T3 = t_T2 * t_T;
    double t_T4 = t_T3 * t_T;
    double t_T5 = t_T4 * t_T;

    y_d = y0 + (6.0 * t_T5 - 15.0 * t_T4 + 10.0 * t_T3) * (yT - y0);
    return y_d;
}

Eigen::VectorXd TrajectoryGenerator::create_poly_traj(const Eigen::Vector3d &yT, const Eigen::Vector3d &y0, double t,
                                                      const Eigen::Matrix3d &R_init, const Eigen::Vector3d &rot_ax,
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

    Eigen::VectorXd result(7);
    result << p_d.head<3>(), q_d.w(), q_d.x(), q_d.y(), q_d.z();

    return result;
}

Eigen::MatrixXd TrajectoryGenerator::generate_trajectory(double dt, const Eigen::Vector3d &xe0, const Eigen::Vector3d &xeT,
                                                         const Eigen::Matrix3d &R_init, const Eigen::Matrix3d &R_target,
                                                         const std::map<std::string, double> &param_traj_poly)
{
    double T_end = param_traj_poly.at("T_end");

    int N = static_cast<int>(T_end / dt);
    Eigen::MatrixXd traj_data(7, N);

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
        Eigen::VectorXd x_d = create_poly_traj(xeT, xe0, current_time, R_init, rot_ax, rot_alpha_scale, param_traj_poly);
        traj_data.col(i) = x_d;
    }

    return traj_data;
}

std::vector<Eigen::MatrixXd> TrajectoryGenerator::readTrajectoryData(const std::string &traj_file)
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