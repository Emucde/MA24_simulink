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
    update_traj_data(1);
}

void TrajectoryGenerator::switch_traj(uint traj_select)
{
    if (traj_select < 1 || traj_select > all_traj_data_file.size())
    {
        std::cerr << "Invalid trajectory selection. Selecting Trajectory 1" << std::endl;
        traj_select = 1;
    }

    update_traj_data(traj_select);
}

void TrajectoryGenerator::setTransientTrajParams(ParamInitTrajectory param_init_traj)
{
    double T_start = param_init_traj.T_start;
    double T_poly = param_init_traj.T_poly;
    double T_end = param_init_traj.T_end;
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

    param_init_traj_poly = param_init_traj;
}

// In this case only a transient trajectory to a custom target is generated
void TrajectoryGenerator::init_trajectory_custom_target(ParamInitTrajectory param_init_traj,
                                                        ParamTargetTrajectory param_target,
                                                        double T_horizon_max)
{
    setTransientTrajParams(param_init_traj);
    param_target_traj_poly = param_target;
    if (param_init_traj_poly.T_end == 0)
    {
        throw std::runtime_error("Eigen::MatrixXd TrajectoryGenerator::init_trajectory_custom_target(): T_end must be greater than 0");
    }
    init_trajectory(true, T_horizon_max); // per default T_horizon_max = 2s
}

// Functions for generating a polynomial transient trajectory
void TrajectoryGenerator::init_trajectory(uint traj_select, ParamInitTrajectory param_init_traj)
{
    setTransientTrajParams(param_init_traj);
    init_trajectory(traj_select);
}

void TrajectoryGenerator::init_trajectory(uint traj_select)
{
    if (traj_select < 1 || traj_select > all_traj_data_file.size())
    {
        std::cerr << "Invalid trajectory selection. Selecting Trajectory 1" << std::endl;
        traj_select = 1;
    }

    selected_trajectory = traj_select;
    switch_traj(selected_trajectory);
    init_trajectory();
}

// Private Method!
void TrajectoryGenerator::init_trajectory(bool custom_traj, double T_horizon_max)
{
    if (custom_traj)
    {
        traj_data_transient = Eigen::MatrixXd::Zero(traj_rows, 0);
        traj_data_transient_len = 0;

        traj_data_out = generate_transient_trajectory(T_horizon_max);
        traj_data_out_x0_init = param_init_traj_poly.x_init;

        traj_data_out_len = traj_data_out.cols();
        traj_data_file_real_len = robot_config.traj_data_real_len;
    }
    else
    {
        if (param_init_traj_poly.T_end == 0) // only selected trajectory from file is used
        {
            traj_data_transient = Eigen::MatrixXd::Zero(traj_rows, 0);
            traj_data_transient_len = 0;

            traj_data_out = traj_data_file;
            traj_data_out_x0_init = traj_data_file_x0_init;

            traj_data_out_len = traj_data_out.cols();
            traj_data_file_real_len = robot_config.traj_data_real_len;
        }
        else // create transient trajectory and prepend it to selected trajectory from file
        {
            traj_data_transient = generate_transient_trajectory();
            traj_data_transient_len = traj_data_transient.cols();

            traj_data_out.resize(traj_rows, traj_data_transient.cols() + traj_data_file.cols());
            traj_data_out << traj_data_transient, traj_data_file;
            traj_data_out_x0_init = param_init_traj_poly.x_init;

            traj_data_out_len = traj_data_out.cols();
            traj_data_file_real_len = robot_config.traj_data_real_len + traj_data_transient.cols();
        }
    }
    update_traj_data();
}

ParamInitTrajectory TrajectoryGenerator::calc_param_init(const casadi_real *x_k_ndof_ptr, double T_start, double T_poly, double T_end)
{
    Eigen::Vector3d p_init;
    Eigen::Matrix3d R_init;
    Eigen::Map<const Eigen::VectorXd> q_init_nq(x_k_ndof_ptr, nq);
    Eigen::Map<const Eigen::VectorXd> x_init_nq(x_k_ndof_ptr, nx);

    torque_mapper.calcPose(q_init_nq, p_init, R_init);

    ParamInitTrajectory param_init_traj = {T_start, T_poly, T_end, x_init_nq, p_init, R_init};

    return param_init_traj;
}

Eigen::MatrixXd TrajectoryGenerator::generate_transient_trajectory(double T_horizon_max)
{
    Eigen::Map<const Eigen::Matrix3d> R_init(param_init_traj_poly.R_init.data());
    Eigen::Map<const Eigen::Matrix3d> R_target(param_target_traj_poly.R_target.data());

    double T_end = param_init_traj_poly.T_end;

    if (T_end == 0)
    {
        throw std::runtime_error("Eigen::MatrixXd TrajectoryGenerator::generate_transient_trajectory(): T_end must be greater than 0");
    }

    // Per default T_horizon_max is already considered in the trajectory from file, see 'create_trajectories.m'
    // Only for the custom target trajectory it is important because extra samples for the
    // last prediction horizon are necessary to calulate all MPC control values
    int N = static_cast<int>((T_end + T_horizon_max) / dt);
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
        Eigen::VectorXd x_d = create_poly_traj(current_time, rot_ax, rot_alpha_scale);
        traj_data_temp.col(i) = x_d;
    }

    return traj_data_temp;
}

// This function generates a 5th order polynomial trajectory
Eigen::MatrixXd TrajectoryGenerator::trajectory_poly(double t, const Eigen::Vector4d &y0, const Eigen::Vector4d &yT, double T)
{
    double t_T = t / T;
    double t_T2 = t_T * t_T;
    double t_T3 = t_T2 * t_T;
    double t_T4 = t_T3 * t_T;
    double t_T5 = t_T4 * t_T;

    Eigen::Vector4d y_d = y0 + (6.0 * t_T5 - 15.0 * t_T4 + 10.0 * t_T3) * (yT - y0);
    Eigen::Vector4d y_d_p = (30.0 * t_T4 - 60.0 * t_T3 + 30.0 * t_T2) / T * (yT - y0);
    Eigen::Vector4d y_d_pp = (120.0 * t_T3 - 180.0 * t_T2 + 60.0 * t_T) / (T * T) * (yT - y0);

    Eigen::MatrixXd result(4, 3);
    result << y_d, y_d_p, y_d_pp;
    return result;
}

Eigen::VectorXd TrajectoryGenerator::create_poly_traj(double t, const Eigen::Vector3d &rot_ax, double rot_alpha_scale)
{
    // , param_init_traj_poly, param_target_traj_poly
    Eigen::Map<const Eigen::VectorXd> p_init(param_init_traj_poly.p_init.data(), 3);
    Eigen::Map<const Eigen::VectorXd> p_target(param_target_traj_poly.p_target.data(), 3);

    Eigen::Map<const Eigen::Matrix3d> R_init(param_init_traj_poly.R_init.data());
    Eigen::Map<const Eigen::Matrix3d> R_target(param_target_traj_poly.R_target.data());

    double T_start = param_init_traj_poly.T_start;
    double T_poly = param_init_traj_poly.T_poly;

    Eigen::MatrixXd yy_d;

    if (t - T_start < 0)
    {
        yy_d = trajectory_poly(0,
                              Eigen::Vector4d(p_init[0], p_init[1], p_init[2], 0.0),
                              Eigen::Vector4d(p_target[0], p_target[1], p_target[2], rot_alpha_scale),
                              T_poly);
    }
    else if (t - T_start > T_poly)
    {
        yy_d = trajectory_poly(T_poly,
                              Eigen::Vector4d(p_init[0], p_init[1], p_init[2], 0.0),
                              Eigen::Vector4d(p_target[0], p_target[1], p_target[2], rot_alpha_scale),
                              T_poly);
    }
    else
    {
        yy_d = trajectory_poly(t - T_start,
                              Eigen::Vector4d(p_init[0], p_init[1], p_init[2], 0.0),
                              Eigen::Vector4d(p_target[0], p_target[1], p_target[2], rot_alpha_scale),
                              T_poly);
    }

    double alpha = yy_d(3, 0), alpha_p = yy_d(3, 1), alpha_pp = yy_d(3, 2);

    Eigen::Matrix3d skew_ew;
    skew_ew << 0, -rot_ax[2], rot_ax[1],
        rot_ax[2], 0, -rot_ax[0],
        -rot_ax[1], rot_ax[0], 0;

    Eigen::Matrix3d R_act = (Eigen::Matrix3d::Identity() + sin(alpha) * skew_ew +
                             (1 - cos(alpha)) * skew_ew * skew_ew) *
                            R_init;

    Eigen::Quaterniond q_d(R_act);

    omega_d   = alpha_p*rot_ax;
    omega_d_p = alpha_pp*rot_ax;

    Eigen::VectorXd result(19); // p_d, p_d_p, p_d_pp, q_d, omega_d, omega_d_p
    result << yy_d.topRows(3), yy_d.block(3, 0, 3, 3), yy_d.block(3, 1, 3, 3), q_d.w(), q_d.x(), q_d.y(), q_d.z(), omega_d, omega_d_p;

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

// Without transient trajectory
void TrajectoryGenerator::update_traj_data(uint traj_select)
{
    selected_trajectory = traj_select;
    traj_data_file = all_traj_data_file[selected_trajectory - 1];
    traj_data_file_x0_init = all_traj_data_x0_init[selected_trajectory - 1];

    traj_rows = traj_data_file.rows();
    traj_data_out_len = traj_data_file.cols();
    traj_data_out.resize(traj_rows, traj_data_out_len);
    traj_data_out << traj_data_file;

    update_traj_data();

    // It is suggested that no transient trajectory is used
    // therefore traj_data_transient is set to zero
    // and traj_data_transient_len is set to zero

    traj_data_out_x0_init = traj_data_file_x0_init;
    traj_data_file_real_len = robot_config.traj_data_real_len;
    traj_data_transient_len = 0;

    param_init_traj_poly.T_start = 0.0;
    param_init_traj_poly.T_poly = 0.0;
    param_init_traj_poly.T_end = 0.0;

    Eigen::VectorXd p_d_0 = p_d.col(0); // Get the first point
    Eigen::Quaterniond q_d_0(q_d(3, 0), q_d(0, 0), q_d(1, 0), q_d(2, 0)); // Get the first quaternion [w,x,y,z]

    // If a transient trajectory is still generated, only the beginning is held constant.
    param_target_traj_poly.p_target = p_d_0;
    param_target_traj_poly.R_target = q_d_0.toRotationMatrix();

    param_init_traj_poly.p_init = p_d_0;
    param_init_traj_poly.R_init = q_d_0.toRotationMatrix();
}

void TrajectoryGenerator::update_traj_data()
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