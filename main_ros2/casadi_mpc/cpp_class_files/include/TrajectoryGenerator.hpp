#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include "param_robot.h"
#include "eigen_templates.hpp"
#include "FullSystemTorqueMapper.hpp"
#include <vector>
#include <map>
#include <string>
#include <Eigen/Dense>

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(FullSystemTorqueMapper &torque_mapper, double dt);

    void init_trajectory(int traj_select, const double *x_k_ndof_ptr, double T_start, double T_poly, double T_end);
    void init_trajectory(int traj_select, const double *x_k_ndof_ptr);
    void init_trajectory(int traj_select);
    void setTransientTrajParams(double T_start, double T_poly, double T_end);
    void switch_traj(int traj_select);

    // Getters
    const Eigen::MatrixXd *get_traj_data() const { return &traj_data; }
    const Eigen::VectorXd *get_traj_x0_init() const { return &traj_x0_init; }
    const Eigen::MatrixXd *get_transient_traj_data() const { return &transient_traj_data; }
    const Eigen::VectorXd *get_act_traj_x0_init() const { return &traj_file_x0_init; }
    const Eigen::VectorXd *get_traj_file_x0_init(int traj_select) const { 
        if (traj_select < 1 || traj_select > all_traj_x0_init.size())
        {
            std::cerr << "Invalid trajectory selection. Selecting Trajectory 1" << std::endl;
            traj_select = 1;
        }
        return &all_traj_x0_init[traj_select-1];
    }
    int get_traj_data_len() const { return traj_len; }
    int get_traj_file_real_len() const { return robot_config.traj_data_real_len; }
    int get_traj_data_real_len() const { return traj_real_len; }
    int get_transient_traj_len() const { return transient_traj_len; }

    // Setters

private:
    Eigen::Vector4d trajectory_poly(double t, const Eigen::Vector4d &y0, const Eigen::Vector4d &yT, double T);
    Eigen::VectorXd create_poly_traj(const Eigen::Vector3d &yT, const Eigen::Vector3d &y0, double t,
                                     const Eigen::Matrix3d &R_init, const Eigen::Vector3d &rot_ax,
                                     double rot_alpha_scale, const std::map<std::string, double> &param_traj_poly);
    Eigen::MatrixXd generate_trajectory(double dt, const Eigen::Vector3d &xe0, const Eigen::Vector3d &xeT,
                                        const Eigen::Matrix3d &R_init, const Eigen::Matrix3d &R_target,
                                        const std::map<std::string, double> &param_traj_poly);
    std::vector<Eigen::MatrixXd> readTrajectoryData(const std::string &traj_file);
    std::vector<Eigen::VectorXd> read_x0_init(const std::string &x0_init_file);

    // Member variables
    FullSystemTorqueMapper &torque_mapper;
    robot_config_t &robot_config;
    int nq, nx, nq_red, nx_red;
    std::string traj_file, x0_init_file;
    std::vector<Eigen::MatrixXd> all_traj_data;
    std::vector<Eigen::VectorXd> all_traj_x0_init;
    Eigen::MatrixXd traj_data, transient_traj_data;
    Eigen::VectorXd traj_file_x0_init; // Initial state of the trajectory from file "traj_file" in "x0_init_file"
    Eigen::VectorXd traj_x0_init;      // True initial state of total trajectory (including transient trajectory)
                                       // - in case of no transient trajectory it is equal to traj_file_x0_init
    int selected_trajectory, traj_rows, traj_len, traj_real_len, transient_traj_cnt, transient_traj_len;
    double dt;
    std::map<std::string, double> param_transient_traj_poly;

    void generate_transient_trajectory(const double *const x_k_ndof_ptr, double T_start, double T_poly, double T_end);
    void generate_transient_trajectory(const double *const x_k_ndof_ptr);
};

#endif // TRAJECTORY_GENERATOR_HPP
