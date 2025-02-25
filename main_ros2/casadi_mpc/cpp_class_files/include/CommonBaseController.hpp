#ifndef COMMON_BASE_CONTROLLER_HPP
#define COMMON_BASE_CONTROLLER_HPP

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry> // For rotations and quaternions

#include "param_robot.h"
#include "eigen_templates.hpp"
#include "error_flags.h"
#include "json.hpp"
#include "RobotModel.hpp"
#include "TrajectoryGenerator.hpp"

class CommonBaseController
{
public:
    // Constructor

    CommonBaseController(const std::string &urdf_filename,
                         const std::string &mpc_config_filename,
                         const std::string &general_config_filename);

    void simulateModelEuler(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt);
    void simulateModelRK4(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt);
    void error_check(Eigen::VectorXd &tau_full);
    void reset_error_flag()
    {
        error_flag = ErrorFlag::NO_ERROR;
    }
    void init_file_trajectory(uint traj_select, const double *x_k_ndof_ptr,
                              double T_start, double T_poly, double T_end);
    void init_custom_trajectory(ParamPolyTrajectory param_target);

    ErrorFlag get_error_flag()
    {
        return error_flag;
    }

    const Eigen::MatrixXd *get_trajectory()
    {
        return trajectory_generator.get_traj_data();
    }

    uint get_traj_data_real_len()
    {
        return trajectory_generator.get_traj_data_real_len();
    }

    uint get_transient_traj_len()
    {
        return trajectory_generator.get_transient_traj_len();
    }

    // Method to get the length of the trajectory data
    uint get_traj_data_len()
    {
        return trajectory_generator.get_traj_data_len();
    }

    const double *get_traj_x0_init(uint traj_select)
    {
        return trajectory_generator.get_traj_file_x0_init(traj_select)->data();
    }

    FullSystemTorqueMapper* get_torque_mapper()
    {
        return &torque_mapper;
    }

    void set_tau_max_jump(double tau_jump)
    {
        tau_max_jump = tau_jump;
    }

    void set_torque_mapper_config(FullSystemTorqueMapper::Config &config)
    {
        torque_mapper.setConfiguration(config);
    }

    Eigen::VectorXd get_file_traj_x0_nq_init(uint traj_select);
    Eigen::VectorXd get_transient_traj_x0_red_init();

    // NEW
    virtual Eigen::VectorXd update_control(const Eigen::VectorXd &x_nq) = 0; // W: update, Ca, Cr: solveMPC
    // virtual void switch_controller(uint controller_select) = 0; // W: switch_controller, Ca, Cr: setActiveMPC
    virtual void update_config() = 0; // W: set_controller_settings, Ca, Cr: update_mpc_weights

    // EXISTING
    virtual void increase_traj_count() = 0;
    virtual void switch_traj(uint traj_select) = 0;
    virtual void reset() = 0;
    virtual void reset(const double *const x_k_in) = 0;
    virtual void update_trajectory_data(const double *x_k_ndof_ptr) = 0;
    virtual uint get_traj_count() = 0;
    virtual uint get_traj_step() = 0;
    virtual uint get_N_step() = 0;
    virtual const double *get_act_traj_data() = 0; // verwirrender name, eher act traj sample
    
    virtual void set_traj_count(uint new_traj_count) = 0;

protected:
    const std::string &urdf_filename;
    const std::string &mpc_config_filename;
    const std::string &general_config_filename;

    robot_config_t robot_config;
    const Eigen::VectorXi n_indices, n_x_indices;

    const casadi_uint nq, nx, nq_red, nx_red;

    RobotModel robot_model;
    FullSystemTorqueMapper torque_mapper;
    TrajectoryGenerator trajectory_generator;
    double tau_max_jump = 5.0; // Maximum jump in torque (Nm/dt)
    double dt;
    uint traj_data_real_len;

    ErrorFlag error_flag = ErrorFlag::NO_ERROR;

    Eigen::VectorXd tau_full_prev = Eigen::VectorXd::Zero(nq);

    
};

#endif // COMMON_BASE_CONTROLLER_HPP