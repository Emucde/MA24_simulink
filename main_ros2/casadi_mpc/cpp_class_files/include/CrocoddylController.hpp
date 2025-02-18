#ifndef CROCODYL_CONTROLLER_HPP
#define CROCODYL_CONTROLLER_HPP

#include "param_robot.h"
#include "robot_data.hpp"
#include <iostream>
#include "json.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> // For rotations and quaternions
#include <vector>
#include "eigen_templates.hpp"
#include "error_flags.h"

#include "RobotModel.hpp"
#include "TrajectoryGenerator.hpp"
#include "CrocoddylMPC.hpp"
#include "CrocoddylMPCType.hpp"

class CrocoddylController
{
public:
    CrocoddylController(const std::string &urdf_path,
                        const std::string &crocoddyl_config_path,
                        const std::string &tcp_frame_name,
                        bool use_gravity);

    Eigen::VectorXd solveMPC(const double *const x_k_ndof_ptr);

    // Initialize trajectory data
    void init_file_trajectory(uint traj_select, const double *x_k_ndof_ptr,
                              double T_start, double T_poly, double T_end);

    // Method for creating a custom trajectory with extra samples for the last prediction horizon
    void init_custom_trajectory(ParamPolyTrajectory param);

    void switch_traj(uint traj_select);

    void update_trajectory_data(const double *x_k_ndof_ptr);

    void setActiveMPC(CrocoddylMPCType mpc_type);

    // Method to simulate the robot model
    void simulateModelEuler(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt);
    void simulateModelRK4(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt);

    void increase_traj_count()
    {
        active_mpc->increase_traj_count();
    }

    uint get_traj_data_len()
    {
        return trajectory_generator.get_traj_data_len();
    }

    uint get_traj_data_real_len()
    {
        return trajectory_generator.get_traj_data_real_len();
    }

    uint get_transient_traj_len()
    {
        return trajectory_generator.get_transient_traj_len();
    }

    const double *get_act_traj_data()
    {
        return active_mpc->get_act_traj_data();
    }

    const double *get_traj_x0_init(uint traj_select)
    {
        return trajectory_generator.get_traj_file_x0_init(traj_select)->data();
    }

    ErrorFlag get_error_flag()
    {
        return error_flag;
    }

    void set_torque_mapper_config(FullSystemTorqueMapper::Config &config)
    {
        torque_mapper.setConfiguration(config);
    }

    ~CrocoddylController() {};

private:
    const std::string urdf_path;
    const std::string crocoddyl_config_path;
    const std::string tcp_frame_name;
    robot_config_t robot_config;
    const Eigen::VectorXi n_indices, n_x_indices;
    const int nq, nx, nq_red, nx_red;
    RobotModel robot_model;
    FullSystemTorqueMapper torque_mapper;
    TrajectoryGenerator trajectory_generator;

    std::vector<CrocoddylMPC> crocoddyl_mpcs;
    CrocoddylMPC *active_mpc;
    CrocoddylMPCType selected_mpc_type;

    double *u_k_ptr;

    ErrorFlag error_flag = ErrorFlag::NO_ERROR;
    Eigen::VectorXd tau_full_prev = Eigen::VectorXd::Zero(nq);
    double tau_max_jump = 5.0; // Maximum jump in torque (Nm/dt)
};

#endif // CROCODYL_CONTROLLER_HPP