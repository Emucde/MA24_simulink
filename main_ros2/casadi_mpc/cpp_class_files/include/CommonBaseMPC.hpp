#ifndef COMMON_BASE_MPC_HPP
#define COMMON_BASE_MPC_HPP

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "param_robot.h"
#include "RobotModel.hpp"
#include "TrajectoryGenerator.hpp"
#include "FullSystemTorqueMapper.hpp"
#include "error_flags.h"

class CommonBaseMPC
{
public:
    CommonBaseMPC(RobotModel &robot_model,
                  robot_config_t &robot_config,
                  const std::string &config_filename,
                  TrajectoryGenerator &trajectory_generator)
        : robot_model(robot_model), // Initialize with a null pointer
          robot_config(robot_config),
          config_filename(config_filename),
          trajectory_generator(trajectory_generator),
          nq(robot_config.nq), nx(robot_config.nx),
          nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
          traj_data(trajectory_generator.get_traj_data()),
          traj_data_real_len(trajectory_generator.get_traj_data_real_len()),
          traj_rows(trajectory_generator.get_traj_data()->rows()),
          traj_cols(trajectory_generator.get_traj_data()->cols()),
          traj_count(0), traj_select(1), N_step(1), dt(robot_config.dt)
    {
    }

    CommonBaseMPC(robot_config_t &robot_config,
                  TrajectoryGenerator &trajectory_generator) :
                  CommonBaseMPC(*(RobotModel *)nullptr, robot_config, "", trajectory_generator)
    {
    }

    casadi_uint get_traj_count()
    {
        return traj_count;
    }

    // if it solves too slow.
    void increase_traj_count()
    {
        traj_count++;
    }

    // if it solves too slow.
    void set_traj_count(casadi_uint new_traj_count)
    {
        traj_count = new_traj_count;
    }

    // Method to get the current trajectory data
    const casadi_real *get_act_traj_data()
    {
        return traj_data->col((traj_count > 0 ? traj_count - 1 : traj_count)).data();
    }

    casadi_uint get_N_step()
    {
        return N_step;
    }

    casadi_uint get_traj_step()
    {
        return traj_step;
    }

    // Overwrite Methods
    virtual void reset(const Eigen::VectorXd &x_k) = 0;
    virtual void switch_traj(const Eigen::VectorXd &x_k) = 0;

    // New functions
    // solve() / control() / update() / ...
    // get optimal control (evtl)
    // init_config
    RobotModel &robot_model;
    robot_config_t &robot_config;
    const std::string config_filename;
    TrajectoryGenerator &trajectory_generator;
    const uint nq, nx, nq_red, nx_red; // Number of joints, number of states, number of reduced joints, number of reduced states

    const Eigen::MatrixXd *traj_data; // Trajectory data
    casadi_uint traj_data_real_len;
    casadi_uint traj_step;
    casadi_uint traj_rows;
    casadi_uint traj_cols;
    casadi_uint traj_count;
    casadi_uint traj_select;
    casadi_uint N_step;
    double dt;
};

#endif // COMMON_BASE_MPC_HPP