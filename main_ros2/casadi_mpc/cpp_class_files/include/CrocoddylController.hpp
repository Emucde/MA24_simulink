#ifndef CROCODYL_CONTROLLER_HPP
#define CROCODYL_CONTROLLER_HPP

#include "CommonBaseController.hpp"
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

class CrocoddylController : public CommonBaseController
{
public:
    CrocoddylController(const std::string &urdf_filename,
                        const std::string &mpc_config_filename,
                        const std::string &general_config_filename);

    Eigen::VectorXd update_control(const Eigen::VectorXd &x_nq) override;
    nlohmann::json read_config(std::string file_path);
    void update_mpc_weights();
    void update_config() override;

    void switch_traj(uint traj_select) override;

    void update_trajectory_data(const double *x_k_ndof_ptr) override;

    void setActiveMPC(CrocoddylMPCType mpc_type);

    // Method to simulate the robot model
    void reset(const casadi_real *const x_k_in) override;
    void reset() override;

    const double *get_act_traj_data() override
    {
        return active_mpc->get_act_traj_data();
    }

    std::vector< Eigen::VectorXd > &get_u_opt_full()
    {
        return active_mpc->get_u_opt_full();
    }

    uint get_traj_count() override
    {
        return active_mpc->get_traj_count();
    }

    uint get_traj_step() override
    {
        return active_mpc->get_traj_step();
    }

    uint get_N_step() override
    {
        return active_mpc->get_N_step();
    }

    CrocoddylMPCType get_controller_type()
    {
        return selected_mpc_type;
    }

    // increase counter from mpc if it solves too slow
    void increase_traj_count() override
    {
        active_mpc->increase_traj_count();
    }

    void set_traj_count(uint new_traj_count) override
    {
        active_mpc->set_traj_count(new_traj_count);
    }

    ~CrocoddylController() {};

private:
    std::vector<CrocoddylMPC> crocoddyl_mpcs;
    CrocoddylMPC *active_mpc;
    CrocoddylMPCType selected_mpc_type;

    double *u_k_ptr;
};

#endif // CROCODYL_CONTROLLER_HPP