#ifndef CROCODDYL_MPC_HPP
#define CROCODDYL_MPC_HPP

#include "CommonBaseMPC.hpp"
#include "param_robot.h"
#include "robot_data.hpp"
#include <iostream>
#include "json.hpp"

#include <crocoddyl/core/solvers/ddp.hpp>
#include <crocoddyl/core/solvers/box-ddp.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/core/residuals/joint-acceleration.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/integrator/rk.hpp> // Include the RK Integrator
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/activations/weighted-quadratic-barrier.hpp>
#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <memory>

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
#include "CrocoddylBaseIntegrator.hpp"
#include "CrocoddylMPCType.hpp"

/*
This class implements the Crocoddyl MPC (Model Predictive Control) for the robot.
It inherits from CommonBaseMPC and provides methods to set up and solve the MPC problem.
*/
class CrocoddylMPC : public CommonBaseMPC
{
public:
    CrocoddylMPC(CrocoddylMPCType mpc_type,
                 RobotModel &robot_model,
                 const std::string &config_filename,
                 TrajectoryGenerator &trajectory_generator);

    CrocoddylMPCType mpc_type;
    std::shared_ptr<BaseCrocoddylIntegrator> create_integrator(const std::string &int_type);
    
    
    template <typename T>
    T get_param(const std::string &name)
    {
        return get_config_value<nlohmann::json>(param_mpc_weight, name).get<T>();
    }

    template <typename T>
    T get_setting(const std::string &name)
    {
        return get_config_value<nlohmann::json>(mpc_settings, name).get<T>();
    }

    template <typename T = Eigen::VectorXd,
              typename std::enable_if<std::is_same<T, Eigen::VectorXd>::value, int>::type = 0>
    Eigen::VectorXd get_param_vec(const std::string &name)
    {
        std::vector<double> vector_data = param_mpc_weight[name].get<std::vector<double>>();
        Eigen::VectorXd eigen_vector = Eigen::Map<Eigen::VectorXd>(vector_data.data(), vector_data.size());
        return eigen_vector;
    }

    void generate_trajectory_blocks();
    
    void create_mpc_solver();
    void set_references(const Eigen::VectorXd &x_k);
    bool solve(const Eigen::VectorXd &x_k);
    void reset(const Eigen::VectorXd &x_k) override;
    void set_coldstart_init_guess(const Eigen::VectorXd &x_k);
    void init_config();
    void switch_traj(const Eigen::VectorXd &x_k) override;

    double *get_optimal_control()
    {
        return us_init_guess[0].data();
    }

    std::vector< Eigen::VectorXd > &get_u_opt_full()
    {
        return us_init_guess;
    }

private:

    nlohmann::json mpc_settings;
    nlohmann::json param_mpc_weight;
    double dt_MPC;
    std::string int_method;
    uint N_MPC, N_solver_steps;
    Eigen::VectorXd x_min, x_max, x_mean;
    Eigen::VectorXd u_min, u_max;
    std::vector< Eigen::VectorXd > xs_init_guess;
    std::vector< Eigen::VectorXd > us_init_guess;
    Eigen::VectorXi mpc_traj_indices; // MPC stepwidth indices for sampling trajectory data

    std::shared_ptr<crocoddyl::SolverAbstract> ddp;
    std::vector<std::map<std::string, std::shared_ptr<crocoddyl::CostItem>>> cost_models;
    std::vector<std::map<std::string, std::shared_ptr<crocoddyl::ResidualModelState>>> residual_state_models;
    std::vector<std::map<std::string, std::shared_ptr<crocoddyl::ResidualModelJointAcceleration>>> residual_joint_acceleration_models;
    std::vector<std::map<std::string, std::shared_ptr<crocoddyl::ResidualModelControl>>> residual_control_models;
    std::vector<std::map<std::string, std::shared_ptr<crocoddyl::ResidualModelFrameTranslation>>> residual_frame_translation_models;
    std::vector<std::map<std::string, std::shared_ptr<crocoddyl::ResidualModelFrameRotation>>> residual_frame_rotation_models;
    std::shared_ptr<crocoddyl::ShootingProblem> problem_reference;

    const Eigen::Vector3i p_d_rows{0, 1, 2};
    const Eigen::Vector4i q_d_rows{9, 10, 11, 12};

    std::vector<Eigen::MatrixXd> p_d_blocks;
    std::vector<std::vector<Eigen::Matrix3d>> R_d_blocks;
public:
    bool is_kinematic;
};

#endif // CROCODDYL_MPC_HPP