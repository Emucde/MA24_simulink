#ifndef WORKSPACE_CONTROLLER_HPP
#define WORKSPACE_CONTROLLER_HPP

#include "CommonBaseController.hpp"
#include "param_robot.h"
#include "robot_data.hpp"
#include "workspace_controller_config.hpp"
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

/*
This class implements a workspace controller for a robot model.
It provides methods for controlling the robot's movements in a workspace,
including calculating control data, updating state, and handling different controller types.
It uses the Pinocchio library for kinematics and dynamics calculations.
It supports different controller types such as CT, PDPlus, and InverseDynamics.
It also includes methods for trajectory generation and handling.
It is designed to be used in a robotics context, where the robot's state is updated based on joint positions and velocities.
The class is initialized with a URDF file and robot configuration, and it can handle both full and reduced models based on the provided configuration.
*/
class BaseWorkspaceController
{
public:
    BaseWorkspaceController(RobotModel &robot_model,
                   const std::string &general_config_filename,
                   TrajectoryGenerator &trajectory_generator,
                   double& tau_max_jump)
        : robot_model(robot_model), general_config_filename(general_config_filename),
          trajectory_generator(trajectory_generator),
          nq(robot_model.nq), nx(robot_model.nx),
          J(Eigen::MatrixXd::Zero(6, nq)), J_pinv(Eigen::MatrixXd::Zero(nq, 6)), J_p(Eigen::MatrixXd::Zero(6, nq)),
          M(Eigen::MatrixXd::Zero(nq, nq)), C(Eigen::MatrixXd::Zero(nq, nq)), C_rnea(Eigen::VectorXd::Zero(nq)),
          g(Eigen::VectorXd::Zero(nq)),
          q(Eigen::VectorXd::Zero(nq)), q_p(Eigen::VectorXd::Zero(nq)),
          x_err(Eigen::VectorXd::Zero(6)), x_err_p(Eigen::VectorXd::Zero(6)),
          x_d_p(Eigen::VectorXd::Zero(6)), x_d_pp(Eigen::VectorXd::Zero(6)),
          n_indices(ConstIntVectorMap(robot_model.robot_config.n_indices, robot_model.robot_config.nq_red)),
          traj_data_real_len(trajectory_generator.get_traj_data_real_len()), dt(robot_model.robot_config.dt),
          tau_max_jump(tau_max_jump), traj_count(0)
    {
        update_controller_settings();
    }
    virtual Eigen::VectorXd control(const Eigen::VectorXd &x) = 0; // Pure virtual function
    void update_controller_settings();
    Eigen::MatrixXd computeJacobianRegularization();       // Common method in BaseWorkspaceController
    void calculateControlData(const Eigen::VectorXd &x);   // Common method in BaseWorkspaceController
    void calculateControlDataRPY(const Eigen::VectorXd &x);   // Common method in BaseWorkspaceController
    void calculateControlDataID(const Eigen::VectorXd &x);   // Common method in BaseWorkspaceController
    void stateObserver(const Eigen::VectorXd &x); // Common method in BaseWorkspaceController
    void reset() {
        traj_count = 0;
        x_I_err.setZero();
        init = true;
    }
    void increment_traj_count()
    {
        if (traj_count < traj_data_real_len - 1)
        {
            traj_count++;
        }
    }
    virtual ~BaseWorkspaceController() = default; // Virtual destructor
protected:
    RobotModel &robot_model;
    std::string general_config_filename;
    ControllerSettings controller_settings;
    RegularizationSettings regularization_settings;
    RegularizationMode sing_method;
    TrajectoryGenerator &trajectory_generator;
    const int nq, nx; // this is nq_red and nx_red!!!
    Eigen::MatrixXd J, J_pinv, J_p;
    Eigen::MatrixXd M, C, C_rnea;
    Eigen::VectorXd g;
    Eigen::VectorXd q, q_p;
    bool init = true;
    Eigen::VectorXd x_e_p = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd x_I_err = Eigen::VectorXd::Zero(6), x_err = Eigen::VectorXd::Zero(6), x_err_p = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd x_d_p = Eigen::VectorXd::Zero(6), x_d_pp = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd column_weights = Eigen::VectorXd::Ones(nq);
    const Eigen::VectorXi n_indices;
    uint traj_data_real_len;
    double dt;
    double& tau_max_jump;

public:
    Eigen::VectorXd q_d, q_p_d;
    uint traj_count;
};


/*
This class implements a general workspace controller for a robot model.
It inherits from CommonBaseController and provides methods for controlling the robot's movements in a workspace.
The used workspace controller can be CT, PDPlus, or InverseDynamics and is based on the class BaseWorkspaceController.
The class BaseWorkspaceController provides the common functionality for all workspace controllers.
*/
class WorkspaceController : public CommonBaseController
{
public:
    // Constructor
    WorkspaceController(const std::string &urdf_filename,
                        const std::string &general_config_filename);

    ControllerType get_classic_controller_type(const std::string &controller_type_str);
    std::string get_classic_controller_string(ControllerType controller_type);
    void switchController(ControllerType type);
    void update_controller_settings()
    {
        ct_controller.update_controller_settings();
        pd_plus_controller.update_controller_settings();
        inverse_dyn_controller.update_controller_settings();
    }
    void update_trajectory_data(const double *x_k_ndof_ptr) override;
    void update_config() override;

    void increase_traj_count() override
    {
        active_controller->traj_count++;
    }

    void set_traj_count(casadi_uint new_traj_count) override
    {
        active_controller->traj_count = new_traj_count;
    }

    void switch_traj(uint traj_select) override
    {
        trajectory_generator.switch_traj(traj_select);
        traj_data_real_len = trajectory_generator.get_traj_data_real_len();
    }

    Eigen::VectorXd update_control(const Eigen::VectorXd &x_nq) override;

    void reset() override;
    void reset(const casadi_real *const x_k_in) override;

    // Method to get the error flag
    ErrorFlag get_error_flag()
    {
        return error_flag;
    }

    uint get_traj_count() override
    {
        return active_controller->traj_count;
    }

    uint get_traj_step() override
    {
        return 1; // always 1 because it is extremely fast
    }

    uint get_N_step() override
    {
        return 1;
    }

    const double *get_act_traj_data() override
    {
        uint traj_count = active_controller->traj_count;
        return trajectory_generator.get_traj_data()->col((traj_count > 0 ? traj_count - 1 : traj_count)).data();
    }

    ControllerType get_controller_type()
    {
        return selected_controller_type;
    }

private:

    // Nested classes inheriting from BaseWorkspaceController
    class CTController : public BaseWorkspaceController
    {
    public:
        CTController(RobotModel &robot_model,
                     const std::string &general_config_filename,
                     TrajectoryGenerator &trajectory_generator,
                     double& tau_max_jump)
            : BaseWorkspaceController(robot_model, general_config_filename, trajectory_generator, tau_max_jump) {}
        Eigen::VectorXd control(const Eigen::VectorXd &x) override;
        ~CTController() override = default;
    };

    class PDPlusController : public BaseWorkspaceController
    {
    public:
        PDPlusController(RobotModel &robot_model,
                         const std::string &general_config_filename,
                         TrajectoryGenerator &trajectory_generator,
                         double& tau_max_jump)
            : BaseWorkspaceController(robot_model, general_config_filename, trajectory_generator, tau_max_jump) {}
        Eigen::VectorXd control(const Eigen::VectorXd &x) override;
        ~PDPlusController() override = default;
    };

    class InverseDynamicsController : public BaseWorkspaceController
    {
    public:
        InverseDynamicsController(RobotModel &robot_model,
                                  const std::string &general_config_filename,
                                  TrajectoryGenerator &trajectory_generator,
                                  double& tau_max_jump)
            : BaseWorkspaceController(robot_model, general_config_filename, trajectory_generator, tau_max_jump) {}
        Eigen::VectorXd control(const Eigen::VectorXd &x) override;
        ~InverseDynamicsController() override = default;
    };

    // Instances of nested classes
    CTController ct_controller;                       // Instance of CTController
    PDPlusController pd_plus_controller;              // Instance of PDPlusController
    InverseDynamicsController inverse_dyn_controller; // Instance of InverseDynamicsController
    BaseWorkspaceController *active_controller=0;     // I do not need a smart pointer because I store all instances in the class.
    ControllerType selected_controller_type;
};

#endif