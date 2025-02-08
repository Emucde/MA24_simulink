#include "param_robot.h"
#include "robot_data.hpp"
#include "workspace_controller_config.hpp"
#include <iostream>

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

class BaseController
{
public:
    BaseController(RobotModel &robot_model,
                   RegularizationMode &sing_method,
                   ControllerSettings &controller_settings,
                   TrajectoryGenerator &trajectory_generator)
        : robot_model(robot_model),
          sing_method(sing_method),
          controller_settings(controller_settings),
          regularization_settings(controller_settings.regularization_settings),
          trajectory_generator(trajectory_generator),
          nq(robot_model.nq), nx(robot_model.nx),
          J(Eigen::MatrixXd::Zero(6, nq)), J_pinv(Eigen::MatrixXd::Zero(nq, 6)), J_p(Eigen::MatrixXd::Zero(6, nq)),
          M(Eigen::MatrixXd::Zero(nq, nq)), C(Eigen::MatrixXd::Zero(nq, nq)), C_rnea(Eigen::VectorXd::Zero(nq)),
          g(Eigen::VectorXd::Zero(nq)),
          q(Eigen::VectorXd::Zero(nq)), q_p(Eigen::VectorXd::Zero(nq)),
          x_err(Eigen::VectorXd::Zero(6)), x_err_p(Eigen::VectorXd::Zero(6)),
          x_d_p(Eigen::VectorXd::Zero(6)), x_d_pp(Eigen::VectorXd::Zero(6)),
          dt(robot_model.robot_config.dt), traj_count(0)
    {
    }
    virtual Eigen::VectorXd control(const Eigen::VectorXd &x) = 0; // Pure virtual function
    virtual Eigen::MatrixXd computeJacobianRegularization();       // Common method in BaseController
    virtual void calculateControlData(const Eigen::VectorXd &x);   // Common method in BaseController
    virtual void set_singularity_robustness_mode(RegularizationMode sing_method) { this->sing_method = sing_method; }
    virtual void set_regularization_settings(RegularizationSettings regularization_settings)
    {
        this->regularization_settings = regularization_settings;
        this->sing_method = regularization_settings.mode;
    }
    virtual void set_controller_settings(ControllerSettings controller_settings) {
        this->controller_settings = controller_settings;
        this->regularization_settings = controller_settings.regularization_settings;
        this->sing_method = controller_settings.regularization_settings.mode;
    }
    virtual ~BaseController() = default; // Virtual destructor
protected:
    RobotModel &robot_model;
    RegularizationMode &sing_method;
    ControllerSettings controller_settings;
    RegularizationSettings regularization_settings;
    TrajectoryGenerator &trajectory_generator;
    const int nq, nx; // this is nq_red and nx_red!!!
    Eigen::MatrixXd J, J_pinv, J_p;
    Eigen::MatrixXd M, C, C_rnea;
    Eigen::VectorXd g;
    Eigen::VectorXd q, q_p;
    Eigen::VectorXd x_err, x_err_p;
    Eigen::VectorXd x_d_p, x_d_pp;
    double dt;

public:
    uint traj_count;
};

class WorkspaceController
{
public:
    // Constructor
    WorkspaceController(const std::string &urdf_path,
                        const std::string &tcp_frame_name,
                        bool use_gravity);

    void switchController(ControllerType type);
    void simulateModelEuler(casadi_real *const x_k_ndof_ptr, const casadi_real *const tau_ptr, double dt);
    void simulateModelRK4(casadi_real *const x_k_ndof_ptr, const casadi_real *const tau_ptr, double dt);
    Eigen::VectorXd update(const double *const x_nq);

    ControllerSettings init_default_controller_settings();
    RegularizationSettings init_default_regularization_settings();

    // Initialize trajectory data
    void init_file_trajectory(casadi_uint traj_select, const casadi_real *x_k_ndof_ptr,
                              double T_start, double T_poly, double T_end);

    // Method for creating a custom trajectory with extra samples for the last prediction horizon
    void init_custom_trajectory(ParamPolyTrajectory param_target);

    // Method to set the Robustness mode
    void set_singularity_robustness_mode(RegularizationMode sing_method)
    {
        active_controller->set_singularity_robustness_mode(sing_method);
    }

    // Method to set the maximum torque jump
    void set_tau_max_jump(double tau_jump)
    {
        tau_max_jump = tau_jump;
    }

    // Method to get the error flag
    ErrorFlag get_error_flag()
    {
        return error_flag;
    }

    const double *get_act_traj_data()
    {
        uint traj_count = active_controller->traj_count;
        return trajectory_generator.get_traj_data()->col((traj_count > 0 ? traj_count - 1 : traj_count)).data();
    }

    int get_traj_data_real_len()
    {
        return trajectory_generator.get_traj_data_real_len();
    }

    const double *get_traj_x0_init(casadi_uint traj_select)
    {
        return trajectory_generator.get_traj_file_x0_init(traj_select)->data();
    }

    void set_regularization_settings(RegularizationSettings regularization_settings)
    {
        active_controller->set_regularization_settings(regularization_settings);
    }

    void set_controller_settings(ControllerSettings controller_settings)
    {
        active_controller->set_controller_settings(controller_settings);
    }

private:
    const std::string urdf_path;
    const std::string tcp_frame_name;
    robot_config_t robot_config;
    const Eigen::VectorXi n_indices, n_x_indices;
    const int nq, nx, nq_red, nx_red;
    ControllerSettings controller_settings;
    RobotModel robot_model;
    FullSystemTorqueMapper torque_mapper;
    TrajectoryGenerator trajectory_generator;
    RegularizationMode sing_method;

    // Nested classes inheriting from BaseController
    class CTController : public BaseController
    {
    public:
        CTController(RobotModel &robot_model,
                     RegularizationMode &sing_method,
                     ControllerSettings &controller_settings,
                     TrajectoryGenerator &trajectory_generator)
            : BaseController(robot_model, sing_method, controller_settings, trajectory_generator) {}
        Eigen::VectorXd control(const Eigen::VectorXd &x) override;
        ~CTController() override = default;
    };

    class PDPlusController : public BaseController
    {
    public:
        PDPlusController(RobotModel &robot_model,
                         RegularizationMode &sing_method,
                         ControllerSettings &controller_settings,
                         TrajectoryGenerator &trajectory_generator)
            : BaseController(robot_model, sing_method, controller_settings, trajectory_generator) {}
        Eigen::VectorXd control(const Eigen::VectorXd &x) override;
        ~PDPlusController() override = default;
    };

    class InverseDynamicsController : public BaseController
    {
    public:
        InverseDynamicsController(RobotModel &robot_model,
                                  RegularizationMode &sing_method,
                                  ControllerSettings &controller_settings,
                                  TrajectoryGenerator &trajectory_generator)
            : BaseController(robot_model, sing_method, controller_settings, trajectory_generator) {}
        Eigen::VectorXd control(const Eigen::VectorXd &x) override;
        Eigen::VectorXd q_d_prev, q_p_d_prev;
        bool init = true;
        ~InverseDynamicsController() override = default;
    };

    // Instances of nested classes
    CTController ct_controller;                       // Instance of CTController
    PDPlusController pd_plus_controller;              // Instance of PDPlusController
    InverseDynamicsController inverse_dyn_controller; // Instance of InverseDynamicsController
    BaseController *active_controller;                // I do not need a smart pointer because I store all instances in the class.

    ErrorFlag error_flag = ErrorFlag::NO_ERROR;
    Eigen::VectorXd tau_full_prev = Eigen::VectorXd::Zero(nq);
    double tau_max_jump = 5.0; // Maximum jump in torque (Nm/dt)

    void error_check(Eigen::VectorXd &tau_full);
};