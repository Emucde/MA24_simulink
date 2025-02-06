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

#include "RobotModel.hpp"
#include "TrajectoryGenerator.hpp"

class BaseController
{
public:
    BaseController(RobotModel &robot_model,
                   SingularityRobustnessMode &sing_method,
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
          dt(robot_model.robot_config.dt)
    {
    }
    virtual Eigen::VectorXd control(double *x) = 0;          // Pure virtual function
    virtual Eigen::MatrixXd computeJacobianRegularization(); // Common method in BaseController
    virtual void calculateControlData(double *x);            // Common method in BaseController
    virtual void set_singularity_robustness_mode(SingularityRobustnessMode sing_method) { this->sing_method = sing_method; }
    virtual ~BaseController() = default; // Virtual destructor
protected:
    RobotModel &robot_model;
    SingularityRobustnessMode &sing_method;
    ControllerSettings &controller_settings;
    RegularizationSettings &regularization_settings;
    TrajectoryGenerator &trajectory_generator;
    int nq;
    int nx;
    Eigen::MatrixXd J, J_pinv, J_p;
    Eigen::MatrixXd M, C, C_rnea;
    Eigen::VectorXd g;
    Eigen::VectorXd q, q_p;
    Eigen::VectorXd x_err, x_err_p;
    Eigen::VectorXd x_d_p, x_d_pp;
    double dt;
    bool is_initialized = false;
};

class WorkspaceController
{
public:
    // Constructor
    WorkspaceController(const std::string &urdf_path,
                        const std::string &tcp_frame_name,
                        bool use_gravity,
                        ControllerSettings &controller_settings);

    void switchController(ControllerType type);
    void update(const double *const x);

private:
    BaseController *active_controller; // I do not need a smart pointer because I store all instances in the class.

    const std::string urdf_path;
    const std::string tcp_frame_name;
    robot_config_t robot_config;
    ControllerSettings controller_settings;
    RobotModel robot_model;
    FullSystemTorqueMapper torque_mapper;
    TrajectoryGenerator trajectory_generator;
    SingularityRobustnessMode sing_method;

    // Nested classes inheriting from BaseController
    class CTController : public BaseController
    {
    public:
        CTController(RobotModel &robot_model,
                     SingularityRobustnessMode &sing_method,
                     ControllerSettings &controller_settings,
                     TrajectoryGenerator &trajectory_generator)
            : BaseController(robot_model, sing_method, controller_settings, trajectory_generator) {}
        Eigen::VectorXd control(double *x) override;
        ~CTController() override = default;
    };

    class PDPlusController : public BaseController
    {
    public:
        PDPlusController(RobotModel &robot_model,
                         SingularityRobustnessMode &sing_method,
                         ControllerSettings &controller_settings,
                         TrajectoryGenerator &trajectory_generator)
            : BaseController(robot_model, sing_method, controller_settings, trajectory_generator) {}
        Eigen::VectorXd control(double *x) override;
        ~PDPlusController() override = default;
    };

    class InverseDynamicsController : public BaseController
    {
    public:
        InverseDynamicsController(RobotModel &robot_model,
                                  SingularityRobustnessMode &sing_method,
                                  ControllerSettings &controller_settings,
                                  TrajectoryGenerator &trajectory_generator)
            : BaseController(robot_model, sing_method, controller_settings, trajectory_generator) {}
        Eigen::VectorXd control(double *x) override;
        Eigen::VectorXd q_d_prev, q_p_d_prev;
        ~InverseDynamicsController() override = default;
    };

    // Instances of nested classes
    CTController ct_controller;                       // Instance of CTController
    PDPlusController pd_plus_controller;              // Instance of PDPlusController
    InverseDynamicsController inverse_dyn_controller; // Instance of InverseDynamicsController

    // Initialize trajectory data
    void init_file_trajectory(casadi_uint traj_select, const casadi_real *x_k_ndof_ptr,
                              double T_start, double T_poly, double T_end);

    // Method for creating a custom trajectory with extra samples for the last prediction horizon
    void init_custom_trajectory(ParamPolyTrajectory param_target);
};