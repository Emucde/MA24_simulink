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
                   ControllerSettings &controller_settings)
        : robot_model(robot_model), 
          sing_method(sing_method), 
          controller_settings(controller_settings), 
          regularization_settings(controller_settings.regularization_settings),
          nq(robot_model.nq), nx(robot_model.nx) {}
    virtual void control() = 0;                   // Pure virtual function
    virtual void computeJacobianRegularization(); // Common method in BaseController
    virtual ~BaseController() = default;          // Virtual destructor
protected:
    RobotModel &robot_model;
    SingularityRobustnessMode &sing_method;
    ControllerSettings &controller_settings;
    RegularizationSettings &regularization_settings;
    int nq, nx;
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
                     ControllerSettings &controller_settings)
            : BaseController(robot_model, sing_method, controller_settings) {}
        void control() override;
        ~CTController() override = default;
    };

    class PDPlusController : public BaseController
    {
    public:
        PDPlusController(RobotModel &robot_model, 
                         SingularityRobustnessMode &sing_method, 
                         ControllerSettings &controller_settings)
            : BaseController(robot_model, sing_method, controller_settings) {}
        void control() override;
        ~PDPlusController() override = default;
    };

    class InverseDynamicsController : public BaseController
    {
    public:
        InverseDynamicsController(RobotModel &robot_model, 
                                  SingularityRobustnessMode &sing_method, 
                                  ControllerSettings &controller_settings)
            : BaseController(robot_model, sing_method, controller_settings) {}
        void control() override;
        ~InverseDynamicsController() override = default;
    };

    // Instances of nested classes
    CTController ct_controller;                       // Instance of CTController
    PDPlusController pd_plus_controller;              // Instance of PDPlusController
    InverseDynamicsController inverse_dyn_controller; // Instance of InverseDynamicsController

    // Initialize trajectory data
    void init_trajectory(casadi_uint traj_select, const casadi_real *x_k_ndof_ptr,
                                       double T_start, double T_poly, double T_end);

    // Initialize trajectory data use T_start, T_poly and T_end from current stored values
    void init_trajectory(casadi_uint traj_select);

    // Method for creating a custom trajectory with extra samples for the last prediction horizon
    void init_trajectory_custom_target(ParamTargetTrajectory param_target, const casadi_real *x_k_ndof_ptr, double T_start, double T_poly, double T_end, double T_horizon_max=2);

};