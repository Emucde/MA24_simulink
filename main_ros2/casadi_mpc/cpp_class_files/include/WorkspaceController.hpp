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

class BaseController {
public:
    virtual void control() = 0; // Pure virtual function
    virtual void computeJacobianRegularization(); // Common method in BaseController
    virtual ~BaseController() = default; // Virtual destructor
};

class WorkspaceController {
public:
    // Constructor
    WorkspaceController(const std::string &urdf_filename,
                        const std::string &tcp_frame_name,
                        bool use_gravity,
                        ControllerSettings &controller_settings,
                        RegularizationSettings &regularization_settings);
    
    void switchController(ControllerType type);
    void update(const double* const x);
private:
    BaseController* active_controller; // I do not need a smart pointer because I store all instances in the class.
    
    const std::string urdf_filename;
    const std::string tcp_frame_name;
    robot_config_t robot_config;
    ControllerSettings controller_settings;
    RegularizationSettings regularization_settings;

    // Robot model
    RobotModel robot_model;

    // Nested classes inheriting from BaseController
    class CTController : public BaseController {
    public:
        void control() override;
        ~CTController() override = default;
    };

    class PDPlusController : public BaseController {
    public:
        void control() override;
        ~PDPlusController() override = default;
    };

    class InverseDynamicsController : public BaseController {
    public:
        void control() override;
        ~InverseDynamicsController() override = default;
    };
	
	// Instances of nested classes
    CTController ct_controller;       // Instance of CTController
    PDPlusController pd_plus_controller; // Instance of PDPlusController
    InverseDynamicsController inverse_dyn_controller; // Instance of InverseDynamicsController
 };