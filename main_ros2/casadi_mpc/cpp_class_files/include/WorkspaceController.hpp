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
                        const Config &config);
    
    void switchController(ControllerType type);
    void update(const double const* x);
private:
    BaseController* active_controller;
    
    // Robot model data
    pinocchio::Model robot_model;
    pinocchio::Data robot_data;
    pinocchio::Model robot_model_reduced;
    pinocchio::Data robot_data_reduced;
    
    // Calculated Robot Data
    JointData joint_data;
    KinematicsData kinematics_data;
    DynamicsData dynamics_data;
    
    // Robot configuration struct
    robot_config_t robot_config;

	// Create robot models
    void createRobotModels(const std::string &urdf_filename);
	
    void calculateRobotData(const double const* x);
    
    Config createControllerConfig();
    
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
 }


//  void update(double* x) {
// 	calculateRobotData(double* x);
//     active_controller->control();
        
// void switchController(ControllerType type) {
// 	switch (type) {
// 		case ControllerType::CT:
// 			active_controller = &ct_controller;
// 			break;
// 		case ControllerType::PDPlus:
// 			active_controller = &pd_plus_controller;
// 			break;
// 		case ControllerType::InverseDynamics:
// 			active_controller = &inverse_dyn_controller;
// 			break;
// 	}
// }