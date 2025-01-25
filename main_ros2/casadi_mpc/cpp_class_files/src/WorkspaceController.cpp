#include "WorkspaceController.hpp"

void BaseController ::computeJacobianRegularization()
{
    ;
}

WorkspaceController ::WorkspaceController(
    const std::string &urdf_filename,
    const std::string &tcp_frame_name,
    bool use_gravity,
    ControllerSettings &controller_settings,
    RegularizationSettings &regularization_settings) : urdf_filename(urdf_filename),
                                                       tcp_frame_name(tcp_frame_name),
                                                       robot_config(get_robot_config()),
                                                       controller_settings(controller_settings),
                                                       regularization_settings(regularization_settings),
                                                       robot_model(urdf_filename, tcp_frame_name, robot_config, use_gravity)
{
    
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