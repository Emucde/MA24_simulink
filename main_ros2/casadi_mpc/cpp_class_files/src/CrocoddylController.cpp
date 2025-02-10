#include "CrocoddylController.hpp"

CrocoddylController::CrocoddylController(const std::string &urdf_path,
                                         const std::string &crocoddyl_config_path,
                                         const std::string &tcp_frame_name,
                                         bool use_gravity)
    : urdf_path(urdf_path), crocoddyl_config_path(crocoddyl_config_path),
      tcp_frame_name(tcp_frame_name),
      robot_config(get_robot_config()),
      n_indices(ConstIntVectorMap(robot_config.n_indices, robot_config.nq_red)),
      n_x_indices(ConstIntVectorMap(robot_config.n_x_indices, robot_config.nx_red)),
      nq(robot_config.nq), nx(robot_config.nx), nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
      robot_model(urdf_path, tcp_frame_name, robot_config, use_gravity, true), // use reduced model
      torque_mapper(urdf_path, tcp_frame_name, robot_config, use_gravity, false),
      trajectory_generator(torque_mapper, robot_config.dt),
      dyn_mpc_v1(robot_model, crocoddyl_config_path, tcp_frame_name, trajectory_generator),
      active_mpc(&dyn_mpc_v1)
{
  // Initialize the previous torque
  tau_full_prev = Eigen::VectorXd::Zero(nq);
}

void CrocoddylController::simulateModelEuler(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt)
{
  Eigen::Map<Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);
  Eigen::Map<const Eigen::VectorXd> tau(tau_ptr, nq);
  torque_mapper.simulateModelEuler(x_k_ndof, tau, dt);

  // Check for errors
  if (x_k_ndof.hasNaN())
  {
    error_flag = ErrorFlag::NAN_DETECTED;
    std::cerr << "NaN values detected in the joint vector!" << std::endl;
  }
}

void CrocoddylController::simulateModelRK4(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt)
{
  Eigen::Map<Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);
  Eigen::Map<const Eigen::VectorXd> tau(tau_ptr, nq);
  torque_mapper.simulateModelRK4(x_k_ndof, tau, dt);

  // Check for errors
  if (x_k_ndof.hasNaN())
  {
    error_flag = ErrorFlag::NAN_DETECTED;
    std::cerr << "NaN values detected in the joint vector!" << std::endl;
  }
}