// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#define MY_LOG_LEVEL  RCUTILS_LOG_SEVERITY_WARN
// #define MY_LOG_LEVEL RCUTILS_LOG_SEVERITY_INFO

#define N_DOF 7
#define MAX_INVALID_COUNT 100

#include <string>

#include <controller_interface/controller_interface.hpp>
#include "franka_example_controllers/visibility_control.h"

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include "mpc_interfaces/msg/num.hpp"
#include "mpc_interfaces/srv/add_three_ints.hpp"
#include "mpc_interfaces/srv/simple_command.hpp"
#include "mpc_interfaces/srv/trajectory_command.hpp"
#include <semaphore.h>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The gravity compensation controller only sends zero torques so that the robot does gravity
 * compensation
 */
class ModelPredictiveControllerPinocchio : public controller_interface::ControllerInterface{
 public:
  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  CallbackReturn on_init() override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  std::string arm_id_;
  const int num_joints = N_DOF;
  double torques_prev[N_DOF] = {0}; // previous torques
  int invalid_counter = 0; // counter for invalid data, if exceeds MAX_INVALID_COUNT, terminate the controller
  int shm_states = 0;
  int shm_states_valid = 0;
  sem_t* shm_changed_semaphore = 0;
  int shm_start_mpc = 0;
  int shm_reset_mpc = 0;
  int shm_stop_mpc = 0;
  int shm_select_trajectory = 0;
  int shm_torques = 0;
  int shm_torques_valid = 0;
  bool mpc_started = false;
  bool first_torque_read = false;
  // rclcpp::Subscription<mpc_interfaces::msg::Num>::SharedPtr subscription_;
  rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr start_mpc_service_;
  rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr reset_mpc_service_;
  rclcpp::Service<mpc_interfaces::srv::SimpleCommand>::SharedPtr stop_mpc_service_;
  rclcpp::Service<mpc_interfaces::srv::TrajectoryCommand>::SharedPtr traj_switch_service_;
  
  void open_shared_memories();
  void close_shared_memories();
  // void topic_callback(const mpc_interfaces::msg::Num & msg);
  void start_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
          std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response>       response);
  void reset_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
          std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response>       response);
  void stop_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
          std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response>       response);
  void traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
          std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Response>       response);
};
}  // namespace franka_example_controllers
