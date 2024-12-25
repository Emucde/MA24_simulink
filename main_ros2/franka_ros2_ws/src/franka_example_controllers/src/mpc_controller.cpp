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

#include <franka_example_controllers/mpc_controller.hpp>

#include <exception>
#include <string>
#include "shared_memory.hpp"
#include <unistd.h>

#include <memory>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
ModelPredictiveController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= N_DOF; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
ModelPredictiveController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= N_DOF; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= N_DOF; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return state_interfaces_config;
}

controller_interface::return_type ModelPredictiveController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  double* torques_crocoddyl = 0;
  double read_torques[N_DOF] = {0.0};
  int8_t valid_flag = 0;

  double state[2*N_DOF];

  for (int i = 0; i < 2*N_DOF; ++i) {
    state[i] = state_interfaces_[i].get_value();
  }

  // Write combined state (q, q_p) to shared memory (send data to crocoddyl)
  write_to_shared_memory(shm_states, state, sizeof(state), get_node()->get_logger());

  // Write validity flag for state data to shared memory
  valid_flag = 1;
  write_to_shared_memory(shm_states_valid, &valid_flag, sizeof(int8_t), get_node()->get_logger());

  // Logging joint states
  RCLCPP_INFO(get_node()->get_logger(), "q (rad): [%f, %f, %f, %f, %f, %f, %f]",
            state[0], state[1], state[2],
            state[3], state[4], state[5], state[6]);

  RCLCPP_INFO(get_node()->get_logger(), "q_p (rad/s): [%f, %f, %f, %f, %f, %f, %f]",
            state[N_DOF+0], state[N_DOF+1], state[N_DOF+2],
            state[N_DOF+3], state[N_DOF+4], state[N_DOF+5], state[N_DOF+6]);

  // Read validity flag for torque data from shared memory
  read_shared_memory_flag(shm_torques_valid, &valid_flag, sizeof(int8_t), get_node()->get_logger());
  
  if(valid_flag == 1)
  {
    // Reading data from shared memory
    read_shared_memory(shm_torques, &read_torques[0], N_DOF * sizeof(double), get_node()->get_logger());
    std::memcpy(torques_prev, read_torques, N_DOF * sizeof(double));
    torques_crocoddyl = &read_torques[0];
    
    invalid_counter = 0;
    valid_flag = 0;
    write_to_shared_memory(shm_states_valid, &valid_flag, sizeof(int8_t), get_node()->get_logger());
  }
  else if(valid_flag == 0)
  {
    if(invalid_counter < MAX_INVALID_COUNT)
    {
      torques_crocoddyl = &torques_prev[0];
      invalid_counter++;
      RCLCPP_WARN(get_node()->get_logger(), "No valid data received from Python. Using previous torques.");
    }
    else
    {
      for (int i = 0; i < N_DOF; ++i) {
        command_interfaces_[i].set_value(0);
      }
      RCLCPP_ERROR(get_node()->get_logger(), "No valid data received from Python. Terminating the controller.");
      return controller_interface::return_type::ERROR;
    }
  }

  // Write torques to shared memory (send data to robot)
  for (int i = 0; i < N_DOF; ++i) {
    command_interfaces_[i].set_value(torques_crocoddyl[i]);
  }

  // Logging torques
  RCLCPP_INFO(get_node()->get_logger(), "tau (Nm): [%f, %f, %f, %f, %f, %f, %f]",
            torques_crocoddyl[0], torques_crocoddyl[1], torques_crocoddyl[2],
            torques_crocoddyl[3], torques_crocoddyl[4], torques_crocoddyl[5], torques_crocoddyl[6]);

  return controller_interface::return_type::OK;
}

CallbackReturn ModelPredictiveController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  RCLCPP_INFO(get_node()->get_logger(), "Configuring MPC controller for %s arm", arm_id_.c_str());
  
  subscription_ = get_node()->create_subscription<mpc_interfaces::msg::Num>(
      "topic", 10, std::bind(&ModelPredictiveController::topic_callback, this, _1));

  RCLCPP_INFO(get_node()->get_logger(), "Subscribed to topic 'topic'");

  // Create the service
  // service_ = get_node()->create_service<custom_interfaces::Trigger>(
  //   "my_service",
  //   std::bind(&ModelPredictiveController::handle_service, this, std::placeholders::_1, std::placeholders::_2));

  service_ = get_node()->create_service<mpc_interfaces::srv::AddThreeInts>(
      "add_three_ints",
      std::bind(&ModelPredictiveController::add, this, std::placeholders::_1, std::placeholders::_2)
  );

  RCLCPP_INFO(get_node()->get_logger(), "Service 'add_three_ints' created");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ModelPredictiveController::on_init() {
  rcutils_ret_t ret = rcutils_logging_set_logger_level(
    get_node()->get_logger().get_name(), MY_LOG_LEVEL);
  if (ret != RCUTILS_RET_OK)
  {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set logger level: %d", ret);
  }
  try {
    auto_declare<std::string>("arm_id", "panda");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  try{
    open_shared_memories();
  }
  catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  // Write start flag to shared memory
  //int8_t traj_select = 1;
  //write_to_shared_memory(shm_selected_trajectory, &traj_select, sizeof(int8_t), get_node()->get_logger());
  int8_t start_flag = 1;
  write_to_shared_memory(shm_start_trajectory, &start_flag, sizeof(int8_t), get_node()->get_logger());

  return CallbackReturn::SUCCESS;
}

CallbackReturn ModelPredictiveController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    close_shared_memories();
    return controller_interface::CallbackReturn::SUCCESS;
}

CallbackReturn ModelPredictiveController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    close_shared_memories();
    return controller_interface::CallbackReturn::SUCCESS;
}

CallbackReturn ModelPredictiveController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    close_shared_memories();
    return controller_interface::CallbackReturn::SUCCESS;
}

void ModelPredictiveController::open_shared_memories()
{
    // Open shared memory for writing states to crocddyl
    shm_states = open_write_shm("data_from_simulink", 2*N_DOF*sizeof(double), get_node()->get_logger());
    shm_states_valid = open_write_shm("data_from_simulink_valid", sizeof(int8_t), get_node()->get_logger());
    shm_selected_trajectory = open_write_shm("data_from_simulink_traj_switch", sizeof(int8_t), get_node()->get_logger());
    shm_start_trajectory = open_write_shm("data_from_simulink_start", sizeof(int8_t), get_node()->get_logger());
    
    // Open shared memory for reading torques from crocddyl
    shm_torques = open_read_shm("data_from_python", get_node()->get_logger());
    shm_torques_valid = open_read_shm("data_from_python_valid", get_node()->get_logger());
    RCLCPP_INFO(get_node()->get_logger(), "Shared memory opened successfully.");
}

void ModelPredictiveController::close_shared_memories()
{
    if (shm_states != -1) close(shm_states);
    if (shm_states_valid != -1) close(shm_states_valid);
    if (shm_selected_trajectory != -1) close(shm_selected_trajectory);
    if (shm_start_trajectory != -1) close(shm_start_trajectory);
    if (shm_torques != -1) close(shm_torques);
    if (shm_torques_valid != -1) close(shm_torques_valid);
}

void ModelPredictiveController::topic_callback(const mpc_interfaces::msg::Num & msg)
{
  RCLCPP_WARN(get_node()->get_logger(), "I heard: '%ld'", msg.num);
}

// void ModelPredictiveController::handle_service(
//   const std::shared_ptr<custom_interfaces::Trigger::Request> request,
//   std::shared_ptr<custom_interfaces::Trigger::Response> response)
// {
//   // Implement your service logic here
//   response->success = true;
//   response->message = "Service called successfully";
//   RCLCPP_INFO(get_node()->get_logger(), "Service called");
// }

void ModelPredictiveController::add(const std::shared_ptr<mpc_interfaces::srv::AddThreeInts::Request> request,
          std::shared_ptr<mpc_interfaces::srv::AddThreeInts::Response>       response)
{
  response->sum = request->a + request->b + request->c;                               
  // RCLCPP_INFO(get_node()->get_logger(), "Incoming request\na: %ld" " b: %ld" " c: %ld",
  //               request->a, request->b, request->c);                                     
  // RCLCPP_INFO(get_node()->get_logger(), "sending back response: [%ld]", (long int)response->sum);
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ModelPredictiveController,
                       controller_interface::ControllerInterface)
