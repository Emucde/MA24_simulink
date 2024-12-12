// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>
#include "shared_memory.hpp"
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <controller_interface/controller_interface.hpp>
// #include <controller_interface/helpers.hpp>
// #include <hardware_interface/hardware_interface.hpp> // does not work
#include "hardware_interface/loaned_command_interface.hpp"
// #include "franka_example_controllers/visibility_control.h"
// controller_interface/helpers.hpp

#define MY_LOG_LEVEL  RCUTILS_LOG_SEVERITY_WARN
// #define MY_LOG_LEVEL  RCUTILS_LOG_SEVERITY_INFO

using std::placeholders::_1;

//////////////////////////////////////////////////////////
///////////////////// SUBSCRIBER NODE ////////////////////
//////////////////////////////////////////////////////////

class RobotStateSubscriber : public rclcpp::Node
{
public:
  RobotStateSubscriber()
  : Node("joint_state_publisher")
  {
    rcutils_ret_t ret = rcutils_logging_set_logger_level("joint_state_publisher", MY_LOG_LEVEL);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set logger level: %d", ret);
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/franka/joint_states", 10, std::bind(&RobotStateSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState & msg) const
  {
    // warum auch immer ist die Reihenfolge der States ungeordnet.
    // TODO: Write own state subscriber!
    RCLCPP_INFO(this->get_logger(), "Joint positions: [%f, %f, %f, %f, %f, %f, %f]",
                msg.position[0], msg.position[3], msg.position[4],
                msg.position[5], msg.position[1], msg.position[2], msg.position[6]);

    RCLCPP_INFO(this->get_logger(), "Joint velocities: [%f, %f, %f, %f, %f, %f, %f]",
                msg.velocity[0], msg.velocity[3], msg.velocity[4],
                msg.velocity[5], msg.velocity[1], msg.velocity[2], msg.velocity[6]);

    // Combined array for positions and velocities
    double joint_states[14];
    int indices[] = {0, 3, 4, 5, 1, 2, 6};

    // Copy positions (first 7 elements)
    for (size_t i = 0; i < 7; ++i)
    {
        joint_states[i] = msg.position[indices[i]];
    }

    // Copy velocities (next 7 elements)
    for (size_t i = 0; i < 7; ++i)
    {
        joint_states[i + 7] = msg.velocity[indices[i]];
    }

    // Write combined positions and velocities to shared memory
    write_to_shared_memory("data_from_simulink", joint_states, sizeof(joint_states));


    // Write validity flag as int8 (value of 1)
    int8_t valid_flag = 1; // Validity flag set to 1
    write_to_shared_memory("data_from_simulink_valid", &valid_flag, sizeof(int8_t));
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_; // Subscription pointer
};

//////////////////////////////////////////////////////////
///////////////////// CONTROL NODE ///////////////////////
//////////////////////////////////////////////////////////

class FrankaController : public rclcpp::Node, public controller_interface::ControllerInterface
{
public:
    FrankaController()
    : Node("franka_controller"), num_joints(7)
    {
        rcutils_ret_t ret = rcutils_logging_set_logger_level("franka_controller", MY_LOG_LEVEL);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set logger level: %d", ret);
        }

        this->declare_parameter("arm_id", "fr3");
        arm_id_ = this->get_parameter("arm_id").as_string();

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/franka/joint_states", 10, 
            std::bind(&FrankaController::joint_state_callback, this, std::placeholders::_1));
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const 
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (int i = 1; i <= num_joints; ++i) {
            config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
        }
        return config;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const 
    {
        return {};
    }

    controller_interface::return_type update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) 
    {
        double* data = read_shared_memory("data_from_python", 7 * sizeof(double));
        if (data)
        {
            RCLCPP_INFO(this->get_logger(), "Read Torques: [%f, %f, %f, %f, %f, %f, %f]", 
                        data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
        }
        for (auto& command_interface : command_interfaces_) {
            command_interface.set_value(0);
        }
        return controller_interface::return_type::OK;
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) 
    {
        arm_id_ = this->get_parameter("arm_id").as_string();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_init() {
    try {
      auto_declare<std::string>("arm_id", "fr3");
    } catch (const std::exception& e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Process joint states
        // You can add your logic here to handle the incoming joint states

        // Call update method
        update(this->now(), rclcpp::Duration(0, 0));
    }

    std::string arm_id_;
    const int num_joints;
    std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};


//////////////////////////////////////////////////////////
///////////////////// MAIN FUNCTION //////////////////////
//////////////////////////////////////////////////////////

/*
Die Idee ist, dass zuvor das Programm
ros2 launch franka_bringup franka.launch.py arm_id:=fr3 robot_ip:=$robot_ip
gestartet wird. Dieses liest die joint states vom Roboter aus und pubished sie
mit einer Rate von 1kHz.

Danach wird sowohl die Klasse RobotStateSubscriber als auch die Klasse FrankaController
auf das Topic /franka/joint_states gesubscribed. Das hat den Vorteil, dass die update
Funktion in beiden Klassen zugleich aufgerufen wird, wodurch garantiert ist, dass beide
Klassen perfekt synchron laufen.

Die Klasse RobotStateSubscriber schreibt die joint states in den shared memory, sodass
sie von Crocoddyl gelesen werden können. Sie ersetzt die Simulink Funktion, wodurch
das crocoddyl Programm überhaupt nicht umgeschrieben werden muss und genau gleich wie
in Simulink verwendet werden kann.

Die Klasse FrankaController ist fast 1:1 vom example "gravity_compensation_example_controller"
übernommen. Zum Zeitpunkt, wenn neue Daten verfügbar sind und an crocoddyl geschrieben
werden liest sie das vorherige Ergebnis der torques von Crocoddyl und schickt diese zum
Roboter.

*/

// Signal handler
void signal_handler(int signal) {
    if (signal == SIGINT) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Caught SIGINT, shutting down...");
        rclcpp::shutdown();
    }
}

int main(int argc, char *argv[]) {
    // Write start flag to shared memory
    int8_t start_flag = 1;  // Set the value to 1
    write_to_shared_memory("data_from_simulink_start", &start_flag, sizeof(int8_t));

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Register signal handler for graceful shutdown
    std::signal(SIGINT, signal_handler);

    // Create nodes
    auto robot_state_subscriber = std::make_shared<RobotStateSubscriber>();
    auto franka_controller = std::make_shared<FrankaController>();

    // Use a MultiThreadedExecutor to spin both nodes concurrently
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(robot_state_subscriber);
    executor.add_node(franka_controller);

    // Spin the executor
    try {
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
    }

    // Shutdown ROS 2
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down ROS 2...");
    rclcpp::shutdown();

    return 0;
}