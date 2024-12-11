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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp" // Include the correct message type

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("joint_state_publisher")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/franka/joint_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState & msg) const
  {
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
    // Copy positions (first 7 elements)
    // std::copy(msg.position.data(), msg.position.data() + 7, joint_states);

    // // Copy velocities (next 7 elements)
    // std::copy(msg.velocity.data(), msg.velocity.data() + 7, joint_states + 7);

    // Write combined positions and velocities to shared memory
    write_to_shared_memory("data_from_simulink", joint_states, sizeof(joint_states));


    // Write validity flag as int8 (value of 1)
    int8_t valid_flag = 1; // Validity flag set to 1
    write_to_shared_memory("data_from_simulink_valid", &valid_flag, sizeof(int8_t));
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_; // Subscription pointer
};

int main(int argc, char * argv[])
{
  int8_t start_flag = 1;  // Set the value to 1
  write_to_shared_memory("data_from_simulink_start", &start_flag, sizeof(int8_t));
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
