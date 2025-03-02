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

#include <franka_example_controllers/mpc_crocoddyl_controller_shm_controller.hpp>

#include <exception>
#include <string>
#include <unistd.h>

#include <memory>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace franka_example_controllers
{
    controller_interface::return_type ModelPredictiveControllerCrocoddylSHMController::update(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {   
        if (controller_started)
        {   

#ifndef SIMULATION_MODE
            // Read states from joint state interface
            for (int i = 0; i < nx; ++i)
            {
                state[i] = state_interfaces_[i].get_value();
            }
            x_measured = state;
#else
            base_controller->simulateModelRK4(state.data(), tau_full.data(), Ts);
            if(use_noise)
                x_measured = state + generateNoiseVector(nx, Ts, mean_noise_amplitude);
            else
                x_measured = state;
#endif
        
            valid_cpp = read_valid_cpp_shm();
            if (valid_cpp == 1)
            {
                write_valid_cpp_shm(0);
                shm.read_double("cpp_control_data", tau_full.data());
                shm.read_int8("error_cpp", &error_flag_int8);
                error_flag = static_cast<ErrorFlag>(error_flag_int8);

                if (error_flag != ErrorFlag::NO_ERROR)
                {
                    if (error_flag == ErrorFlag::JUMP_DETECTED)
                        RCLCPP_WARN(this->get_node()->get_logger(), "Jump in torque detected. Stopping the controller.");
                    else if (error_flag == ErrorFlag::NAN_DETECTED)
                        RCLCPP_WARN(this->get_node()->get_logger(), "NaN in torque detected. Stopping the controller.");
                    else if (error_flag == ErrorFlag::CROCODDYL_ERROR)
                        RCLCPP_WARN(this->get_node()->get_logger(), "Error in Crocoddyl function call. Stopping the controller.");
                    error_flag = ErrorFlag::NO_ERROR;
                    int8_t stop = 1;
                    shm.feedback_write_int8("stop_cpp", &stop);
                    controller_started = false;
                }

                shm.write("ros2_state_data", x_measured.data());
                shm.post_semaphore("ros2_semaphore");

#ifndef SIMULATION_MODE
                for (int i = 0; i < nq; ++i) {
                    command_interfaces_[i].set_value(tau_full[i]);
                }
            }
#endif
        }
        else
        {
#ifndef SIMULATION_MODE
            for (int i = 0; i < nq; ++i) {
                command_interfaces_[i].set_value(0);
            }
#endif
        }
        return controller_interface::return_type::OK;
    }

    CallbackReturn ModelPredictiveControllerCrocoddylSHMController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        arm_id_ = this->get_node()->get_parameter("arm_id").as_string();

        RCLCPP_INFO(this->get_node()->get_logger(), "Configuring MPC controller for %s arm", arm_id_.c_str());

        // subscription_ = this->get_node()->create_subscription<mpc_interfaces::msg::ControlArray>(
        //     "topic", 10, std::bind(&ModelPredictiveControllerCrocoddylSHMController::topic_callback, this, _1));

        // RCLCPP_INFO(this->get_node()->get_logger(), "Subscribed to topic 'topic'");

        std::string node_name = this->get_node()->get_name();
        std::string service_prefix = "/" + node_name + "/";

        start_mpc_service_ = this->get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "start_mpc_service",
            std::bind(&ModelPredictiveControllerCrocoddylSHMController::start_mpc, this, std::placeholders::_1, std::placeholders::_2));

        reset_mpc_service_ = this->get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "reset_mpc_service",
            std::bind(&ModelPredictiveControllerCrocoddylSHMController::reset_mpc, this, std::placeholders::_1, std::placeholders::_2));

        stop_mpc_service_ = this->get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "stop_mpc_service",
            std::bind(&ModelPredictiveControllerCrocoddylSHMController::stop_mpc, this, std::placeholders::_1, std::placeholders::_2));

        traj_switch_service_ = this->get_node()->create_service<mpc_interfaces::srv::TrajectoryCommand>(
            service_prefix + "traj_switch_service",
            std::bind(&ModelPredictiveControllerCrocoddylSHMController::traj_switch, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_node()->get_logger(), "Services created");

        return CallbackReturn::SUCCESS;
    }

    void ModelPredictiveControllerCrocoddylSHMController::open_shared_memories()
    {
        shm.open_readwrite_shms(shm_readwrite_infos);
        shm.open_readwrite_sems(sem_readwrite_names);
        ros2_semaphore = shm.get_semaphore("ros2_semaphore");
        valid_cpp_shm = shm.get_shared_memory("valid_cpp");
        RCLCPP_INFO(this->get_node()->get_logger(), "Shared memory opened successfully.");
    }

    // void ModelPredictiveControllerCrocoddylSHMController::topic_callback(const mpc_interfaces::msg::ControlArray & msg)
    // {
    //     Eigen::Map<const Eigen::VectorXd> u_k(msg.control_array.data(), msg.control_array.size());
    //     RCUTILS_LOG_WARN("Received control array: [%f, %f, %f, %f, %f, %f]",
    //                     u_k[0], u_k[1], u_k[2], u_k[3], u_k[4], u_k[5]);
    // }

    void ModelPredictiveControllerCrocoddylSHMController::start_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
                                                    std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        controller_started = false;

        shm_flags flags = {
            1, // start
            1, // reset
            0, // stop
            0  // torques_valid
        };

        shm.write("start_cpp", &flags.start);
        shm.write("reset_cpp", &flags.reset);
        shm.write("stop_cpp", &flags.stop);
        
#ifndef SIMULATION_MODE
        for (int i = 0; i < nx; ++i)
            state[i] = state_interfaces_[i].get_value();
#else
        init_trajectory();
        Eigen::VectorXd x0_init = base_controller->get_file_traj_x0_nq_init(traj_select);
        state = x0_init;
#endif

        init_controller();
        shm.write("ros2_state_data", state.data());

        shm.post_semaphore("ros2_semaphore");
        response->status = "start flag set";
        controller_started = true;
    }

    void ModelPredictiveControllerCrocoddylSHMController::reset_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
                                                    std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        shm_flags flags = {
            0, // start
            1, // reset
            0, // stop
            0  // torques_valid
        };

        shm.feedback_write_int8("data_from_simulink_start", &flags.start);
        shm.feedback_write_int8("data_from_simulink_stop", &flags.stop);
        shm.feedback_write_int8("data_from_simulink_reset", &flags.reset);

        shm.feedback_write_int8("start_cpp", &flags.start);
        shm.feedback_write_int8("stop_cpp", &flags.stop);
        shm.feedback_write_int8("reset_cpp", &flags.reset);

        reset();
        first_start = true;

        response->status = "reset flag set";
        controller_started = false;
        solve_started = false;
        shm.post_semaphore("ros2_semaphore");
        shm.post_semaphore("shm_changed_semaphore");
        RCLCPP_INFO(this->get_node()->get_logger(), "Crocoddyl MPC reset");
    }

    void ModelPredictiveControllerCrocoddylSHMController::stop_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
                                                   std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        shm_flags flags = {
            0, // start
            0, // reset
            1, // stop
            0  // torques_valid
        };

        shm.feedback_write_int8("data_from_simulink_start", &flags.start);
        shm.feedback_write_int8("data_from_simulink_reset", &flags.reset);
        shm.feedback_write_int8("data_from_simulink_stop", &flags.stop);

        shm.write("start_cpp", &flags.start);
        shm.write("reset_cpp", &flags.reset);
        shm.write("stop_cpp", &flags.stop);

        shm.post_semaphore("shm_changed_semaphore");
        shm.post_semaphore("ros2_semaphore");

        response->status = "stop flag set";
        controller_started = false;
        solve_started = false;
        RCLCPP_INFO(this->get_node()->get_logger(), "Crocoddyl MPC stopped");
    }

    void ModelPredictiveControllerCrocoddylSHMController::traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
                                                      std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Response> response)
    {
        int old_traj_select = traj_select;
        traj_select = request->traj_select; // it is later used at start_mpc() in init_trajectory
        shm.write("traj_switch_cpp", &traj_select);
        if(old_traj_select == traj_select)
        {
            response->status = "trajectory " + std::to_string(traj_select) + " already selected";
            RCLCPP_INFO(this->get_node()->get_logger(), "Trajectory %d already selected", traj_select);
        }
        else
        {
            controller.switch_traj(traj_select);
            reset();

            response->status = "trajectory " + std::to_string(traj_select) + " selected";
            controller_started = false;
            first_start = true;
            
            RCLCPP_INFO(this->get_node()->get_logger(), "Trajectory %d selected", traj_select);
        }
    }

} // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ModelPredictiveControllerCrocoddylSHMController,
                       controller_interface::ControllerInterface)