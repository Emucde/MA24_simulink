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

#include <franka_example_controllers/mpc_casadi_controller.hpp>

#include <exception>
#include <string>
#include "shared_memory.hpp"
#include <unistd.h>

#include <memory>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace franka_example_controllers
{

    controller_interface::InterfaceConfiguration
    ModelPredictiveControllerCasadi::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (int i = 1; i <= N_DOF; ++i)
        {
            config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
        }
        return config;
    }

    controller_interface::InterfaceConfiguration
    ModelPredictiveControllerCasadi::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (int i = 1; i <= N_DOF; ++i)
        {
            state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
        }
        for (int i = 1; i <= N_DOF; ++i)
        {
            state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
        }
        return state_interfaces_config;
    }

    controller_interface::return_type ModelPredictiveControllerCasadi::update(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {
        double state[2 * N_DOF];

        if (mpc_started)
        {
            // Read states from joint state interface
            for (int i = 0; i < 2 * N_DOF; ++i)
            {
                state[i] = state_interfaces_[i].get_value();
            }

            // Logging joint states
            RCLCPP_INFO(get_node()->get_logger(), "q (rad): [%f, %f, %f, %f, %f, %f, %f]",
                        state[0], state[1], state[2],
                        state[3], state[4], state[5], state[6]);

            RCLCPP_INFO(get_node()->get_logger(), "q_p (rad/s): [%f, %f, %f, %f, %f, %f, %f]",
                        state[N_DOF + 0], state[N_DOF + 1], state[N_DOF + 2],
                        state[N_DOF + 3], state[N_DOF + 4], state[N_DOF + 5], state[N_DOF + 6]);

            // Attempt to get the result (blocking call)
            if (tau_full_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
            {
                invalid_counter = 0;                      // Reset the counter on successful retrieval
                tau_full = tau_full_future.get();         // Get the result
                error_flag = controller.get_error_flag(); // Get the error flag

                if (error_flag == ErrorFlag::NO_ERROR)
                {
                    tau_full_future = std::async(std::launch::async, [this, state]()
                                                 { return controller.solveMPC(state); });
                }
                else
                {
                    mpc_started = false;

                    if (error_flag == ErrorFlag::JUMP_DETECTED)
                    {
                        RCLCPP_WARN(get_node()->get_logger(), "Jump in torque detected (tau (Nm): [%f, %f, %f, %f, %f, %f, %f]). Stopping the controller.",
                                    tau_full[0], tau_full[1], tau_full[2],
                                    tau_full[3], tau_full[4], tau_full[5], tau_full[6]);
                    }
                    else if (error_flag == ErrorFlag::NAN_DETECTED)
                    {
                        RCLCPP_WARN(get_node()->get_logger(), "NaN in torque detected (tau (Nm): [%f, %f, %f, %f, %f, %f, %f]). Stopping the controller.",
                                    tau_full[0], tau_full[1], tau_full[2],
                                    tau_full[3], tau_full[4], tau_full[5], tau_full[6]);
                    }
                    else if (error_flag == ErrorFlag::CASADI_ERROR)
                    {
                        RCLCPP_WARN(get_node()->get_logger(), "Error in Casadi function call. Stopping the controller.");
                    }

                    tau_full = Eigen::VectorXd::Zero(N_DOF);
                }
            }
            else
            {
                if (invalid_counter < MAX_INVALID_COUNT)
                {
                    invalid_counter++;
                    RCLCPP_WARN(get_node()->get_logger(), "No valid Python data (%d/%d). Using previous torques.", invalid_counter, MAX_INVALID_COUNT);
                }
                else
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "No valid data received from Python %d times. Stopping the controller.", MAX_INVALID_COUNT);
                    tau_full = Eigen::VectorXd::Zero(N_DOF);
                    invalid_counter = 0;
                    mpc_started = false;
                }
            }
        }
        else
        {
            tau_full = Eigen::VectorXd::Zero(N_DOF);
        }

        // Wuse tau_full from last solution
        for (int i = 0; i < N_DOF; ++i)
        {
            command_interfaces_[i].set_value(tau_full[i]);
        }

        // Logging torques
        RCLCPP_INFO(get_node()->get_logger(), "tau (Nm): [%f, %f, %f, %f, %f, %f, %f]",
                    tau_full[0], tau_full[1], tau_full[2],
                    tau_full[3], tau_full[4], tau_full[5], tau_full[6]);

        return controller_interface::return_type::OK;
    }

    CallbackReturn ModelPredictiveControllerCasadi::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        arm_id_ = get_node()->get_parameter("arm_id").as_string();

        RCLCPP_INFO(get_node()->get_logger(), "Configuring MPC controller for %s arm", arm_id_.c_str());

        // subscription_ = get_node()->create_subscription<mpc_interfaces::msg::Num>(
        //     "topic", 10, std::bind(&ModelPredictiveControllerCasadi::topic_callback, this, _1));

        // RCLCPP_INFO(get_node()->get_logger(), "Subscribed to topic 'topic'");

        start_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            "start_mpc_service",
            std::bind(&ModelPredictiveControllerCasadi::start_mpc, this, std::placeholders::_1, std::placeholders::_2));

        reset_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            "reset_mpc_service",
            std::bind(&ModelPredictiveControllerCasadi::reset_mpc, this, std::placeholders::_1, std::placeholders::_2));

        stop_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            "stop_mpc_service",
            std::bind(&ModelPredictiveControllerCasadi::stop_mpc, this, std::placeholders::_1, std::placeholders::_2));

        traj_switch_service_ = get_node()->create_service<mpc_interfaces::srv::TrajectoryCommand>(
            "traj_switch_service",
            std::bind(&ModelPredictiveControllerCasadi::traj_switch, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_node()->get_logger(), "Service 'add_three_ints' created");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCasadi::on_init()
    {
        rcutils_ret_t ret = rcutils_logging_set_logger_level(
            get_node()->get_logger().get_name(), MY_LOG_LEVEL);
        if (ret != RCUTILS_RET_OK)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set logger level: %d", ret);
        }
        try
        {
            auto_declare<std::string>("arm_id", "panda");
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        try
        {
            open_shared_memories();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCasadi::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        close_shared_memories();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCasadi::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        open_shared_memories();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCasadi::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        close_shared_memories();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCasadi::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        close_shared_memories();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void ModelPredictiveControllerCasadi::open_shared_memories()
    {
        // Open shared memory for writing states to crocddyl
        shm_states = open_write_shm("data_from_simulink", 2 * N_DOF * sizeof(double), get_node()->get_logger());
        shm_states_valid = open_write_shm("data_from_simulink_valid", sizeof(int8_t), get_node()->get_logger());
        shm_start_mpc = open_write_shm("data_from_simulink_start", sizeof(int8_t), get_node()->get_logger());
        shm_reset_mpc = open_write_shm("data_from_simulink_reset", sizeof(int8_t), get_node()->get_logger());
        shm_stop_mpc = open_write_shm("data_from_simulink_stop", sizeof(int8_t), get_node()->get_logger());
        shm_select_trajectory = open_write_shm("data_from_simulink_traj_switch", sizeof(int8_t), get_node()->get_logger());

        shm_changed_semaphore = open_write_sem("shm_changed_semaphore", get_node()->get_logger());

        // Open shared memory for reading torques from crocddyl
        shm_torques = open_read_shm("data_from_python", get_node()->get_logger());
        shm_torques_valid = open_write_shm("data_from_python_valid", sizeof(int8_t), get_node()->get_logger());
        RCLCPP_INFO(get_node()->get_logger(), "Shared memory opened successfully.");
    }

    void ModelPredictiveControllerCasadi::close_shared_memories()
    {
        if (shm_states != -1)
            close(shm_states);
        if (shm_states_valid != -1)
            close(shm_states_valid);
        if (shm_start_mpc != -1)
            close(shm_start_mpc);
        if (shm_reset_mpc != -1)
            close(shm_reset_mpc);
        if (shm_stop_mpc != -1)
            close(shm_stop_mpc);
        if (shm_select_trajectory != -1)
            close(shm_select_trajectory);
        if (shm_torques != -1)
            close(shm_torques);
        if (shm_torques_valid != -1)
            close(shm_torques_valid);
    }

    // void ModelPredictiveControllerCasadi::topic_callback(const mpc_interfaces::msg::Num & msg)
    // {
    //   RCLCPP_WARN(get_node()->get_logger(), "I heard: '%ld'", msg.num);
    // }

    void ModelPredictiveControllerCasadi::start_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
                                                    std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        shm_flags flags = {
            1, // start
            0, // reset
            0, // stop
            0  // torques_valid
        };

        write_to_shared_memory(shm_start_mpc, &flags.start, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_reset_mpc, &flags.reset, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_stop_mpc, &flags.stop, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_torques_valid, &flags.torques_valid, sizeof(int8_t), get_node()->get_logger());

        response->status = "start flag set";

        double state[2 * N_DOF];
        for (int i = 0; i < 2 * N_DOF; ++i)
        {
            state[i] = state_interfaces_[i].get_value();
        }

        tau_full_future = std::async(std::launch::async, [this, state]()
                                     {
            Eigen::VectorXd tau_full_temp = controller.solveMPC(state);
            mpc_started = true;
            first_torque_read = false;
            sem_post(shm_changed_semaphore);
            return tau_full_temp; });
    }

    void ModelPredictiveControllerCasadi::reset_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
                                                    std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        shm_flags flags = {
            0, // start
            1, // reset
            0, // stop
            0  // torques_valid
        };

        write_to_shared_memory(shm_start_mpc, &flags.start, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_reset_mpc, &flags.reset, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_stop_mpc, &flags.stop, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_torques_valid, &flags.torques_valid, sizeof(int8_t), get_node()->get_logger());

        response->status = "reset flag set";
        mpc_started = false;
        sem_post(shm_changed_semaphore);
    }

    void ModelPredictiveControllerCasadi::stop_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
                                                   std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        shm_flags flags = {
            0, // start
            0, // reset
            1, // stop
            0  // torques_valid
        };

        write_to_shared_memory(shm_start_mpc, &flags.start, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_reset_mpc, &flags.reset, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_stop_mpc, &flags.stop, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_torques_valid, &flags.torques_valid, sizeof(int8_t), get_node()->get_logger());

        response->status = "stop flag set";
        mpc_started = false;
        sem_post(shm_changed_semaphore);
    }

    void ModelPredictiveControllerCasadi::traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
                                                      std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Response> response)
    {
        int8_t traj_select = request->traj_select;
        write_to_shared_memory(shm_select_trajectory, &traj_select, sizeof(int8_t), get_node()->get_logger());
        int8_t flag = 0;
        write_to_shared_memory(shm_start_mpc, &flag, sizeof(int8_t), get_node()->get_logger());
        flag = 1;
        write_to_shared_memory(shm_reset_mpc, &flag, sizeof(int8_t), get_node()->get_logger());
        flag = 0;
        write_to_shared_memory(shm_stop_mpc, &flag, sizeof(int8_t), get_node()->get_logger());
        response->status = "trajectory " + std::to_string(traj_select) + " selected";
        mpc_started = false;
        sem_post(shm_changed_semaphore);
    }

} // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ModelPredictiveControllerCasadi,
                       controller_interface::ControllerInterface)

int main(int argc, char **argv)
{
    // Your code here
    return 0;
}