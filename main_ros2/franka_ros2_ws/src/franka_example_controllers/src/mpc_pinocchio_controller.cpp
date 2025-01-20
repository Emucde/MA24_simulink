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

#include <franka_example_controllers/mpc_pinocchio_controller.hpp>

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
    ModelPredictiveControllerPinocchio::command_interface_configuration() const
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
    ModelPredictiveControllerPinocchio::state_interface_configuration() const
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

    controller_interface::return_type ModelPredictiveControllerPinocchio::update(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {
        double *torques = 0;
        double read_torques[N_DOF] = {0.0};
        double zero_torques[N_DOF] = {0.0};
        int8_t valid_flag = 0;

        double state[2 * N_DOF];

        /*
         * start flag sended: mpc_started = true, first_torque_read = false
         * 1. Read states from joint state interface
         * 2. Write combined state (q, q_p) to shared memory (send data to crocoddyl)
         * 3. Write validity flag for state data to shared memory
         * ... Parallel process in Python:
         *     3.1 Python waits until the state_valid flag is set to 1
         *     3.2 Python reads the state data from shared memory
         *     3.3 Python initializes the MPC controller and initial trajectory
         *     3.4 Python solves the MPC problem and writes the torques to shared memory
         *     3.5 Python sets the torques_valid flag to 1
         * 4. Read torque_valid_flag
         * 5. If first_torque_read is false, waiting as long as torque_valid_flag is 1
         * 6. ... (3.1 to 3.5)
         * 7. If torque_valid_flag is 1, then first_torque_read is set to true.
         * 8. Reading torque data from shared memory until end of the control loop
         */

        if (mpc_started)
        {
            // Read states from joint state interface
            for (int i = 0; i < 2 * N_DOF; ++i)
            {
                state[i] = state_interfaces_[i].get_value();
            }

            Eigen::Map<Eigen::VectorXd> q_measured(state, N_DOF);
            Eigen::Map<Eigen::VectorXd> q_p_measured(state + N_DOF, N_DOF);
            Eigen::Map<Eigen::VectorXd> q_filtered(x_filtered.data(), N_DOF);
            Eigen::Map<Eigen::VectorXd> q_p_filtered(x_filtered.data() + N_DOF, N_DOF);
            
            q_filtered = A1 * q_filtered + B1 * q_measured;
            q_p_filtered = A2 * q_p_filtered + B2 * q_p_measured;


            // Write combined state (q, q_p) to shared memory (send data to crocoddyl)
            write_to_shared_memory(shm_states, state, sizeof(state), get_node()->get_logger());

            // Write validity flag for state data to shared memory
            valid_flag = 1;
            write_to_shared_memory(shm_states_valid, &valid_flag, sizeof(int8_t), get_node()->get_logger());

            #ifdef DEBUG
            // Logging joint states
            RCLCPP_INFO(get_node()->get_logger(), "q (rad): [%f, %f, %f, %f, %f, %f, %f]",
                        state[0], state[1], state[2],
                        state[3], state[4], state[5], state[6]);

            RCLCPP_INFO(get_node()->get_logger(), "q_p (rad/s): [%f, %f, %f, %f, %f, %f, %f]",
                        state[N_DOF + 0], state[N_DOF + 1], state[N_DOF + 2],
                        state[N_DOF + 3], state[N_DOF + 4], state[N_DOF + 5], state[N_DOF + 6]);
            #endif

            // Read validity flag for torque data from shared memory
            valid_flag = 0;
            read_shared_memory_flag(shm_torques_valid, &valid_flag, sizeof(int8_t), get_node()->get_logger());

            if (valid_flag == 0 && !first_torque_read)
            {
                RCLCPP_INFO(get_node()->get_logger(), "No valid Python data received yet. Using zero torques.");
                torques = &zero_torques[0];
            }
            else if (valid_flag == 1)
            {
                first_torque_read = true;

                // Reading data from shared memory
                read_shared_memory(shm_torques, &read_torques[0], N_DOF * sizeof(double), get_node()->get_logger());
                std::memcpy(torques_prev, read_torques, N_DOF * sizeof(double));
                torques = &read_torques[0];

                invalid_counter = 0;
                valid_flag = 0;
                write_to_shared_memory(shm_torques_valid, &valid_flag, sizeof(int8_t), get_node()->get_logger());
            }
            else
            {
                // wenn die Echtzeitbedingung nicht erfüllt ist, wird der vorherige Torque-Wert verwendet
                if (invalid_counter < MAX_INVALID_COUNT)
                {
                    torques = &torques_prev[0];
                    invalid_counter++;
                    RCLCPP_WARN(get_node()->get_logger(), "No valid Python data (%d/%d). Using previous torques.", invalid_counter, MAX_INVALID_COUNT);
                }
                else
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "No valid data received from Python %d times. Stopping the controller.", MAX_INVALID_COUNT);
                    torques = &zero_torques[0];
                    invalid_counter = 0;
                    mpc_started = false;
                }
            }
        }
        else
        {
            torques = &zero_torques[0];
        }

        // Write torques to shared memory (send data to robot)
        for (int i = 0; i < N_DOF; ++i)
        {
            command_interfaces_[i].set_value(torques[i]);
        }

        #ifdef DEBUG
        // Logging torques
        RCLCPP_INFO(get_node()->get_logger(), "tau (Nm): [%f, %f, %f, %f, %f, %f, %f]",
                    torques[0], torques[1], torques[2],
                    torques[3], torques[4], torques[5], torques[6]);
        #endif

        sem_post(shm_changed_semaphore);
        return controller_interface::return_type::OK;
    }

    CallbackReturn ModelPredictiveControllerPinocchio::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        arm_id_ = get_node()->get_parameter("arm_id").as_string();

        RCLCPP_INFO(get_node()->get_logger(), "Configuring MPC controller for %s arm", arm_id_.c_str());

        // subscription_ = get_node()->create_subscription<mpc_interfaces::msg::Num>(
        //     "topic", 10, std::bind(&ModelPredictiveControllerPinocchio::topic_callback, this, _1));

        // RCLCPP_INFO(get_node()->get_logger(), "Subscribed to topic 'topic'");

        std::string node_name = get_node()->get_name();
        std::string service_prefix = "/" + node_name + "/";

        start_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "start_mpc_service",
            std::bind(&ModelPredictiveControllerPinocchio::start_mpc, this, std::placeholders::_1, std::placeholders::_2));

        reset_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "reset_mpc_service",
            std::bind(&ModelPredictiveControllerPinocchio::reset_mpc, this, std::placeholders::_1, std::placeholders::_2));

        stop_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "stop_mpc_service",
            std::bind(&ModelPredictiveControllerPinocchio::stop_mpc, this, std::placeholders::_1, std::placeholders::_2));

        traj_switch_service_ = get_node()->create_service<mpc_interfaces::srv::TrajectoryCommand>(
            service_prefix + "traj_switch_service",
            std::bind(&ModelPredictiveControllerPinocchio::traj_switch, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_node()->get_logger(), "Services created");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerPinocchio::on_init()
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
            int8_t readonly_mode = 0;
            open_shared_memories();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "Pinocchio MPC controller initialized.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerPinocchio::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_deactivate: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerPinocchio::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        open_shared_memories();
        int8_t readonly_mode = 0;
        write_to_shared_memory(shm_readonly_mode, &readonly_mode, sizeof(int8_t), get_node()->get_logger());
        RCLCPP_INFO(get_node()->get_logger(), "on_activate: Shared memory opened successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerPinocchio::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_cleanup: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerPinocchio::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_shutdown: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void ModelPredictiveControllerPinocchio::open_shared_memories()
    {
        // Open shared memory for writing states to crocddyl
        shm_states = open_write_shm("data_from_simulink", get_node()->get_logger());
        shm_states_valid = open_write_shm("data_from_simulink_valid", get_node()->get_logger());
        shm_start_mpc = open_write_shm("data_from_simulink_start", get_node()->get_logger());
        shm_reset_mpc = open_write_shm("data_from_simulink_reset", get_node()->get_logger());
        shm_stop_mpc = open_write_shm("data_from_simulink_stop", get_node()->get_logger());
        shm_select_trajectory = open_write_shm("data_from_simulink_traj_switch", get_node()->get_logger());
        shm_readonly_mode = open_write_shm("readonly_mode", get_node()->get_logger());

        shm_changed_semaphore = open_write_sem("shm_changed_semaphore", get_node()->get_logger());

        // Open shared memory for reading torques from crocddyl
        shm_torques = open_read_shm("data_from_python", get_node()->get_logger());
        shm_torques_valid = open_write_shm("data_from_python_valid", get_node()->get_logger());
        RCLCPP_INFO(get_node()->get_logger(), "Shared memory opened successfully.");
    }

    void ModelPredictiveControllerPinocchio::close_shared_memories()
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
        if (shm_changed_semaphore != 0)
            sem_close(shm_changed_semaphore);
    }

    // void ModelPredictiveControllerPinocchio::topic_callback(const mpc_interfaces::msg::Num & msg)
    // {
    //   RCLCPP_WARN(get_node()->get_logger(), "I heard: '%ld'", msg.num);
    // }

    void ModelPredictiveControllerPinocchio::start_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
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

        double state[2 * N_DOF];
        for (int i = 0; i < 2 * N_DOF; ++i)
        {
            state[i] = state_interfaces_[i].get_value();
        }

        Eigen::VectorXd state_eig = Eigen::Map<Eigen::VectorXd>(state, 2 * N_DOF);
        x_filtered = state_eig; // set x0 for Lowpass Filter

        response->status = "start flag set";
        mpc_started = true;
        first_torque_read = false;
        sem_post(shm_changed_semaphore);
        RCLCPP_INFO(get_node()->get_logger(), "Pinocchio MPC started");
    }

    void ModelPredictiveControllerPinocchio::reset_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
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
        RCLCPP_INFO(get_node()->get_logger(), "Pinocchio MPC reset");
    }

    void ModelPredictiveControllerPinocchio::stop_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request> request,
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
        RCLCPP_INFO(get_node()->get_logger(), "Pinocchio MPC stopped");
    }

    void ModelPredictiveControllerPinocchio::traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
                                                         std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Response> response)
    {
        int8_t traj_select = request->traj_select;
        
        shm_flags flags = {
            0, // start
            1, // reset
            0, // stop
            0  // torques_valid
        };

        write_to_shared_memory(shm_select_trajectory, &traj_select, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_start_mpc, &flags.start, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_reset_mpc, &flags.reset, sizeof(int8_t), get_node()->get_logger());
        write_to_shared_memory(shm_stop_mpc, &flags.stop, sizeof(int8_t), get_node()->get_logger());
        
        response->status = "trajectory " + std::to_string(traj_select) + " selected";
        mpc_started = false;
        sem_post(shm_changed_semaphore);
        RCLCPP_INFO(get_node()->get_logger(), "Pinocchio MPC trajectory %d selected", traj_select);
    }

} // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ModelPredictiveControllerPinocchio,
                       controller_interface::ControllerInterface)
