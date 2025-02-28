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

#include <franka_example_controllers/mpc_crocoddyl_controller.hpp>

#include <exception>
#include <string>
#include <unistd.h>

#include <memory>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace franka_example_controllers
{
    controller_interface::InterfaceConfiguration
    ModelPredictiveControllerCrocoddyl::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (int i = 1; i <= nq; ++i)
        {
            config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
        }
        return config;
    }

    controller_interface::InterfaceConfiguration
    ModelPredictiveControllerCrocoddyl::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (int i = 1; i <= nq; ++i)
        {
            state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
        }
        for (int i = 1; i <= nq; ++i)
        {
            state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
        }
        return state_interfaces_config;
    }

    controller_interface::return_type ModelPredictiveControllerCrocoddyl::update(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {   
        if (controller_started)
        {   
            valid_cpp = read_valid_cpp_shm();
            if (valid_cpp == 1)
            {
#ifndef SIMULATION_MODE
                // Read states from joint state interface
                for (int i = 0; i < nx; ++i)
                {
                    state[i] = state_interfaces_[i].get_value();
                }
                x_measured = state;
#endif
        
                write_valid_cpp_shm(0);
                shm.read_double("cpp_control_data", tau_full.data());
                shm.read_int8("error_cpp", &error_flag_int8);
                error_flag = static_cast<ErrorFlag>(error_flag_int8);

                if (error_flag != ErrorFlag::NO_ERROR)
                {
                    if (error_flag == ErrorFlag::JUMP_DETECTED)
                        RCLCPP_WARN(get_node()->get_logger(), "Jump in torque detected. Stopping the controller.");
                    else if (error_flag == ErrorFlag::NAN_DETECTED)
                        RCLCPP_WARN(get_node()->get_logger(), "NaN in torque detected. Stopping the controller.");
                    else if (error_flag == ErrorFlag::CROCODDYL_ERROR)
                        RCLCPP_WARN(get_node()->get_logger(), "Error in Crocoddyl function call. Stopping the controller.");
                    error_flag = ErrorFlag::NO_ERROR;
                    int8_t stop = 1;
                    shm.feedback_write_int8("stop_cpp", &stop);
                    controller_started = false;
                }
#ifdef SIMULATION_MODE
                base_controller->simulateModelRK4(state.data(), tau_full.data(), Ts);
                if(use_noise)
                    x_measured = state + generateNoiseVector(nx, Ts, mean_noise_amplitude);
                else
                    x_measured = state;
#endif
                shm.write("ros2_state_data", x_measured.data());
                shm.post_semaphore("ros2_semaphore");
            }

#ifndef SIMULATION_MODE
            for (int i = 0; i < nq; ++i) {
                command_interfaces_[i].set_value(tau_full[i]);
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

    CallbackReturn ModelPredictiveControllerCrocoddyl::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        arm_id_ = get_node()->get_parameter("arm_id").as_string();

        RCLCPP_INFO(get_node()->get_logger(), "Configuring MPC controller for %s arm", arm_id_.c_str());

        // subscription_ = get_node()->create_subscription<mpc_interfaces::msg::ControlArray>(
        //     "topic", 10, std::bind(&ModelPredictiveControllerCrocoddyl::topic_callback, this, _1));

        // RCLCPP_INFO(get_node()->get_logger(), "Subscribed to topic 'topic'");

        std::string node_name = get_node()->get_name();
        std::string service_prefix = "/" + node_name + "/";

        start_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "start_mpc_service",
            std::bind(&ModelPredictiveControllerCrocoddyl::start_mpc, this, std::placeholders::_1, std::placeholders::_2));

        reset_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "reset_mpc_service",
            std::bind(&ModelPredictiveControllerCrocoddyl::reset_mpc, this, std::placeholders::_1, std::placeholders::_2));

        stop_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "stop_mpc_service",
            std::bind(&ModelPredictiveControllerCrocoddyl::stop_mpc, this, std::placeholders::_1, std::placeholders::_2));

        traj_switch_service_ = get_node()->create_service<mpc_interfaces::srv::TrajectoryCommand>(
            service_prefix + "traj_switch_service",
            std::bind(&ModelPredictiveControllerCrocoddyl::traj_switch, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_node()->get_logger(), "Services created");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCrocoddyl::on_init()
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

        timer_all.tic();
        timer_solver.tic();

        RCLCPP_INFO(get_node()->get_logger(), "MPC controller initialized");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCrocoddyl::on_deactivate(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_deactivate: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCrocoddyl::on_activate(const rclcpp_lifecycle::State &)
    {
        open_shared_memories();
        init_controller();
        int8_t readonly_mode = 1;
        shm.feedback_write_int8("readonly_mode", &readonly_mode);
        RCLCPP_INFO(get_node()->get_logger(), "on_activate: Shared memory opened successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCrocoddyl::on_cleanup(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_cleanup: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCrocoddyl::on_shutdown(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_shutdown: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void ModelPredictiveControllerCrocoddyl::open_shared_memories()
    {
        shm.open_readwrite_shms(shm_readwrite_infos);
        shm.open_readwrite_sems(sem_readwrite_names);
        ros2_semaphore = shm.get_semaphore("ros2_semaphore");
        valid_cpp_shm = shm.get_shared_memory("valid_cpp");
        RCLCPP_INFO(get_node()->get_logger(), "Shared memory opened successfully.");
    }

    void ModelPredictiveControllerCrocoddyl::close_shared_memories()
    {
        shm.close_shared_memories();
        shm.close_semaphores();
    }

    // void ModelPredictiveControllerCrocoddyl::topic_callback(const mpc_interfaces::msg::ControlArray & msg)
    // {
    //     Eigen::Map<const Eigen::VectorXd> u_k(msg.control_array.data(), msg.control_array.size());
    //     RCUTILS_LOG_WARN("Received control array: [%f, %f, %f, %f, %f, %f]",
    //                     u_k[0], u_k[1], u_k[2], u_k[3], u_k[4], u_k[5]);
    // }

    void ModelPredictiveControllerCrocoddyl::start_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
                                                    std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        shm_flags flags = {
            1, // start
            0, // reset
            0, // stop
            0  // torques_valid
        };

        shm.write("start_cpp", &flags.start);
        shm.write("reset_cpp", &flags.reset);
        shm.write("stop_cpp", &flags.stop);

        init_controller();
        init_trajectory();

#ifndef SIMULATION_MODE
        for (int i = 0; i < nx; ++i)
            state[i] = state_interfaces_[i].get_value();
#else
        if(first_start)
        {
            Eigen::VectorXd x0_init = base_controller->get_file_traj_x0_nq_init(traj_select);
            state = x0_init;
        }
#endif
        if(first_start)
        {
            shm.write("ros2_state_data", state.data());
            first_start = false;
        }

        tau_full = Eigen::VectorXd::Zero(nq);

        shm.feedback_write_int8("valid_cpp", &flags.torques_valid);

        shm.post_semaphore("ros2_semaphore");
        controller_started = true;
        response->status = "start flag set";
    }

    void ModelPredictiveControllerCrocoddyl::reset_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
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
        shm.feedback_write_int8("reset_cpp", &flags.reset);
        shm.feedback_write_int8("stop_cpp", &flags.stop);

        reset_trajectory();
        first_start = true;

        response->status = "reset flag set";
        controller_started = false;
        solve_started = false;
        shm.post_semaphore("shm_changed_semaphore");
        shm.post_semaphore("ros2_semaphore");
        RCLCPP_INFO(get_node()->get_logger(), "Crocoddyl MPC reset");
    }

    void ModelPredictiveControllerCrocoddyl::stop_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
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

        shm.feedback_write_int8("start_cpp", &flags.start);
        shm.feedback_write_int8("reset_cpp", &flags.reset);
        shm.feedback_write_int8("stop_cpp", &flags.stop);

        shm.post_semaphore("shm_changed_semaphore");
        shm.post_semaphore("ros2_semaphore");

        response->status = "stop flag set";
        controller_started = false;
        solve_started = false;
        RCLCPP_INFO(get_node()->get_logger(), "Crocoddyl MPC stopped");
    }

    void ModelPredictiveControllerCrocoddyl::traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
                                                      std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Response> response)
    {
        int old_traj_select = traj_select;
        traj_select = request->traj_select; // it is later used at start_mpc() in init_trajectory
        if(old_traj_select == traj_select)
        {
            response->status = "trajectory " + std::to_string(traj_select) + " already selected";
            RCLCPP_INFO(get_node()->get_logger(), "Trajectory %d already selected", traj_select);
        }
        else
        {
            controller.switch_traj(traj_select);
            reset_trajectory();

            response->status = "trajectory " + std::to_string(traj_select) + " selected";
            controller_started = false;
            first_start = true;
            
            RCLCPP_INFO(get_node()->get_logger(), "Trajectory %d selected", traj_select);
        }
    }

} // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ModelPredictiveControllerCrocoddyl,
                       controller_interface::ControllerInterface)