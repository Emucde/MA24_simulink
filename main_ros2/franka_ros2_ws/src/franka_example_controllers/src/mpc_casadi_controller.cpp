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

        for (int i = 1; i <= nq; ++i)
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

    controller_interface::return_type ModelPredictiveControllerCasadi::update(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {   
        
        #ifndef SIMULATION_MODE
        // Read states from joint state interface
        for (int i = 0; i < nx; ++i)
        {
            state[i] = state_interfaces_[i].get_value();
        }
        x_measured = state;
        #else
        if(use_noise)
            x_measured = state + generateNoiseVector(nx, Ts, mean_noise_amplitude);
        else
            x_measured = state;
        #endif

        x_filtered = filter_x_measured();

        #ifdef DEBUG
        RCLCPP_INFO(get_node()->get_logger(), "q (rad): [%f, %f, %f, %f, %f, %f, %f]",
                    state[0], state[1], state[2],
                    state[3], state[4], state[5], state[6]);

        RCLCPP_INFO(get_node()->get_logger(), "q_p (rad/s): [%f, %f, %f, %f, %f, %f, %f]",
                    state[nq + 0], state[nq + 1], state[nq + 2],
                        state[nq + 3], state[nq + 4], state[nq + 5], state[nq + 6]);
        #endif

        if (controller_started)
        {
            if(solver_step_counter % solver_steps == 0)
                solve();
            
            if(solver_step_counter >= 1000)
                solver_step_counter = 0;
            solver_step_counter++;

            current_frequency = timer_solver.get_frequency()*solver_steps;
            shm.write("read_state_data_full", x_measured.data(), global_traj_count);
            shm.write("read_control_data", tau_full.data());
            shm.write("read_control_data_full", tau_full.data(), global_traj_count);
            shm.write("read_traj_data_full", current_trajectory->col(global_traj_count).data(), global_traj_count);
            shm.write("read_frequency_full", &current_frequency, global_traj_count);
            shm.post_semaphore("shm_changed_semaphore");

            if(global_traj_count < traj_len)
                global_traj_count++;

            error_flag = controller.get_error_flag(); // Get the error flag

            if (error_flag != ErrorFlag::NO_ERROR)
            {
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

                controller_started = false;
            }

            #ifdef SIMULATION_MODE
            controller.simulateModelRK4(state.data(), tau_full.data(), Ts);
            #else
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

    CallbackReturn ModelPredictiveControllerCasadi::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        arm_id_ = get_node()->get_parameter("arm_id").as_string();

        RCLCPP_INFO(get_node()->get_logger(), "Configuring MPC controller for %s arm", arm_id_.c_str());

        // subscription_ = get_node()->create_subscription<mpc_interfaces::msg::ControlArray>(
        //     "topic", 10, std::bind(&ModelPredictiveControllerCasadi::topic_callback, this, _1));

        // RCLCPP_INFO(get_node()->get_logger(), "Subscribed to topic 'topic'");

        std::string node_name = get_node()->get_name();
        std::string service_prefix = "/" + node_name + "/";

        start_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "start_mpc_service",
            std::bind(&ModelPredictiveControllerCasadi::start_mpc, this, std::placeholders::_1, std::placeholders::_2));

        reset_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "reset_mpc_service",
            std::bind(&ModelPredictiveControllerCasadi::reset_mpc, this, std::placeholders::_1, std::placeholders::_2));

        stop_mpc_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "stop_mpc_service",
            std::bind(&ModelPredictiveControllerCasadi::stop_mpc, this, std::placeholders::_1, std::placeholders::_2));

        traj_switch_service_ = get_node()->create_service<mpc_interfaces::srv::TrajectoryCommand>(
            service_prefix + "traj_switch_service",
            std::bind(&ModelPredictiveControllerCasadi::traj_switch, this, std::placeholders::_1, std::placeholders::_2));

        mpc_switch_service_ = get_node()->create_service<mpc_interfaces::srv::CasadiMPCTypeCommand>(
            service_prefix + "mpc_switch_service",
            std::bind(&ModelPredictiveControllerCasadi::mpc_switch, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_node()->get_logger(), "Services created");

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

        timer_all.tic();
        timer_solver.tic();

        RCLCPP_INFO(get_node()->get_logger(), "MPC controller initialized");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCasadi::on_deactivate(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_deactivate: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCasadi::on_activate(const rclcpp_lifecycle::State &)
    {
        open_shared_memories();
        init_controller();
        int8_t readonly_mode = 1;
        shm.write("readonly_mode", &readonly_mode);
        RCLCPP_INFO(get_node()->get_logger(), "on_activate: Shared memory opened successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCasadi::on_cleanup(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_cleanup: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ModelPredictiveControllerCasadi::on_shutdown(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_shutdown: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void ModelPredictiveControllerCasadi::open_shared_memories()
    {
        shm.open_readwrite_shms(shm_readwrite_infos);
        shm.open_readwrite_sems(sem_readwrite_names);
        RCLCPP_INFO(get_node()->get_logger(), "Shared memory opened successfully.");
    }

    void ModelPredictiveControllerCasadi::close_shared_memories()
    {
        shm.close_shared_memories();
        shm.close_semaphores();
    }

    // void ModelPredictiveControllerCasadi::topic_callback(const mpc_interfaces::msg::ControlArray & msg)
    // {
    //     Eigen::Map<const Eigen::VectorXd> u_k(msg.control_array.data(), msg.control_array.size());
    //     RCUTILS_LOG_WARN("Received control array: [%f, %f, %f, %f, %f, %f]",
    //                     u_k[0], u_k[1], u_k[2], u_k[3], u_k[4], u_k[5]);
    // }

    void ModelPredictiveControllerCasadi::start_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
                                                    std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        shm_flags flags = {
            1, // start
            0, // reset
            0, // stop
            0  // torques_valid
        };

        shm.write("data_from_simulink_reset", &flags.reset);
        shm.write("data_from_simulink_stop", &flags.stop);
        shm.write("data_from_simulink_start", &flags.start);

        response->status = "start flag set";

        #ifndef SIMULATION_MODE
        for (int i = 0; i < nx; ++i)
            state[i] = state_interfaces_[i].get_value();
        x_measured = state;
        #endif

        init_controller();

        filter_x_measured(); // uses x_measured

        if(first_start)
        {
            reset_trajectory();
            init_trajectory();
            tau_full = controller.update_control(x_filtered);
            controller.set_traj_count(0);
            tau_full = controller.update_control(x_filtered);
            controller.set_traj_count(0);
            first_start = false;
        }
        
        int8_t readonly_mode = 1;
        shm.write("read_traj_length", &traj_len);
        shm.write("readonly_mode", &readonly_mode);

        controller_started = true;
    }

    void ModelPredictiveControllerCasadi::reset_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
                                                    std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        shm_flags flags = {
            0, // start
            1, // reset
            0, // stop
            0  // torques_valid
        };

        shm.write("data_from_simulink_start", &flags.start);
        shm.write("data_from_simulink_stop", &flags.stop);
        shm.write("data_from_simulink_reset", &flags.reset);

        reset_trajectory();

        response->status = "reset flag set";
        controller_started = false;
        shm.post_semaphore("shm_changed_semaphore");
        RCLCPP_INFO(get_node()->get_logger(), "CasAdi MPC reset");
    }

    void ModelPredictiveControllerCasadi::stop_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
                                                   std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        shm_flags flags = {
            0, // start
            0, // reset
            1, // stop
            0  // torques_valid
        };

        shm.write("data_from_simulink_start", &flags.start);
        shm.write("data_from_simulink_reset", &flags.reset);
        shm.write("data_from_simulink_stop", &flags.stop);
        shm.post_semaphore("shm_changed_semaphore");

        response->status = "stop flag set";
        controller_started = false;
        RCLCPP_INFO(get_node()->get_logger(), "CasAdi MPC stopped");
    }

    void ModelPredictiveControllerCasadi::traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
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

    void ModelPredictiveControllerCasadi::mpc_switch(const std::shared_ptr<mpc_interfaces::srv::CasadiMPCTypeCommand::Request> request,
                                                         std::shared_ptr<mpc_interfaces::srv::CasadiMPCTypeCommand::Response> response)
    {
        CasadiMPCType mpc_type = static_cast<CasadiMPCType>(request->mpc_type);
        CasadiMPCType current_mpc_type = controller.get_active_mpc_type();
        if (mpc_type == current_mpc_type)
        {
            response->status = "MPC type " + std::to_string(request->mpc_type) + " already selected";
            RCLCPP_INFO(get_node()->get_logger(), "MPC type %ld already selected", request->mpc_type);
        }
        else
        {
            controller.setActiveMPC(mpc_type);
            reset_trajectory();

            response->status = "MPC type " + std::to_string(request->mpc_type) + " selected";
            std::string status_message = "Switched to Casadi " + casadi_mpctype_to_string(mpc_type);
            RCLCPP_INFO(get_node()->get_logger(), status_message.c_str());
            response->status = status_message;
        }
    }

    void ModelPredictiveControllerCasadi::solve()
    {
        timer_solver.tic();
        tau_full = controller.update_control(x_filtered);
        timer_solver.toc();
    }

} // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ModelPredictiveControllerCasadi,
                       controller_interface::ControllerInterface)