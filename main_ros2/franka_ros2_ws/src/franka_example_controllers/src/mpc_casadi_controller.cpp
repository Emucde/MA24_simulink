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

#ifndef SIMULATION_MODE
    controller_interface::return_type ModelPredictiveControllerCasadi::update(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {
        if (mpc_started)
        {
            // Read states from joint state interface
            for (int i = 0; i < 2 * N_DOF; ++i)
            {
                state[i] = state_interfaces_[i].get_value();
            }
            x_measured = Eigen::Map<Eigen::VectorXd>(state, 2 * N_DOF);

            //TODO
            Eigen::VectorXd x_filtered;
            if(use_ekf && use_lowpass_filter)
            {
                lowpass_filter.run(state);
                ekf.predict(tau_full.data(), x_filtered_lowpass_ptr);
                x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_ekf_ptr, 2 * N_DOF);
            }
            else if(use_lowpass_filter)
            {
                lowpass_filter.run(state); // updates data from x_filtered_ptr
                x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_lowpass_ptr, 2 * N_DOF);
            }
            else if(use_ekf)
            {
                ekf.predict(tau_full.data(), state);
                x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_ekf_ptr, 2 * N_DOF);
            }
            else
            {
                x_filtered = Eigen::Map<Eigen::VectorXd>(state, 2 * N_DOF);
            }

            // Logging joint states
            #ifdef DEBUG
            RCLCPP_INFO(get_node()->get_logger(), "q (rad): [%f, %f, %f, %f, %f, %f, %f]",
                        state[0], state[1], state[2],
                        state[3], state[4], state[5], state[6]);

            RCLCPP_INFO(get_node()->get_logger(), "q_p (rad/s): [%f, %f, %f, %f, %f, %f, %f]",
                        state[N_DOF + 0], state[N_DOF + 1], state[N_DOF + 2],
                        state[N_DOF + 3], state[N_DOF + 4], state[N_DOF + 5], state[N_DOF + 6]);
            #endif

            // Attempt to get the result (blocking call)
            if (tau_full_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
            {
                invalid_counter = 0;                      // Reset the counter on successful retrieval
                tau_full = tau_full_future.get();         // Get the result
                tau_prev = tau_full;                      // Update the previous torque
                x_prev = x_filtered;                      // Update the previous state
                error_flag = controller.get_error_flag(); // Get the error flag
                current_frequency = timer_mpc_solver.get_frequency();
                

                if (error_flag == ErrorFlag::NO_ERROR)
                {
                    tau_full_future = std::async(std::launch::async, [this, x_filtered]()
                                                 {
                                                    timer_mpc_solver.tic();
                                                    Eigen::VectorXd tau_temp = controller.solveMPC(x_filtered.data());
                                                    timer_mpc_solver.toc();
                                                    return tau_temp; });
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
                current_frequency = 0.0;
                controller.set_traj_count(global_traj_count); // sync the global trajectory count

                if (invalid_counter < MAX_INVALID_COUNT)
                {
                    invalid_counter++;
                    //Interpolate previous u_next
                    // Eigen::VectorXd u_next = Eigen::Map<Eigen::VectorXd>(u_next_ptr, robot_config.nq_red);
                    // if(invalid_counter <= static_cast<int>(N_step))
                    // {
                    //     tau_full = tau_prev + static_cast<double>(invalid_counter) / N_step * (torque_mapper->calc_full_torque(u_next, x_prev) - tau_prev);
                    // }
                    // tau_full = tau_prev;
                    RCLCPP_WARN(get_node()->get_logger(), "No valid MPC data (%d/%d). Using previous torques.", invalid_counter, MAX_INVALID_COUNT);
                }
                else
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "No valid MPC data %d times. Stopping the controller.", MAX_INVALID_COUNT);
                    tau_full = Eigen::VectorXd::Zero(N_DOF);
                    invalid_counter = 0;
                    mpc_started = false;
                }
            }

            RCLCPP_WARN(get_node()->get_logger(), "time: %f", current_Time);
            // Send data to shared memory
            shm.write("read_state_data_full", state, global_traj_count);
            shm.write("read_control_data_full", tau_full.data(), global_traj_count);
            shm.write("read_traj_data_full", controller.get_act_traj_data(), global_traj_count);
            shm.write("read_frequency_full", &current_frequency, global_traj_count);
            shm.post_semaphore("shm_changed_semaphore");

            if(global_traj_count < traj_len)
                global_traj_count++;
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
        #ifdef DEBUG
        RCLCPP_INFO(get_node()->get_logger(), "tau (Nm): [%f, %f, %f, %f, %f, %f, %f]",
                    tau_full[0], tau_full[1], tau_full[2],
                    tau_full[3], tau_full[4], tau_full[5], tau_full[6]);
        #endif

        return controller_interface::return_type::OK;
    }
#else
    controller_interface::return_type ModelPredictiveControllerCasadi::update(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {   
        
        #ifndef SIMULATION_MODE
        // Read states from joint state interface
        for (int i = 0; i < 2 * N_DOF; ++i)
        {
            state[i] = state_interfaces_[i].get_value();
        }
        #endif

        if (mpc_started)
        {
            // Attempt to get the result (blocking call)
            if (tau_full_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
            {
                invalid_counter = 0;                      // Reset the counter on successful retrieval
                tau_full = tau_full_future.get();         // Get the result
                tau_prev = tau_full;                      // Update the previous torque
                error_flag = controller.get_error_flag(); // Get the error flag
                current_frequency = timer_mpc_solver.get_frequency();
                
                // simulate one step
                #ifndef SIMULATION_MODE
                controller.simulateModelRK4(x_filtered.data(), tau_full.data(), Ts);
                #else
                controller.simulateModelRK4(state, tau_full.data(), Ts);

                if(use_noise)
                {
                    Eigen::Map<Eigen::VectorXd> state_eig(state, 2 * N_DOF);
                    x_measured << state_eig + generateNoiseVector(2*N_DOF, Ts, mean_noise_amplitude);
                }
                else
                {
                    x_measured = Eigen::Map<const Eigen::VectorXd>(state, 2 * N_DOF);
                }
                #endif

                Eigen::VectorXd x_filtered;
                if(use_ekf && use_lowpass_filter)
                {
                    ekf.predict(tau_full.data(), x_measured.data());
                    lowpass_filter.run(x_filtered_ekf_ptr);
                    x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_lowpass_ptr, 2 * N_DOF);
                }
                else if(use_lowpass_filter)
                {
                    lowpass_filter.run(x_measured.data()); // updates data from x_filtered_ptr
                    x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_lowpass_ptr, 2 * N_DOF);
                }
                else if(use_ekf)
                {
                    ekf.predict(tau_full.data(), x_measured.data());
                    x_filtered_ekf_ptr = ekf.get_x_k_plus_ptr();
                    x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_ekf_ptr, 2 * N_DOF);
                }
                else
                {
                    x_filtered = Eigen::Map<Eigen::VectorXd>(x_measured.data(), 2 * N_DOF);
                }
                
                if (error_flag == ErrorFlag::NO_ERROR)
                {
                    tau_full_future = std::async(std::launch::async, [this, x_filtered]()
                                                 {
                                                    timer_mpc_solver.tic();
                                                    Eigen::VectorXd tau_temp = controller.solveMPC(x_filtered.data());
                                                    timer_mpc_solver.toc();
                                                    return tau_temp; });
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
                current_frequency = 0.0;
                controller.set_traj_count(global_traj_count); // sync the global trajectory count

                if (invalid_counter < MAX_INVALID_COUNT)
                {
                    invalid_counter++;
                    //Interpolate previous u_next
                    // Eigen::VectorXd u_next = Eigen::Map<Eigen::VectorXd>(u_next_ptr, robot_config.nq_red);
                    // if(invalid_counter <= static_cast<int>(N_step))
                    // {
                    //     tau_full = tau_prev + static_cast<double>(invalid_counter) / N_step * (torque_mapper->calc_full_torque(u_next, x_prev) - tau_prev);
                    // }
                    // tau_full = tau_prev;
                    RCLCPP_WARN(get_node()->get_logger(), "No valid MPC data (%d/%d). Using previous torques.", invalid_counter, MAX_INVALID_COUNT);
                }
                else
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "No valid MPC data %d times. Stopping the controller.", MAX_INVALID_COUNT);
                    tau_full = Eigen::VectorXd::Zero(N_DOF);
                    invalid_counter = 0;
                    mpc_started = false;
                }
                
                #ifdef SIMULATION_MODE
                controller.simulateModelRK4(state, tau_full.data(), Ts);
                #endif
            }

            shm.write("read_state_data_full", x_measured.data(), global_traj_count);
            shm.write("read_control_data_full", tau_full.data(), global_traj_count);
            shm.write("read_traj_data_full", current_trajectory->col(global_traj_count).data(), global_traj_count);
            shm.write("read_frequency_full", &current_frequency, global_traj_count);
            shm.post_semaphore("shm_changed_semaphore");

            if(global_traj_count < traj_len)
                global_traj_count++;

            #ifdef SIMULATION_MODE
            //controller.simulateModelRK4(state, tau_full.data(), Ts);
            #endif
        }
        else
        {
            tau_full = Eigen::VectorXd::Zero(N_DOF);
            // if(use_ekf && use_lowpass_filter)
            // {
            //     ekf.predict(tau_full.data(), state);
            //     lowpass_filter.run(x_filtered_ekf_ptr);
            // }
            // else if(use_lowpass_filter)
            // {
            //     lowpass_filter.run(state); // updates data from x_filtered_ptr
            // }
            // else if(use_ekf)
            // {
            //     ekf.predict(tau_full.data(), state);
            // }
        }

        return controller_interface::return_type::OK;
    }
#endif

    CallbackReturn ModelPredictiveControllerCasadi::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        arm_id_ = get_node()->get_parameter("arm_id").as_string();

        RCLCPP_INFO(get_node()->get_logger(), "Configuring MPC controller for %s arm", arm_id_.c_str());

        // subscription_ = get_node()->create_subscription<mpc_interfaces::msg::Num>(
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

        controller.setActiveMPC(CasadiMPCType::MPC8);
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

    // void ModelPredictiveControllerCasadi::topic_callback(const mpc_interfaces::msg::Num & msg)
    // {
    //   RCLCPP_WARN(get_node()->get_logger(), "I heard: '%ld'", msg.num);
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

        // Update general configuration
        nlohmann::json general_config = read_config(general_config_filename);
        use_lowpass_filter = general_config["use_lowpass_filter"];
        use_ekf = general_config["use_ekf"];
        traj_select = general_config["trajectory_selection"];
        double T_traj_start = general_config["transient_traj_start_time"];
        double T_traj_dur = general_config["transient_traj_duration"];
        double T_traj_end = general_config["transient_traj_end_time"];
        bool use_planner = general_config["use_casadi_planner"];
        double omega_c_q = general_config["lowpass_filter_omega_c_q"];
        double omega_c_dq = general_config["lowpass_filter_omega_c_dq"];

        controller.setActiveMPC(string_to_casadi_mpctype(general_config["default_casadi_mpc"]));

        #ifndef SIMULATION_MODE
        controller.init_file_trajectory(traj_select, state, T_traj_start, T_traj_dur, T_traj_end);
        #else
        mean_noise_amplitude = general_config["mean_noise_amplitude"];
        use_noise = general_config["use_noise"];
        Eigen::VectorXd q_0_ref = Eigen::Map<const Eigen::VectorXd>(robot_config.q_0_ref, N_DOF);
        Eigen::VectorXd q_0_p_ref = Eigen::Map<const Eigen::VectorXd>(robot_config.q_0_p_ref, N_DOF);
        Eigen::VectorXd x0_ref = Eigen::VectorXd::Zero(2 * N_DOF);
        x0_ref << q_0_ref, q_0_p_ref;
        controller.init_file_trajectory(traj_select, x0_ref.data(), T_traj_start, T_traj_dur, T_traj_end);
        #endif
        traj_len = controller.get_traj_data_real_len();
        N_step = controller.get_N_step();
        
        current_trajectory = controller.get_trajectory();
        controller.set_planner_mode(use_planner);
        controller.update_mpc_weights();
        traj_len = controller.get_traj_data_real_len();

        #ifndef SIMULATION_MODE
        for (int i = 0; i < 2 * N_DOF; ++i)
        {
            state[i] = state_interfaces_[i].get_value();
        }
        #else
        const double* x0_init = controller.get_traj_x0_init(traj_select);
        for (int i = 0; i < 2 * N_DOF; ++i)
        {
            state[i] = x0_init[i];
        }

        if(use_noise)
        {
            Eigen::Map<Eigen::VectorXd> state_eig(state, 2 * N_DOF);
            x_measured << state_eig + generateNoiseVector(2*N_DOF, Ts, mean_noise_amplitude);
        }
        else
        {
            x_measured = Eigen::Map<const Eigen::VectorXd>(state, 2 * N_DOF);
        }
        #endif

        if(use_ekf && use_lowpass_filter)
        {
            ekf.update_config();
            lowpass_filter.init(x_measured.data(), omega_c_q, omega_c_dq);
            x_filtered_lowpass_ptr = lowpass_filter.getFilteredOutputPtr();
            ekf.initialize(x_measured.data());
            x_filtered_ekf_ptr = ekf.get_x_k_plus_ptr();
        }
        else if(use_ekf)
        {
            ekf.update_config();
            ekf.initialize(x_measured.data());
            x_filtered_ekf_ptr = ekf.get_x_k_plus_ptr();
            x_filtered_lowpass_ptr = x_filtered_ekf_ptr;
        }
        else if(use_lowpass_filter)
        {
            lowpass_filter.init(x_measured.data(), omega_c_q, omega_c_dq);
            x_filtered_lowpass_ptr = lowpass_filter.getFilteredOutputPtr();
            x_filtered_ekf_ptr = x_filtered_lowpass_ptr;
        }
        else
        {
            x_filtered_ekf_ptr = x_measured.data();
            x_filtered_lowpass_ptr = x_measured.data();
        }
        // without previous data, we cannot filter

        shm.write("read_traj_length", &traj_len);

        tau_full_future = std::async(std::launch::async, [this]()
                                     {
            Eigen::VectorXd tau_full_temp = controller.solveMPC(x_measured.data());
            mpc_started = true;
            first_torque_read = false;
            int8_t readonly_mode = 1;

            shm.write("readonly_mode", &readonly_mode);
            shm.write("read_state_data_full", x_measured.data(), global_traj_count);
            shm.write("read_control_data_full", tau_full.data(), global_traj_count);
            shm.write("read_traj_data_full", controller.get_act_traj_data(), global_traj_count);
            shm.write("read_frequency_full", &current_frequency, global_traj_count);
            shm.post_semaphore("shm_changed_semaphore");

            if(global_traj_count < traj_len)
                global_traj_count++;

            RCLCPP_INFO(get_node()->get_logger(), "CasAdi MPC started");
            return tau_full_temp; });

        RCLCPP_INFO(get_node()->get_logger(), "CasAdi MPC started!");
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

        #ifdef SIMULATION_MODE
        const double* x0_init = controller.get_traj_x0_init(traj_select);
        controller.reset(x0_init);
        for(int i = 0; i < 2 * N_DOF; ++i)
        {
            state[i] = x0_init[i];
            tau_full = Eigen::VectorXd::Zero(N_DOF);
        }
        #endif

        global_traj_count = 0;

        response->status = "reset flag set";
        mpc_started = false;
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
        mpc_started = false;
        RCLCPP_INFO(get_node()->get_logger(), "CasAdi MPC stopped");
    }

    void ModelPredictiveControllerCasadi::traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
                                                      std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Response> response)
    {
        traj_select = request->traj_select; // it is later used at start_mpc() in init_trajectory

        // shm_flags flags = {
        //     0, // start
        //     1, // reset
        //     0, // stop
        //     0  // torques_valid
        // };

        // shm.write("data_from_simulink_stop", &flags.stop);
        // shm.write("data_from_simulink_reset", &flags.reset);
        // shm.write("data_from_simulink_start", &flags.start);
        // shm.post_semaphore("shm_changed_semaphore");
        controller.switch_traj(traj_select);

        response->status = "trajectory " + std::to_string(traj_select) + " selected";
        mpc_started = false;
        
        RCLCPP_INFO(get_node()->get_logger(), "Trajectory %d selected", traj_select);
    }

    void ModelPredictiveControllerCasadi::mpc_switch(const std::shared_ptr<mpc_interfaces::srv::CasadiMPCTypeCommand::Request> request,
                                                         std::shared_ptr<mpc_interfaces::srv::CasadiMPCTypeCommand::Response> response)
    {
        CasadiMPCType mpc_type = static_cast<CasadiMPCType>(request->mpc_type);
        controller.setActiveMPC(mpc_type);

        response->status = "MPC type " + std::to_string(request->mpc_type) + " selected";
        std::string status_message = "Switched to Casadi " + casadi_mpctype_to_string(mpc_type);
        RCLCPP_INFO(get_node()->get_logger(), status_message.c_str());
        response->status = status_message;
    }

    nlohmann::json ModelPredictiveControllerCasadi::read_config(std::string file_path)
    {
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            std::cerr << "Error: Could not open JSON file." << std::endl;
            return {};
        }
        nlohmann::json jsonData;
        file >> jsonData; // Parse JSON file
        file.close();
        return jsonData;
    }

    #ifdef SIMULATION_MODE
    // Function to generate an Eigen vector of white noise
    Eigen::VectorXd ModelPredictiveControllerCasadi::generateNoiseVector(int n, double Ts, double mean_noise_amplitude) {
        // Calculate noise power
        double noise_power = 1 / (2*Ts) * (Ts / 2) * M_PI * std::pow(mean_noise_amplitude, 2);

        // Initializes random number generator for normal distribution
        std::random_device rd;
        std::mt19937 generator(rd()); // Mersenne Twister random number generator
        std::normal_distribution<double> distribution(0.0, std::sqrt(noise_power)); // Normal distribution

        // Create an Eigen vector to hold the noise
        Eigen::VectorXd white_noise(n);

        // Generate white noise
        for (int i = 0; i < n; ++i) {
            white_noise[i] = distribution(generator);
        }

        return white_noise; // Return the generated noise vector
    }
    #endif

} // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ModelPredictiveControllerCasadi,
                       controller_interface::ControllerInterface)