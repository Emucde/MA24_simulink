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

#include <franka_example_controllers/common_ros_base_controller.hpp>

#include <exception>
#include <string>
#include <unistd.h>

#include <memory>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace franka_example_controllers
{

    nlohmann::json CommonROSBaseController::read_config(std::string file_path)
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

    // Function to generate an Eigen vector of white noise
    Eigen::VectorXd CommonROSBaseController::generateNoiseVector(int n, double Ts, double mean_noise_amplitude)
    {
        // Calculate noise power
        double noise_power = 1 / (2 * Ts) * (Ts / 2) * M_PI * std::pow(mean_noise_amplitude, 2);

        // Initializes random number generator for normal distribution
        std::random_device rd;
        std::mt19937 generator(rd());                                               // Mersenne Twister random number generator
        std::normal_distribution<double> distribution(0.0, std::sqrt(noise_power)); // Normal distribution

        // Create an Eigen vector to hold the noise
        Eigen::VectorXd white_noise(n);

        // Generate white noise
        for (int i = 0; i < n; ++i)
        {
            white_noise[i] = distribution(generator);
        }

        return white_noise; // Return the generated noise vector
    }

    void CommonROSBaseController::init_controller()
    {
        // Update general configuration
        nlohmann::json general_config = read_config(general_config_filename);
        use_lowpass_filter = get_config_value<bool>(general_config, "use_lowpass_filter");
        use_ekf = get_config_value<bool>(general_config, "use_ekf");
        // Is set by using the nodejs gui
        // traj_select = general_config["trajectory_selection"];

        base_controller->update_config();

        N_step = base_controller->get_N_step();
        solver_steps = base_controller->get_traj_step();

#ifdef SIMULATION_MODE
        mean_noise_amplitude = get_config_value<double>(general_config, "mean_noise_amplitude");
        use_noise = get_config_value<bool>(general_config, "use_noise");
        if (use_noise)
            x_measured << state + generateNoiseVector(nx, Ts, mean_noise_amplitude);
        else
            x_measured << state;
#else
        for (int i = 0; i < nx; ++i)
            state[i] = state_interfaces_[i].get_value();
        x_measured << state;
#endif
        init_filter(x_measured);
    }

    void CommonROSBaseController::init_filter(Eigen::VectorXd &x_nq)
    {
        lowpass_filter.update_config();
        lowpass_filter.reset_state(x_nq.data());
        ekf.update_config();
        ekf.initialize(x_nq.data());

        if(use_ekf && use_lowpass_filter)
        {
            base_filter = &base_lpekf_filter;
        }
        else if(use_ekf)
        {
            base_filter = &base_ekf_filter;
        }
        else if(use_lowpass_filter)
        {
            base_filter = &base_lowpass_filter;
        }
        else
        {
            base_filter = &base_no_filter;
        }
        x_filtered_ptr = base_filter->get_filtered_data_ptr();
    }

    void CommonROSBaseController::init_trajectory()
    {
        nlohmann::json general_config = read_config(general_config_filename);
        double T_traj_start = get_config_value<double>(general_config, "transient_traj_start_time");
        double T_traj_dur = get_config_value<double>(general_config, "transient_traj_duration");
        double T_traj_end = get_config_value<double>(general_config, "transient_traj_end_time");
        base_controller->init_file_trajectory(traj_select, x_measured.data(), T_traj_start, T_traj_dur, T_traj_end);
        traj_len = base_controller->get_traj_data_real_len();
        current_trajectory = base_controller->get_trajectory();
    }

    void CommonROSBaseController::reset()
    {
#ifdef SIMULATION_MODE
        Eigen::VectorXd x0_init = base_controller->get_file_traj_x0_nq_init(traj_select);
        Eigen::VectorXd x0_red_init = base_controller->get_file_traj_x0_nq_red_init(traj_select);
        state = x0_init;
        x_measured << state;
#else
        Eigen::VectorXd x0_red_init = base_controller->get_transient_traj_x0_red_init();
#endif

        base_controller->reset(x0_red_init.data());

        tau_full << Eigen::VectorXd::Zero(nq);
        global_traj_count = 0;
        first_start = true;
    }

    void CommonROSBaseController::solve()
    {
        timer_solver.tic();
        tau_full << base_controller->update_control(Eigen::Map<Eigen::VectorXd>(x_filtered_ptr, nx));
        timer_solver.toc();
    }

    controller_interface::return_type CommonROSBaseController::update(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {

#ifndef SIMULATION_MODE
        // Read states from joint state interface
        for (int i = 0; i < nx; ++i)
        {
            state[i] = state_interfaces_[i].get_value();
        }
        x_measured << state;
#else
        base_controller->simulateModelRK4(state.data(), tau_full.data(), Ts);
        if (use_noise)
            x_measured << state + generateNoiseVector(nx, Ts, mean_noise_amplitude);
        else
            x_measured << state;
#endif

        base_filter->run_filter(); // automatically mapped to x_filtered_ptr, uses x_measured

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

            base_controller->simulateModelRK4(state.data(), tau_full.data(), Ts);
            if (solver_step_counter % solver_steps == 0)
            {
                solve();
                // std::async(std::launch::async, &CommonROSBaseController::solve, this);
                // std::thread solver_thread([this]() {
                //     solve();
                // });
                // solver_thread.detach();
            }

            if (solver_step_counter >= 1000)
                solver_step_counter = 0;
            solver_step_counter++;

            current_frequency = timer_solver.get_frequency() * solver_steps;
            shm.write("read_state_data_full", x_measured.data(), global_traj_count);
            shm.write("read_control_data", tau_full.data());
            shm.write("read_control_data_full", tau_full.data(), global_traj_count);
            shm.write("read_traj_data_full", current_trajectory->col(global_traj_count).data(), global_traj_count);
            shm.write("read_frequency_full", &current_frequency, global_traj_count);
            shm.post_semaphore("shm_changed_semaphore");

            if (global_traj_count < traj_len)
                global_traj_count++;

            error_flag = base_controller->get_error_flag(); // Get the error flag

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
                tau_full << Eigen::VectorXd::Zero(nq);
            }
#ifndef SIMULATION_MODE
            for (int i = 0; i < nq; ++i)
            {
                command_interfaces_[i].set_value(tau_full[i]);
            }
#endif
        }
        else
        {
#ifndef SIMULATION_MODE
            for (int i = 0; i < nq; ++i)
            {
                command_interfaces_[i].set_value(0);
            }
            tau_full << Eigen::VectorXd::Zero(nq);
#endif
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::InterfaceConfiguration
    CommonROSBaseController::command_interface_configuration() const
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
    CommonROSBaseController::state_interface_configuration() const
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

    CallbackReturn CommonROSBaseController::on_init()
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

    CallbackReturn CommonROSBaseController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_deactivate: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn CommonROSBaseController::on_activate(const rclcpp_lifecycle::State &)
    {
        open_shared_memories();

        tau_full << Eigen::VectorXd::Zero(nq);
        controller_started = false;
        init_controller();
        init_trajectory();
        reset();

        int8_t readonly_mode = 1;
        shm.write("readonly_mode", &readonly_mode);
        RCLCPP_INFO(get_node()->get_logger(), "on_activate: Shared memory opened successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn CommonROSBaseController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_cleanup: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn CommonROSBaseController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_shutdown: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void CommonROSBaseController::open_shared_memories()
    {
        shm.open_readwrite_shms(shm_readwrite_infos);
        shm.open_readwrite_sems(sem_readwrite_names);
        RCLCPP_INFO(get_node()->get_logger(), "Shared memory opened successfully.");
    }

    void CommonROSBaseController::close_shared_memories()
    {
        shm.close_shared_memories();
        shm.close_semaphores();
    }

    void CommonROSBaseController::start_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
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

        init_controller();
        init_trajectory();
        reset();

        // base_filter->run_filter(); // automatically mapped to x_filtered_ptr, uses x_measured
        // std::async(std::launch::async, [this]() {
            solve(); // cold start init guess
            tau_full << Eigen::VectorXd::Zero(nq);
            base_controller->set_traj_count(0);

            int8_t readonly_mode = 1;
            shm.write("read_traj_length", &traj_len);
            shm.write("readonly_mode", &readonly_mode);

            controller_started = true;
        // });
    }

    void CommonROSBaseController::reset_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
                                                    std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        tau_full << Eigen::VectorXd::Zero(nq);
        controller_started = false;

        shm_flags flags = {
            0, // start
            1, // reset
            0, // stop
            0  // torques_valid
        };

        shm.write("data_from_simulink_start", &flags.start);
        shm.write("data_from_simulink_stop", &flags.stop);
        shm.write("data_from_simulink_reset", &flags.reset);

        reset();

        response->status = "reset flag set";
        shm.post_semaphore("shm_changed_semaphore");
        RCLCPP_INFO(get_node()->get_logger(), "CasAdi MPC reset");
    }

    void CommonROSBaseController::stop_mpc(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
                                                   std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Response> response)
    {
        tau_full << Eigen::VectorXd::Zero(nq);
        controller_started = false;

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
        RCLCPP_INFO(get_node()->get_logger(), "CasAdi MPC stopped");
    }

    void CommonROSBaseController::traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
                                                      std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Response> response)
    {
        int old_traj_select = traj_select;
        traj_select = request->traj_select; // it is later used at start_mpc() in init_trajectory
        if (old_traj_select == traj_select)
        {
            response->status = "trajectory " + std::to_string(traj_select) + " already selected";
            RCLCPP_INFO(get_node()->get_logger(), "Trajectory %d already selected", traj_select);
        }
        else
        {
            base_controller->switch_traj(traj_select);
            reset();

            response->status = "trajectory " + std::to_string(traj_select) + " selected";
            controller_started = false;
            first_start = true;

            RCLCPP_INFO(get_node()->get_logger(), "Trajectory %d selected", traj_select);
        }
    }

} // namespace franka_example_controllers