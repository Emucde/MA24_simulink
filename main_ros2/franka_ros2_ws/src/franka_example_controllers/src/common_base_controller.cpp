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

#include <franka_example_controllers/common_base_controller.hpp>

#include <exception>
#include <string>
#include <unistd.h>

#include <memory>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace franka_example_controllers
{
    controller_interface::InterfaceConfiguration
    CommonBaseController::command_interface_configuration() const
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
    CommonBaseController::state_interface_configuration() const
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

    controller_interface::return_type CommonBaseController::update(
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
                std::async(std::launch::async, &CommonBaseController::solve, this);
            
            if(solver_step_counter >= 1000)
                solver_step_counter = 0;
            solver_step_counter++;

            double current_frequency = timer_mpc_solver.get_frequency()*solver_steps;
            shm.write("read_state_data_full", x_measured.data(), global_traj_count);
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
                else if (error_flag == ErrorFlag::CROCODDYL_ERROR)
                {
                    RCLCPP_WARN(get_node()->get_logger(), "Error in Crocoddyl function call. Stopping the controller.");
                }

                controller_started = false;
                tau_full = Eigen::VectorXd::Zero(nq);
            }

            #ifdef SIMULATION_MODE
            controller.simulateModelRK4(state.data(), tau_full.data(), Ts);
            #endif
        }
        else
        {
            tau_full = Eigen::VectorXd::Zero(nq);
        }

        #ifndef SIMULATION_MODE
        // Write torques to shared memory (send data to robot)
        for (int i = 0; i < nq; ++i) {
            command_interfaces_[i].set_value(tau_full[i]);
        }
        #endif

        return controller_interface::return_type::OK;
    }

    CallbackReturn CommonBaseController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        arm_id_ = get_node()->get_parameter("arm_id").as_string();

        RCLCPP_INFO(get_node()->get_logger(), "Configuring MPC controller for %s arm", arm_id_.c_str());

        // subscription_ = get_node()->create_subscription<mpc_interfaces::msg::ControlArray>(
        //     "topic", 10, std::bind(&CommonBaseController::topic_callback, this, _1));

        // RCLCPP_INFO(get_node()->get_logger(), "Subscribed to topic 'topic'");

        std::string node_name = get_node()->get_name();
        std::string service_prefix = "/" + node_name + "/";

        start_controller_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "start_controller_service",
            std::bind(&CommonBaseController::start_controller, this, std::placeholders::_1, std::placeholders::_2));

        reset_controller_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "reset_controller_service",
            std::bind(&CommonBaseController::reset_controller, this, std::placeholders::_1, std::placeholders::_2));

        stop_controller_service_ = get_node()->create_service<mpc_interfaces::srv::SimpleCommand>(
            service_prefix + "stop_controller_service",
            std::bind(&CommonBaseController::stop_controller, this, std::placeholders::_1, std::placeholders::_2));

        traj_switch_service_ = get_node()->create_service<mpc_interfaces::srv::TrajectoryCommand>(
            service_prefix + "traj_switch_service",
            std::bind(&CommonBaseController::traj_switch, this, std::placeholders::_1, std::placeholders::_2));

        mpc_switch_service_ = get_node()->create_service<mpc_interfaces::srv::CasadiMPCTypeCommand>(
            service_prefix + "mpc_switch_service",
            std::bind(&CommonBaseController::mpc_switch, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_node()->get_logger(), "Services created");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CommonBaseController::on_init()
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
        timer_mpc_solver.tic();

        RCLCPP_INFO(get_node()->get_logger(), "MPC controller initialized");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CommonBaseController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_deactivate: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn CommonBaseController::on_activate(const rclcpp_lifecycle::State &)
    {
        open_shared_memories();
        int8_t readonly_mode = 1;
        shm.write("readonly_mode", &readonly_mode);
        RCLCPP_INFO(get_node()->get_logger(), "on_activate: Shared memory opened successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn CommonBaseController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_cleanup: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn CommonBaseController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        close_shared_memories();
        RCLCPP_INFO(get_node()->get_logger(), "on_shutdown: Shared memory closed successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void CommonBaseController::open_shared_memories()
    {
        shm.open_readwrite_shms(shm_readwrite_infos);
        shm.open_readwrite_sems(sem_readwrite_names);
        RCLCPP_INFO(get_node()->get_logger(), "Shared memory opened successfully.");
    }

    void CommonBaseController::close_shared_memories()
    {
        shm.close_shared_memories();
        shm.close_semaphores();
    }

    // void CommonBaseController::topic_callback(const mpc_interfaces::msg::ControlArray & msg)
    // {
    //     Eigen::Map<const Eigen::VectorXd> u_k(msg.control_array.data(), msg.control_array.size());
    //     RCUTILS_LOG_WARN("Received control array: [%f, %f, %f, %f, %f, %f]",
    //                     u_k[0], u_k[1], u_k[2], u_k[3], u_k[4], u_k[5]);
    // }

    void CommonBaseController::start_controller(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
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

        tau_full = controller.solveMPC(x_filtered.data());
        controller.set_traj_count(0);
        tau_full = controller.solveMPC(x_filtered.data());
        int8_t readonly_mode = 1;

        shm.write("read_traj_length", &traj_len);
        shm.write("readonly_mode", &readonly_mode);

        controller_started = true;
        controller.set_traj_count(0);
    }

    void CommonBaseController::reset_controller(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
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

        reset_controller_trajectory();

        response->status = "reset flag set";
        controller_started = false;
        shm.post_semaphore("shm_changed_semaphore");
        RCLCPP_INFO(get_node()->get_logger(), "CasAdi MPC reset");
    }

    void CommonBaseController::stop_controller(const std::shared_ptr<mpc_interfaces::srv::SimpleCommand::Request>,
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

    void CommonBaseController::traj_switch(const std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Request> request,
                                                      std::shared_ptr<mpc_interfaces::srv::TrajectoryCommand::Response> response)
    {
        traj_select = request->traj_select; // it is later used at start_controller() in init_trajectory

        controller.switch_traj(traj_select);
        reset_controller_trajectory();

        response->status = "trajectory " + std::to_string(traj_select) + " selected";
        controller_started = false;
        
        RCLCPP_INFO(get_node()->get_logger(), "Trajectory %d selected", traj_select);
    }

    void CommonBaseController::mpc_switch(const std::shared_ptr<mpc_interfaces::srv::CasadiMPCTypeCommand::Request> request,
                                                         std::shared_ptr<mpc_interfaces::srv::CasadiMPCTypeCommand::Response> response)
    {
        CasadiMPCType mpc_type = static_cast<CasadiMPCType>(request->mpc_type);
        controller.setActiveMPC(mpc_type);
        reset_controller_trajectory();

        response->status = "MPC type " + std::to_string(request->mpc_type) + " selected";
        std::string status_message = "Switched to Casadi " + casadi_mpctype_to_string(mpc_type);
        RCLCPP_INFO(get_node()->get_logger(), status_message.c_str());
        response->status = status_message;
    }

    nlohmann::json CommonBaseController::read_config(std::string file_path)
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

    Eigen::VectorXd CommonBaseController::filter_x_measured()
    {
        Eigen::VectorXd x_filtered;
        if(use_ekf && use_lowpass_filter)
        {
            ekf.predict(tau_full.data(), x_measured.data());
            lowpass_filter.run(x_filtered_ekf_ptr);
            x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_lowpass_ptr, nx);
        }
        else if(use_lowpass_filter)
        {
            lowpass_filter.run(x_measured.data()); // updates data from x_filtered_ptr
            x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_lowpass_ptr, nx);
        }
        else if(use_ekf)
        {
            ekf.predict(tau_full.data(), x_measured.data());
            x_filtered_ekf_ptr = ekf.get_x_k_plus_ptr();
            x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_ekf_ptr, nx);
        }
        else
        {
            x_filtered = Eigen::Map<Eigen::VectorXd>(x_measured.data(), nx);
        }
        return x_filtered;
    }

    #ifdef SIMULATION_MODE
    // Function to generate an Eigen vector of white noise
    Eigen::VectorXd CommonBaseController::generateNoiseVector(int n, double Ts, double mean_noise_amplitude) {
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

    void CommonBaseController::solve()
    {
        timer_mpc_solver.tic();
        tau_full = controller.solveMPC(x_filtered.data());
        timer_mpc_solver.toc();
    }

    void CommonBaseController::init_filter(double omega_c_q, double omega_c_dq)
    {
        if(use_ekf && use_lowpass_filter)
        {
            ekf.update_config();
            lowpass_filter.init(x_measured.data(), omega_c_q, omega_c_dq);
            x_filtered_lowpass_ptr = lowpass_filter.getFilteredOutputPtr();
            ekf.initialize(x_measured.data());
            x_filtered_ekf_ptr = ekf.get_x_k_plus_ptr();
            x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_ekf_ptr, nx);
        }
        else if(use_ekf)
        {
            ekf.update_config();
            ekf.initialize(x_measured.data());
            x_filtered_ekf_ptr = ekf.get_x_k_plus_ptr();
            x_filtered_lowpass_ptr = x_filtered_ekf_ptr;
            x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_ekf_ptr, nx);
        }
        else if(use_lowpass_filter)
        {
            lowpass_filter.init(x_measured.data(), omega_c_q, omega_c_dq);
            x_filtered_lowpass_ptr = lowpass_filter.getFilteredOutputPtr();
            x_filtered_ekf_ptr = x_filtered_lowpass_ptr;
            x_filtered = Eigen::Map<Eigen::VectorXd>(x_filtered_lowpass_ptr, nx);
        }
        else
        {
            x_filtered_ekf_ptr = x_measured.data();
            x_filtered_lowpass_ptr = x_measured.data();
            x_filtered = Eigen::Map<Eigen::VectorXd>(x_measured.data(), nx);
        }
        // without previous data, we cannot filter
    }

    void CommonBaseController::init_controller()
    {
        // Update general configuration
        nlohmann::json general_config = read_config(general_config_filename);
        use_lowpass_filter = general_config["use_lowpass_filter"];
        use_ekf = general_config["use_ekf"];

        double omega_c_q = general_config["lowpass_filter_omega_c_q"];
        double omega_c_dq = general_config["lowpass_filter_omega_c_dq"];

        //////// TORQUE MAPPER CONFIG ////////
        auto torque_mapper_settings = general_config["torque_mapper_settings"];
        FullSystemTorqueMapper::Config torque_mapper_config;
        torque_mapper_config.K_d = Eigen::VectorXd::Map(torque_mapper_settings["K_d"].get<std::vector<double>>().data(), robot_config.nq).asDiagonal();
        torque_mapper_config.D_d = Eigen::VectorXd::Map(torque_mapper_settings["D_d"].get<std::vector<double>>().data(), robot_config.nq).asDiagonal();
        torque_mapper_config.K_d_fixed = Eigen::VectorXd::Map(torque_mapper_settings["K_d_fixed"].get<std::vector<double>>().data(), robot_config.nq_fixed).asDiagonal();
        torque_mapper_config.D_d_fixed = Eigen::VectorXd::Map(torque_mapper_settings["D_d_fixed"].get<std::vector<double>>().data(), robot_config.nq_fixed).asDiagonal();
        torque_mapper_config.q_ref_nq = Eigen::VectorXd::Map(torque_mapper_settings["q_ref_nq"].get<std::vector<double>>().data(), robot_config.nq);
        torque_mapper_config.q_ref_nq_fixed = Eigen::VectorXd::Map(torque_mapper_settings["q_ref_nq_fixed"].get<std::vector<double>>().data(), robot_config.nq_fixed);
        torque_mapper_config.torque_limit = torque_mapper_settings["torque_limit"];
        controller.set_torque_mapper_config(torque_mapper_config);

        /////// INIT FILE TRAJECTORY ///////
        double T_traj_start = general_config["transient_traj_start_time"];
        double T_traj_dur = general_config["transient_traj_duration"];
        double T_traj_end = general_config["transient_traj_end_time"];
        controller.init_file_trajectory(traj_select, state.data(), T_traj_start, T_traj_dur, T_traj_end);
        traj_len = controller.get_traj_data_real_len();

        solver_steps = controller.get_traj_step();
        current_trajectory = controller.get_trajectory();
        traj_len = controller.get_traj_data_real_len();

        #ifdef SIMULATION_MODE
        mean_noise_amplitude = general_config["mean_noise_amplitude"];
        use_noise = general_config["use_noise"];
        if(use_noise)
            x_measured = state + generateNoiseVector(nx, Ts, mean_noise_amplitude);
        else
            x_measured = state;
        #endif

        controller.custom_init(general_config);

        init_filter(omega_c_q, omega_c_dq); // uses x_measured
    }

    void CommonBaseController::reset_controller_trajectory()
    {
        Eigen::VectorXd x0_red_init = controller.get_traj_x0_red_init(traj_select);
        controller.reset(x0_red_init.data());

        #ifdef SIMULATION_MODE
        const double *x0_init = controller.get_traj_x0_init(traj_select);
        state = Eigen::Map<const Eigen::VectorXd>(x0_init, nx);
        #endif

        tau_full = Eigen::VectorXd::Zero(nq);
        global_traj_count = 0;
    }

} // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CommonBaseController,
                       controller_interface::ControllerInterface)