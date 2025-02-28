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

    void CommonROSBaseController::filter_x_measured()
    {
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
    }

    // Function to generate an Eigen vector of white noise
    Eigen::VectorXd CommonROSBaseController::generateNoiseVector(int n, double Ts, double mean_noise_amplitude) {
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
        if(use_noise)
            x_measured = state + generateNoiseVector(nx, Ts, mean_noise_amplitude);
        else
            x_measured = state;
        #endif

        init_filter(); // uses x_measured
    }

    void CommonROSBaseController::init_filter()
    {
        if(use_ekf && use_lowpass_filter)
        {
            ekf.update_config();
            lowpass_filter.update_config();
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
            lowpass_filter.update_config();
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

    void CommonROSBaseController::init_trajectory()
    {
        nlohmann::json general_config = read_config(general_config_filename);
        double T_traj_start = get_config_value<double>(general_config, "transient_traj_start_time");
        double T_traj_dur = get_config_value<double>(general_config, "transient_traj_duration");
        double T_traj_end = get_config_value<double>(general_config, "transient_traj_end_time");
        base_controller->init_file_trajectory(traj_select, x_filtered.data(), T_traj_start, T_traj_dur, T_traj_end);
        traj_len = base_controller->get_traj_data_real_len();
        current_trajectory = base_controller->get_trajectory();
    }

    void CommonROSBaseController::reset_trajectory()
    {
        Eigen::VectorXd x0_red_init = base_controller->get_transient_traj_x0_red_init();
        base_controller->reset(x0_red_init.data());

        #ifdef SIMULATION_MODE
        Eigen::VectorXd x0_init = base_controller->get_file_traj_x0_nq_init(traj_select);
        state = x0_init;
        #endif

        tau_full = Eigen::VectorXd::Zero(nq);
        global_traj_count = 0;
        first_start=true;
    }
} // namespace franka_example_controllers