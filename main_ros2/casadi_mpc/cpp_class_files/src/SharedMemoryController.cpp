#include "SharedMemoryController.hpp"

SharedMemoryController::SharedMemoryController(const std::string &masterDir, bool use_custom_list)
    : SharedMemoryController(
        masterDir + "/urdf_creation/fr3_no_hand_7dof.urdf",
        masterDir + "/utils_python/mpc_weights_crocoddyl.json",
        masterDir + "/config_settings/ekf_settings.json",
        masterDir + "/config_settings/general_settings.json",
        masterDir + (use_custom_list ? "/config_settings/casadi_mpc_weights_fr3_no_hand_custom_list.json" : "/config_settings/casadi_mpc_weights_fr3_no_hand_simulink.json")
    ) {}

// Constructor using specific file names
SharedMemoryController::SharedMemoryController(const std::string& urdf_file, 
                        const std::string& crocoddyl_config_file, 
                        const std::string& ekf_config_file, 
                        const std::string& general_config_file, 
                        const std::string& casadi_mpc_config_file)
    : urdf_filename(urdf_file),
        crocoddyl_config_filename(crocoddyl_config_file),
        ekf_config_filename(ekf_config_file),
        general_config_filename(general_config_file),
        casadi_mpc_config_filename(casadi_mpc_config_file),
        robot_config(get_robot_config()),
        nq(robot_config.nq),
        nx(robot_config.nx),
        nq_red(robot_config.nq_red),
        nx_red(robot_config.nx_red),
        n_indices(ConstIntVectorMap(robot_config.n_indices, nq_red)),
        n_x_indices(ConstIntVectorMap(robot_config.n_x_indices, nx_red)),
        classic_controller(urdf_filename, general_config_filename),
        casadi_controller(urdf_filename, casadi_mpc_config_filename, general_config_filename),
        crocoddyl_controller(urdf_filename, crocoddyl_config_filename, general_config_filename),
        controller(&crocoddyl_controller),
        x0_nq_init(controller->get_transient_traj_x0_init()),
        x_measured(controller->get_transient_traj_x0_init()),
        tau_full(Eigen::VectorXd::Zero(nq)),
        lowpass_filter(nq, x_measured.data(), general_config_filename),
        ekf(ekf_config_filename),
        error_flag(ErrorFlag::NO_ERROR),
        base_ekf_filter(ekf, x_measured.data(), tau_full.data()),
        base_lowpass_filter(lowpass_filter, x_measured.data()),
        base_lpekf_filter(ekf, lowpass_filter, x_measured.data(), tau_full.data()),
        base_no_filter(x_measured.data()),
        base_filter(&base_no_filter)
{
    update_config();
    init_filter(x_measured);
    init_trajectory(x_measured);
    init_shm();
}

// Destructor
SharedMemoryController::~SharedMemoryController()
{
    shm.close_shared_memories();
    shm.close_semaphores();
}

SharedMemoryController::MainControllerType SharedMemoryController::get_controller_type(const std::string &controller_type_str)
{
    if (controller_type_str == "Classic")
        return MainControllerType::Classic;
    else if (controller_type_str == "Casadi")
        return MainControllerType::Casadi;
    else if (controller_type_str == "Crocoddyl")
        return MainControllerType::Crocoddyl;
    else
    {
        throw std::invalid_argument("Invalid controller type");
        return MainControllerType::COUNT;
    }
}

CrocoddylMPCType SharedMemoryController::get_crocoddyl_controller_type(const std::string &controller_type_str)
{
    if (controller_type_str == "DynMPC_v1")
        return CrocoddylMPCType::DynMPC_v1;
    // else if (controller_type_str == "DynMPC_v2")
    //     return CrocoddylMPCType::DynMPC_v2;
    // else if (controller_type_str == "DynMPC_v3")
    //     return CrocoddylMPCType::DynMPC_v3;
    else
    {
        throw std::invalid_argument("Invalid controller type");
        return CrocoddylMPCType::DynMPC_v1;
    }
}

void SharedMemoryController::update_config()
{
    nlohmann::json general_config = read_config<>(general_config_filename);

    // Robot Settings
    std::string tcp_frame_name = get_config_value<std::string>(general_config, "tcp_frame_name");

    // Simulation Settings
    use_noise = get_config_value<bool>(general_config, "use_noise");
    mean_noise_amplitude = get_config_value<double>(general_config, "mean_noise_amplitude");

    // General Settings
    Ts = get_config_value<double>(general_config, "dt");
    current_frequency = 0.0;
    
    // Trajectory Settings
    trajectory_selection = get_config_value<double>(general_config, "trajectory_selection"); // TODO
    T_traj_start     = get_config_value<double>(general_config, "transient_traj_start_time");
    T_traj_dur   = get_config_value<double>(general_config, "transient_traj_duration");
    T_traj_end   = get_config_value<double>(general_config, "transient_traj_end_time");

    // Filter Settings
    use_lowpass_filter = get_config_value<bool>(general_config, "use_lowpass_filter");
    use_ekf = get_config_value<bool>(general_config, "use_ekf");
    omega_c_q    = get_config_value<double>(general_config, "lowpass_filter_omega_c_q");
    omega_c_dq   = get_config_value<double>(general_config, "lowpass_filter_omega_c_dq");

    casadi_controller.setActiveMPC(string_to_casadi_mpctype(get_config_value<std::string>(general_config, "default_casadi_mpc")));
    casadi_controller.update_mpc_weights();
    classic_controller.switchController(classic_controller.get_classic_controller_type(get_config_value<std::string>(general_config, "default_classic_controller")));
    crocoddyl_controller.setActiveMPC(get_crocoddyl_controller_type(get_config_value<std::string>(general_config, "default_crocoddyl_mpc")));

    active_controller_type = get_controller_type(get_config_value<std::string>(general_config, "default_controller"));

    if (active_controller_type == MainControllerType::Casadi)
        controller = &casadi_controller;
    else if(active_controller_type == MainControllerType::Classic)
        controller = &classic_controller;
    else if(active_controller_type == MainControllerType::Crocoddyl)
        controller = &crocoddyl_controller;
    else
    {
        throw std::invalid_argument("Invalid controller type");
    }
}

void SharedMemoryController::init_filter(Eigen::VectorXd &x_nq)
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
}

void SharedMemoryController::init_trajectory(Eigen::VectorXd &x_nq)
{
    x0_nq_init = x_nq;
    x0_nq_red_init = x_nq(n_x_indices);

    controller->init_file_trajectory(trajectory_selection, x0_nq_init.data(), T_traj_start, T_traj_dur, T_traj_end);
    transient_traj_len = controller->get_transient_traj_len();

    traj_step = controller->get_traj_step();
    traj_len = controller->get_traj_data_real_len();
    act_data = controller->get_act_traj_data();
    traj_count = 0;
}

void SharedMemoryController::init_shm()
{
    shm.open_readwrite_shms(shm_readwrite_infos);
    shm.open_readwrite_sems(sem_readwrite_names);

    start_cpp = 0;
    stop_cpp = 0;
    reset_cpp = 0;
    valid_cpp = 0;
    error_flag_int8 = 0;
    run_flag = false;
    first_run = true;
    ros2_semaphore = shm.get_semaphore("ros2_semaphore");
    valid_cpp_shm = shm.get_shared_memory("valid_cpp");

    reset_cpp_shm_flags();
    shm.feedback_write_int8("error_cpp", &error_flag_int8);
    shm.feedback_write_int8("valid_cpp", &valid_cpp);
    shm.write("read_traj_length", &traj_len);
    shm.write("data_from_simulink_start", &start_cpp);
    shm.write("data_from_simulink_reset", &reset_cpp);
    shm.write("data_from_simulink_stop", &stop_cpp);

    //reset semaphore counter
    while (sem_trywait(ros2_semaphore) == 0) {
        // Keep decrementing until it fails
    }
}

// Function for shared memory in python
void SharedMemoryController::init_python()
{
    int8_t readonly_mode = 1, start = 1, reset = 0, stop = 0;
    shm.write("readonly_mode", &readonly_mode);
    shm.write("read_traj_length", &traj_len);
    shm.write("data_from_simulink_start", &start);
    shm.write("data_from_simulink_reset", &stop);
    shm.write("data_from_simulink_stop", &reset);
}

// Function to generate an Eigen vector of white noise
Eigen::VectorXd SharedMemoryController::generateNoiseVector(int n, double Ts, double mean_noise_amplitude) {
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

void SharedMemoryController::reset_cpp_shm_flags()
{
    start_cpp = 0;
    reset_cpp = 0;
    stop_cpp = 0;
    shm.feedback_write_int8("start_cpp", &start_cpp);
    shm.feedback_write_int8("reset_cpp", &reset_cpp);
    shm.feedback_write_int8("stop_cpp", &stop_cpp);
}

void SharedMemoryController::run_simulation()
{
    casadi_real x_k_ndof[nx] = {0};
    const casadi_real *x0_init = controller->get_traj_x0_init(trajectory_selection);

    Eigen::Map<Eigen::VectorXd> q_k_ndof_eig(x_k_ndof, nq);
    Eigen::Map<Eigen::VectorXd> x_k_ndof_eig(x_k_ndof, nx);
    x_k_ndof_eig = Eigen::Map<const Eigen::VectorXd>(x0_init, nx);
    Eigen::Map<Eigen::VectorXd> x_filtered(base_filter->get_filtered_data_ptr(), nx);

    init_python();

    // Start measuring time
    timer_total.tic();

    // Main loop for trajectory processing
    for (casadi_uint i = 0; i < traj_len; i=i+traj_step)
    {
        if(use_noise)
            x_measured << x_k_ndof_eig + generateNoiseVector(nx, Ts, mean_noise_amplitude);
        else
            x_measured << x_k_ndof_eig;

        base_filter->run_filter(); // automatically mapped to x_filtered

        timer_mpc_solver.tic();
        tau_full << controller->update_control(x_filtered);
        timer_mpc_solver.toc();
        act_data = controller->get_act_traj_data();
        error_flag = controller->get_error_flag();

        if (i % 100 == 0)
        {
            timer_mpc_solver.print_frequency("i=" + std::to_string(i));
            std::cout << std::endl;
            std::cout << "q_k: |" << q_k_ndof_eig.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "|", "|")) << "|" << std::endl;
            std::cout << "u_k: |" << tau_full.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "|", "|")) << "|"<< std::endl;
        }
        if (i == transient_traj_len)
        {
            std::cout << "Switching to trajectory from data" << std::endl;
        }

        for (casadi_uint j = 0; j < traj_step; j++)
        {
            // Write data to shm:
            current_frequency = timer_mpc_solver.get_frequency()*traj_step;
            shm.write("read_state_data_full", x_filtered.data(), i+j);
            shm.write("read_control_data_full", tau_full.data(), i+j);
            shm.write("read_control_data", tau_full.data());
            shm.write("read_traj_data_full", act_data, i+j);
            shm.write("read_frequency_full", &current_frequency, i+j);
            shm.post_semaphore("shm_changed_semaphore");

            controller->simulateModelRK4(x_k_ndof, tau_full.data(), Ts);
        }

        if (error_flag != ErrorFlag::NO_ERROR)
        {
            std::cerr << "Error flag: " << static_cast<int>(error_flag) << std::endl;
            break;
        }
    }

    // Measure and print execution time
    timer_mpc_solver.print_time("Total casadi_controller.solveMPC time: ");
    std::cout << std::endl;

    timer_total.toc();
    timer_total.print_time("Total execution time: ");
    std::cout << std::endl;

    int8_t start = 1;
    shm.write("data_from_simulink_reset", &start);
    shm.post_semaphore("shm_changed_semaphore");
    shm.close_shared_memories();
    shm.close_semaphores();
}

void SharedMemoryController::run_shm_mode()
{
    while(true)
    {
        Eigen::Map<Eigen::VectorXd> q_k_ndof_eig(x_measured.data(), nq);
        double* x_filtered_ptr = base_filter->get_filtered_data_ptr();
        Eigen::Map<Eigen::VectorXd> x_filtered(x_filtered_ptr, nx);
        int print_counter = 0;

        //reset semaphore counter
        while (sem_trywait(ros2_semaphore) == 0) {
            // Keep decrementing until it fails
        }

        sem_wait(ros2_semaphore);

        shm.read_int8("start_cpp", &start_cpp);
        shm.read_int8("stop_cpp", &stop_cpp);
        shm.read_int8("reset_cpp", &reset_cpp);

        if(reset_cpp == 1)
        {
            std::cout << "Resetting controller" << std::endl;
            reset_cpp = 0;
            run_flag=false;
            shm.write("reset_cpp", &reset_cpp);
        }
        if(start_cpp == 1 && reset_cpp == 0 && stop_cpp == 0)
        {
            start_cpp = 0;
            shm.write("start_cpp", &start_cpp);
            init_python();
            update_config();
            shm.read_double("ros2_state_data", x_measured.data());
            init_filter(x_measured);
            x_measured_red = x_measured(n_x_indices);
            init_trajectory(x_measured);
            controller->reset(x_measured_red.data());
            tau_full = controller->update_control(x_measured);
            controller->set_traj_count(0);
            
            std::cout << "Starting controller" << std::endl;
            run_flag = true;
            while (sem_trywait(ros2_semaphore) == 0) {
                // Keep decrementing until it fails
            }
        }
        valid_cpp = 1;
        if(run_flag)
        {
            do
            {
                timer_mpc_solver.tic();
                shm.read_double("ros2_state_data", x_measured.data());
                controller->simulateModelRK4(x_measured.data(), tau_full.data(), Ts);
                
                base_filter->run_filter(); // automatically mapped to x_filtered
                act_data=controller->get_act_traj_data();
                
                if(traj_count % traj_step == 0)
                {
                    tau_full = controller->update_control(x_filtered);
                    timer_mpc_solver.toc();
                    error_flag = controller->get_error_flag();
                }

                if (print_counter % 100 == 0)
                {
                    timer_mpc_solver.print_frequency("traj_count=" + std::to_string(traj_count));
                    std::cout << std::endl;
                    std::cout << "q_k: |" << q_k_ndof_eig.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "|", "|")) << "|" << std::endl;
                    std::cout << "u_k: |" << tau_full.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "|", "|")) << "|"<< std::endl;
                    print_counter = 1;
                }
                else
                {
                    print_counter++;
                }

                if (error_flag != ErrorFlag::NO_ERROR)
                {
                    std::cerr << "Error flag: " << static_cast<int>(error_flag) << std::endl;
                    std::cout << "q_k: |" << q_k_ndof_eig.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "|", "|")) << "|" << std::endl;
                    run_flag = false;
                    valid_cpp = 0;
                }
                else
                {
                    valid_cpp = 1;
                }

                for (casadi_uint j = 0; j < traj_step; j++)
                {
                    // Write data to shm:
                    current_frequency = timer_mpc_solver.get_frequency()*traj_step;
                    shm.write("read_state_data_full", x_filtered.data(), traj_count+j);
                    shm.write("read_control_data_full", tau_full.data(), traj_count+j);
                    shm.write("read_control_data", tau_full.data());
                    shm.write("read_traj_data_full", act_data, traj_count+j);
                    shm.write("read_frequency_full", &current_frequency, traj_count+j);
                    shm.post_semaphore("shm_changed_semaphore");
                }

                shm.write("cpp_control_data", tau_full.data());
                error_flag_int8 = static_cast<int8_t>(error_flag);
                shm.write("error_cpp", &error_flag_int8);

                shm.read_int8("start_cpp", &start_cpp);
                shm.read_int8("reset_cpp", &reset_cpp);
                shm.read_int8("stop_cpp", &stop_cpp);
                if(reset_cpp == 1)
                {
                    reset_cpp = 0;
                    valid_cpp = 0;
                    shm.write("reset_cpp", &reset_cpp);
                    std::cout << "Resetting controller in do while" << std::endl;
                    run_flag = false;
                }
                else if(stop_cpp == 1)
                {
                    stop_cpp = 0;
                    valid_cpp = 0;
                    shm.write("stop_cpp", &stop_cpp);
                    std::cout << "Stopping controller in do while" << std::endl;
                }
                else if(start_cpp == 1)
                {
                    init_python();
                    start_cpp = 0;
                    shm.write("start_cpp", &start_cpp);
                    std::cout << "Starting controller in do while" << std::endl;
                }
                if(traj_count < traj_len-1)
                    traj_count += traj_step;

                write_valid_cpp_shm(valid_cpp);
                

                sem_wait(ros2_semaphore);
            } while(run_flag);
            run_flag = false;
            valid_cpp = 0;
            shm.feedback_write_int8("valid_cpp", &valid_cpp);
        }
    }
}