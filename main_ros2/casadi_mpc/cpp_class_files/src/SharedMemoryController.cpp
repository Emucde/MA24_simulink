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
        x_measured(x0_nq_init),
        x_filtered(x0_nq_init),
        x_filtered_ekf_ptr(x_measured.data()), x_filtered_lowpass_ptr(x_measured.data()),
        lowpass_filter(nq, robot_config.dt, x_measured.data(), 400.0, 400.0),
        ekf(ekf_config_filename), 
        error_flag(ErrorFlag::NO_ERROR) 
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
    traj_step = 1;
    current_frequency = 0.0;
    
    // Trajectory Settings
    traj_len = 0;
    trajectory_selection = get_config_value<double>(general_config, "trajectory_selection");
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
    CasadiEKF ekf(ekf_config_filename);
    ekf.initialize(x_nq.data());
    x_ekf_filtered_ptr = ekf.get_x_k_plus_ptr();

    SignalFilter lowpass_filter(nq, Ts, x_nq.data(), omega_c_q, omega_c_dq); // int num_joints, double Ts, double *state, double omega_c_q, double omega_c_dq
    x_lowpass_filtered_ptr = lowpass_filter.getFilteredOutputPtr();
}

void SharedMemoryController::init_trajectory(Eigen::VectorXd &x_nq)
{
    x0_nq_init = x_nq;
    x0_nq_red_init = x_nq(n_indices);

    traj_step = controller->get_traj_step();
    traj_len = controller->get_traj_data_real_len();
    act_data = controller->get_act_traj_data();

    controller->init_file_trajectory(trajectory_selection, x0_nq_init.data(), T_traj_start, T_traj_dur, T_traj_end);
    transient_traj_len = controller->get_transient_traj_len();
    traj_count = 0;
}

void SharedMemoryController::init_shm()
{
    shm.open_readwrite_shms(shm_readwrite_infos);
    shm.open_readwrite_sems(sem_readwrite_names);

    start_cpp = 0;
    stop_cpp = 0;
    reset_cpp = 0;
    valid_cpp = 1;
    error_flag_int8 = 0;
    run_flag = false;
    ros2_semaphore = shm.get_semaphore("ros2_semaphore");
}

// Function for shared memory in python
void SharedMemoryController::init_python()
{
    int8_t readonly_mode = 1, start = 1;
    shm.write("readonly_mode", &readonly_mode);
    shm.write("read_traj_length", &traj_len);
    shm.write("data_from_simulink_start", &start);
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
        shm.write("start_cpp", &start_cpp, sizeof(int8_t));
        shm.write("reset_cpp", &reset_cpp, sizeof(int8_t));
        shm.write("stop_cpp", &stop_cpp, sizeof(int8_t));
}

void SharedMemoryController::run_simulation()
{
    traj_step = controller->get_traj_step();
    traj_len = controller->get_traj_data_real_len();
    // const double* x0_init = controller->get_traj_x0_init(trajectory_selection);
    const double *act_data=0;

    // Eigen::VectorXd x_k_ndof_eig = Eigen::Map<const Eigen::VectorXd>(x0_init, nx);
    Eigen::VectorXd x_k_ndof_eig = x0_nq_init;
    Eigen::Map<Eigen::VectorXd> q_k_ndof_eig(x_k_ndof_eig.data(), nq);
    // q_k_ndof_eig(n_indices_eig) += Eigen::VectorXd::Constant(nq_red, 0.1);
    // q_k_ndof_eig += Eigen::VectorXd::Constant(nq, 0.1);

    init_python();

    for (casadi_uint i = 0; i < traj_len; i=i+traj_step)
    {
        traj_count = i;
        if(use_noise)
            x_measured << x_k_ndof_eig + generateNoiseVector(nx, Ts, mean_noise_amplitude);
        else
            x_measured << x_k_ndof_eig;
        
        if(use_ekf)
            ekf.predict(tau_full.data(), x_measured.data());
        else
            x_filtered_ekf_ptr = x_measured.data();
        
        if(use_lowpass_filter)
            lowpass_filter.run(x_filtered_ekf_ptr); // updates data from x_filtered_ptr
        else
            x_lowpass_filtered_ptr = x_filtered_ekf_ptr;

        x_filtered = Eigen::Map<Eigen::VectorXd>(x_lowpass_filtered_ptr, nx);    
        
        // x_filtered_ptr = x_measured.data();

        timer_mpc_solver.tic();
        tau_full = controller->update_control(x_filtered);
        timer_mpc_solver.toc();
        act_data = controller->get_act_traj_data();
        error_flag = controller->get_error_flag();

        if (traj_count % 100 == 0)
        {
            timer_mpc_solver.print_frequency("traj_count=" + std::to_string(traj_count));
            std::cout << std::endl;
            std::cout << "q_k: |" << q_k_ndof_eig.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "|", "|")) << "|" << std::endl;
            std::cout << "u_k: |" << tau_full.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "|", "|")) << "|"<< std::endl;
        }
        if (traj_count == transient_traj_len)
        {
            std::cout << "Switching to trajectory from data" << std::endl;
        }

        for (casadi_uint j = 0; j < traj_step; j++)
        {
            // Write data to shm:
            current_frequency = timer_mpc_solver.get_frequency()*traj_step;
            shm.write("read_state_data_full", x_lowpass_filtered_ptr, traj_count+j);
            shm.write("read_control_data_full", tau_full.data(), traj_count+j);
            shm.write("read_control_data", tau_full.data());
            shm.write("read_traj_data_full", act_data, traj_count+j);
            shm.write("read_frequency_full", &current_frequency, traj_count+j);
            shm.post_semaphore("shm_changed_semaphore");

            controller->simulateModelRK4(x_k_ndof_eig.data(), tau_full.data(), Ts);
        }

        if (error_flag != ErrorFlag::NO_ERROR)
        {
            std::cerr << "Error flag: " << static_cast<int>(error_flag) << std::endl;
            break;
        }
    }
    int8_t reset = 1;
    shm.write("data_from_simulink_reset", &reset);
    shm.post_semaphore("shm_changed_semaphore");
}

void SharedMemoryController::update()
{
    if(use_ekf)
        ekf.predict(tau_full.data(), x_measured.data());
    else
        x_filtered_ekf_ptr = x_measured.data();
    
    if(use_lowpass_filter)
        lowpass_filter.run(x_filtered_ekf_ptr); // updates data from x_filtered_ptr
    else
        x_lowpass_filtered_ptr = x_filtered_ekf_ptr;

    x_filtered = Eigen::Map<Eigen::VectorXd>(x_lowpass_filtered_ptr, nx);    
    
    // x_filtered_ptr = x_measured.data();

    timer_mpc_solver.tic();
    tau_full = controller->update_control(x_filtered);
    timer_mpc_solver.toc();
    act_data = controller->get_act_traj_data();
    error_flag = controller->get_error_flag();

    for (casadi_uint j = 0; j < traj_step; j++)
    {
        // Write data to shm:
        current_frequency = timer_mpc_solver.get_frequency()*traj_step;
        shm.write("read_state_data_full", x_lowpass_filtered_ptr, traj_count+j);
        shm.write("read_control_data_full", tau_full.data(), traj_count+j);
        shm.write("read_control_data", tau_full.data());
        shm.write("read_traj_data_full", act_data, traj_count+j);
        shm.write("read_frequency_full", &current_frequency, traj_count+j);
        shm.post_semaphore("shm_changed_semaphore");
    }

    if (error_flag != ErrorFlag::NO_ERROR)
    {
        std::cerr << "Error flag: " << static_cast<int>(error_flag) << std::endl;
    }
    if(traj_count < traj_len-1)
        traj_count += traj_step;
}

void SharedMemoryController::run_shm_mode()
{
    sem_wait(ros2_semaphore);
    shm.read_int8("start_cpp", &start_cpp);
    shm.read_int8("stop_cpp", &stop_cpp);
    shm.read_int8("reset_cpp", &reset_cpp);
    shm.read_double("ros2_state_data", x_measured.data());

    if(!run_flag)
    {
        if(start_cpp == 1 && stop_cpp == 0 && reset_cpp == 0)
        {
            reset_cpp_shm_flags();
            update_config();
            init_filter(x_measured);
            init_trajectory(x_measured);
            std::cout << "Starting controller" << std::endl;
            run_flag = true;
        }
    }

    if(reset_cpp == 1)
    {
        std::cout << "Resetting controller" << std::endl;
        reset_cpp_shm_flags();
        run_flag = false;
        error_flag = ErrorFlag::NO_ERROR;
        x_measured_red = x_measured(n_x_indices);
        controller->reset(x_measured_red.data());
        tau_full = Eigen::VectorXd::Zero(nq);
        traj_count = 0;
    }

    if(run_flag)
    {
        if(stop_cpp == 1)
        {
            reset_cpp_shm_flags();
            std::cout << "Stopping controller" << std::endl;
            run_flag = false;
            tau_full = Eigen::VectorXd::Zero(nq);
        }
        else
        {
            update(); // use internally tau_full_ptr and error_flag_ptr and x_nq.data()
        }

        // Write data to ros2
        shm.write("cpp_control_data", tau_full.data());
        error_flag_int8 = static_cast<int8_t>(error_flag);
        shm.write("error_cpp", &error_flag_int8);
        shm.write("valid_cpp", &valid_cpp);
    }

    // send data to python:
    shm.post_semaphore("shm_changed_semaphore");
}