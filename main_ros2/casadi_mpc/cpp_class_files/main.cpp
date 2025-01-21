#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy

#include "include/FullSystemTorqueMapper.hpp"
#include "include/CasadiMPC.hpp"
#include "include/CasadiController.hpp"
#include "include/casadi_controller_types.hpp"
#include "param_robot.h"
#include "casadi_types.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include "include/eigen_templates.hpp"
#include "include/TicToc.hpp"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <errno.h> // Include errno for error handling
#include <semaphore.h>

#include "param_robot.h"

// #define PLOT_DATA
#define TRANSIENT_TRAJ_TESTS

sem_t *open_write_sem(const char *name)
{
    sem_t *sem = sem_open(name, O_RDWR, 0666, 0);
    if (sem == SEM_FAILED)
    {
        std::cerr << "write: open: Error opening semaphore " << name << ": " << strerror(errno) << std::endl;
        throw std::runtime_error("Error allocating semaphore");
        return nullptr;
    }
    return sem;
}

int open_read_shm(const char *name)
{
    int fd = shm_open(name, O_RDONLY, 0666);
    if (fd == -1)
    {
        std::cerr << "read: open: Error opening shared memory " << name << ": " << strerror(errno) << std::endl;
        throw std::runtime_error("Error allocating shared memory");
        return 0;
    }
    return fd;
}

int open_write_shm(const char *name)//, size_t size)
{
    int fd = shm_open(name, O_RDWR, 0666);
    if (fd == -1)
    {
        std::cerr << "write: open: Error opening shared memory: " << strerror(errno) << std::endl;
        throw std::runtime_error("Error allocating shared memory");
        return 0;
    }

    // // Allocate the shared memory segment
    // if (ftruncate(fd, size) == -1)
    // {
    //     std::cerr << "write: open: Error allocating shared memory: " << strerror(errno) << std::endl;
    //     // throw std::runtime_error("Error allocating shared memory");
    //     close(fd);
    //     return 0;
    // }

    return fd;
}

// Existing read function
int read_shared_memory(int fd, double *data, size_t size)
{
    double *shared_data = (double *)mmap(0, size, PROT_READ, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        std::cerr << "read: Error mapping shared memory: " << strerror(errno) << std::endl;
        return -1;
    }

    memcpy(data, shared_data, size);

    if (munmap(shared_data, size) == -1)
    {
        std::cerr << "read: Error unmapping shared memory: " << strerror(errno) << std::endl;
        return -1;
    }

    return 0;
}

int read_shared_memory_flag(int fd, int8_t *data, size_t size)
{
    int8_t *shared_data = (int8_t *)mmap(0, size, PROT_READ, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        std::cerr << "readf: Error mapping shared memory: " << strerror(errno) << std::endl;
        return -1;
    }

    memcpy(data, shared_data, size);

    // Clean up
    if (munmap(shared_data, size) == -1)
    {
        std::cerr << "readf: Error unmapping shared memory: " << strerror(errno) << std::endl;
        return -1;
    }

    return 0;
}

// Combined write function for shared memory
int write_to_shared_memory(int fd, const void *data, size_t size)
{
    void *shared_data = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        std::cerr << "write: Error mapping shared memory: " << strerror(errno) << std::endl;
        return -1;
    }

    // Write data to shared memory
    memcpy(shared_data, data, size);

    // Clean up
    if (munmap(shared_data, size) == -1)
    {
        std::cerr << "write: Error unmapping shared memory: " << strerror(errno) << std::endl;
        return -1;
    }

    return 0;
}

// shared memory global variables
int shm_start_mpc = 0;
int shm_reset_mpc = 0;
int shm_stop_mpc = 0;
int shm_readonly_mode=0;
int shm_read_traj_length=0;
int shm_read_traj_data=0;
int shm_read_state_data=0;
int shm_read_control_data=0;
sem_t *shm_changed_semaphore=0;

void close_shared_memories()
{
    if (shm_start_mpc != 0)
        close(shm_start_mpc);
    if (shm_reset_mpc != 0)
        close(shm_reset_mpc);
    if (shm_stop_mpc != 0)
        close(shm_stop_mpc);
    if (shm_readonly_mode != 0)
        close(shm_readonly_mode);
    if (shm_read_traj_length != 0)
        close(shm_read_traj_length);
    if (shm_read_traj_data != 0)
        close(shm_read_traj_data);
    if (shm_read_state_data != 0)
        close(shm_read_state_data);
    if (shm_read_control_data != 0)
        close(shm_read_control_data);
    if (shm_changed_semaphore != 0)
        sem_close(shm_changed_semaphore);
}

void open_shared_memories()
{
    shm_start_mpc         = open_write_shm("data_from_simulink_start");
    shm_reset_mpc         = open_write_shm("data_from_simulink_reset");
    shm_stop_mpc          = open_write_shm("data_from_simulink_stop");
    shm_readonly_mode     = open_write_shm("readonly_mode");
    shm_read_traj_length  = open_write_shm("read_traj_length");
    shm_read_traj_data    = open_write_shm("read_traj_data");
    shm_read_state_data   = open_write_shm("read_state_data");
    shm_read_control_data = open_write_shm("read_control_data");
    shm_changed_semaphore = open_write_sem("shm_changed_semaphore");

    std::cout << "Shared memory opened successfully." << std::endl;
}



casadi_real x_k_tst[12] = {0};

void test_fun1(const casadi_real* x_k_ndof, const casadi_uint *n_x_indices_ptr)
{
    casadi_uint nx_red = 12;
    
    double x_k[nx_red];
    for (casadi_uint i = 0; i < nx_red; i++)
    {
        x_k[i] = x_k_ndof[n_x_indices_ptr[i]];
    }
    //0.01s
    memcpy(x_k_tst, x_k, nx_red*sizeof(casadi_real));
}

void test_fun2(const casadi_real* x_k_ndof, const Eigen::VectorXi n_x_indices_eig)
{
    casadi_uint nx_red = 12;

    // Eigen::Map<Eigen::VectorXd>(x_k, nx_red) = x_k_ndof_eig(n_x_indices_eig);

    // double x_k[nx_red];
    // Eigen::Map<Eigen::VectorXd>(x_k, nx_red) = Eigen::Map<const Eigen::VectorXd>(x_k_ndof, nx)(n_x_indices_eig);
    // memcpy(x_k_tst, x_k, nx_red*sizeof(casadi_real));
    // 0.016s

    // Eigen::VectorXd x_k_ndof_tst = Eigen::Map<const Eigen::VectorXd>(x_k_ndof, nx);
    // memcpy(x_k_tst, x_k_ndof_tst(n_x_indices_eig).eval().data(), nx_red*sizeof(casadi_real));
    // 0.02s

    // Eigen::VectorXd x_k_ndof_tst = Eigen::Map<const Eigen::VectorXd>(x_k_ndof, nx);
    // Eigen::VectorXd selected_indices = x_k_ndof_tst(n_x_indices_eig);
    // memcpy(x_k_tst, selected_indices.data(), nx_red*sizeof(casadi_real));
    // 0.022s

    double x_k[nx_red];
    for (casadi_uint i = 0; i < nx_red; i++)
    {
        x_k[i] = x_k_ndof[n_x_indices_eig[i]];
    }
    memcpy(x_k_tst, x_k, nx_red*sizeof(casadi_real));
    //0.01s
}

int main()
{
    // Configuration flags
    bool use_gravity = false;
    const std::string urdf_filename = "../../../urdf_creation/fr3_no_hand_7dof.urdf";
    const std::string tcp_frame_name = "fr3_link8_tcp";

    CasadiController controller(urdf_filename, tcp_frame_name, use_gravity);
    controller.setActiveMPC(MPCType::MPC8);

    const casadi_uint nq = controller.nq;
    const casadi_uint nx = controller.nx;
    const casadi_uint nq_red = controller.nq_red;
    const casadi_uint nx_red = controller.nx_red;
    const casadi_uint *n_indices_ptr = controller.get_n_indices();
    const casadi_uint *n_x_indices_ptr = controller.get_n_x_indices();
    const casadi_uint traj_data_real_len = controller.get_traj_data_len();
    ErrorFlag error_flag = ErrorFlag::NO_ERROR;

    casadi_real x_k_ndof[nx] = {0};
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nq);

    Eigen::Map<Eigen::VectorXd> q_k_ndof_eig(x_k_ndof, nq);
    Eigen::Map<Eigen::VectorXd> x_k_ndof_eig(x_k_ndof, nx);

    // q_k_ndof_eig = Eigen::Map<const Eigen::VectorXd> (controller.get_q_ref_nq(), nq);

    
    Eigen::VectorXi n_indices_eig = ConstIntVectorMap(n_indices_ptr, nq_red);
    Eigen::VectorXi n_x_indices_eig = ConstIntVectorMap(n_x_indices_ptr, nx_red);

    open_shared_memories();

    // Measure execution time
    TicToc timer_mpc_solver;
    TicToc timer_total;

#ifdef PLOT_DATA
    std::ofstream x_k_ndof_file("x_k_ndof_data.txt");
    std::ofstream tau_full_file("tau_full_data.txt");
#endif

#ifdef TRANSIENT_TRAJ_TESTS


    #define TRAJ_SELECT 1
    // controller.switch_traj(TRAJ_SELECT);
    x_k_ndof_eig = Eigen::Map<const Eigen::VectorXd> (controller.get_traj_x0_init(TRAJ_SELECT), nx);
    // q_k_ndof_eig(n_indices_eig) += Eigen::VectorXd::Constant(nq_red, 0.1);
    // q_k_ndof_eig += Eigen::VectorXd::Constant(nq, 0.1);

    controller.init_trajectory(TRAJ_SELECT, x_k_ndof, 0.0, 1.0, 2.0);
    

    Eigen::MatrixXd trajectory = controller.get_transient_traj_data();
    casadi_uint transient_traj_len = controller.get_transient_traj_len();

    timer_total.tic();

    // // Output the trajectory results
    // for (int i = 0; i < trajectory.cols(); ++i)
    // {
    //     if (i % 10 == 0)
    //         std::cout << "Step " << i << ": " << trajectory.col(i).transpose() << std::endl;
    // }
#endif

    Eigen::VectorXd q_filtered = Eigen::VectorXd::Zero(nq);
    Eigen::VectorXd x_filtered = Eigen::VectorXd::Zero(nx);
    Eigen::VectorXd q_measured = Eigen::VectorXd::Zero(nq);
    Eigen::VectorXd x_measured = Eigen::VectorXd::Zero(nq);
    double Ts = 0.001, T = 1/400, Phi = 0;

    // enable shm read mode:
    casadi_uint total_traj_len = traj_data_real_len + transient_traj_len;
    int8_t readonly_mode = 1, start = 1;
    write_to_shared_memory(shm_readonly_mode, &readonly_mode, sizeof(int8_t));
    write_to_shared_memory(shm_read_traj_length, &total_traj_len, sizeof(casadi_uint));
    write_to_shared_memory(shm_start_mpc, &start, sizeof(int8_t));

    // Write data to shm:
    write_to_shared_memory(shm_read_state_data, x_k_ndof, nx * sizeof(casadi_real));
    write_to_shared_memory(shm_read_control_data, tau_full.data(), nq * sizeof(casadi_real));
    write_to_shared_memory(shm_read_traj_data, controller.get_act_traj_data(), 7 * sizeof(casadi_real));

    sem_post(shm_changed_semaphore); // activate semaphore

    // Main loop for trajectory processing
    for (casadi_uint i = 0; i < total_traj_len; i++)
    {
        if(i == 0)
        {
            x_filtered = x_k_ndof_eig.head(nx);
        }
        else
        {
            Phi = std::exp(-Ts/T);
            x_filtered = Phi * x_filtered + (1 - Phi) * x_measured;
        }

        timer_mpc_solver.tic();
        tau_full = controller.solveMPC(x_filtered.data());
        error_flag = controller.get_error_flag();
        timer_mpc_solver.toc();

        if (i % 100 == 0)
        {
            std::cout << "q_k: " << q_k_ndof_eig.transpose();
            std::cout << "Full torque: " << tau_full.transpose();
            timer_mpc_solver.print_frequency("\t");
            std::cout << std::endl;
        }
        if (i == transient_traj_len)
        {
            std::cout << "Switching to trajectory from data" << std::endl;
        }

#ifdef PLOT_DATA
        x_k_ndof_file << x_k_ndof_eig.transpose() << std::endl;
        tau_full_file << tau_full.transpose() << std::endl;
#endif

        // simulate the model
        controller.simulateModel(x_k_ndof, tau_full.data(), 1e-3);
        x_measured = Eigen::Map<Eigen::VectorXd>(x_k_ndof, nx);

        // Write data to shm:
        write_to_shared_memory(shm_read_state_data, x_k_ndof, nx * sizeof(casadi_real));
        write_to_shared_memory(shm_read_control_data, tau_full.data(), nq * sizeof(casadi_real));
        write_to_shared_memory(shm_read_traj_data, controller.get_act_traj_data(), 7 * sizeof(casadi_real));

        sem_post(shm_changed_semaphore); // activate semaphore

        if (error_flag != ErrorFlag::NO_ERROR)
        {
            std::cerr << "Error flag: " << static_cast<int>(error_flag) << std::endl;
            break;
        }
    }

    // Measure and print execution time
    timer_mpc_solver.print_time("Total controller.solveMPC time: ");
    std::cout << std::endl;

    timer_total.toc();
    timer_total.print_time("Total execution time: ");
    std::cout << std::endl;

#ifdef PLOT_DATA
    x_k_ndof_file.close();
    tau_full_file.close();
#endif
    write_to_shared_memory(shm_reset_mpc, &start, sizeof(int8_t));
    sem_post(shm_changed_semaphore); // activate semaphore
    close_shared_memories();
    return 0;
}

/*
% Read the torque data
torque_data = dlmread('./main_ros2/read_bin_data/tau_full_data.txt');

% Read the state data
state_data = dlmread('./main_ros2/read_bin_data/x_k_ndof_data.txt');

% Create a figure with subplots
figure;

% Plot torque data
subplot(3,1,1);
plot(torque_data);
title('7-Dimensional Torque');
xlabel('Time step');
ylabel('Torque (Nm)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

% Plot joint angles
subplot(3,1,2);
plot(state_data(:,1:7));
title('Joint Angles');
xlabel('Time step');
ylabel('Angle (rad)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

% Plot joint velocities
subplot(3,1,3);
plot(state_data(:,8:14));
title('Joint Velocities');
xlabel('Time step');
ylabel('Velocity (rad/s)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

% Adjust the layout
sgtitle('Robot Joint Data');

*/