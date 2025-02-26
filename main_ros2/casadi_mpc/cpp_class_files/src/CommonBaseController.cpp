#include "CommonBaseController.hpp"

CommonBaseController::CommonBaseController(const std::string &urdf_filename,
                        const std::string &mpc_config_filename,
                        const std::string &general_config_filename)
                        : urdf_filename(urdf_filename), mpc_config_filename(mpc_config_filename), general_config_filename(general_config_filename),
                        robot_config(get_robot_config()),
                        n_indices(ConstIntVectorMap(robot_config.n_indices, robot_config.nq_red)),
                        n_x_indices(ConstIntVectorMap(robot_config.n_x_indices, robot_config.nx_red)),
                        nq(robot_config.nq), nx(robot_config.nx), nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
                        robot_model(urdf_filename, robot_config, general_config_filename, true),
                        torque_mapper(urdf_filename, robot_config, general_config_filename),
                        trajectory_generator(robot_model, robot_config.dt, general_config_filename), 
                        traj_data_real_len(trajectory_generator.get_traj_data_real_len()),
                        error_flag(ErrorFlag::NO_ERROR)
                        {

                        }

void CommonBaseController::simulateModelEuler(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt)
{
    Eigen::Map<Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);

    // Check for errors
    if (x_k_ndof.hasNaN())
    {
        reset();
        error_flag = ErrorFlag::NAN_DETECTED;
        std::cerr << "NaN values detected in the joint vector!" << std::endl;
        return;
    }

    Eigen::Map<const Eigen::VectorXd> tau(tau_ptr, nq);
    torque_mapper.simulateModelEuler(x_k_ndof, tau, dt);
}

void CommonBaseController::simulateModelRK4(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt)
{
    Eigen::Map<Eigen::VectorXd> x_k_ndof(x_k_ndof_ptr, nx);

    // Check for errors
    if (x_k_ndof.hasNaN())
    {
        reset();
        error_flag = ErrorFlag::NAN_DETECTED;
        std::cerr << "NaN values detected in the joint vector!" << std::endl;
        return;
    }

    Eigen::Map<const Eigen::VectorXd> tau(tau_ptr, nq);
    torque_mapper.simulateModelRK4(x_k_ndof, tau, dt);
}

void CommonBaseController::error_check(Eigen::VectorXd &tau_full)
{
    // Check for NaN values in the torque vector
    if (tau_full.hasNaN())
    {
        reset();
        error_flag = ErrorFlag::NAN_DETECTED;
        std::cerr << "NaN values detected in the torque vector!" << std::endl;
    }

    Eigen::VectorXd delta_u = tau_full - tau_full_prev;

    // Conditions for jumps
    bool condition1 = (delta_u.array() > 0 && delta_u.array() > tau_max_jump).any();
    bool condition2 = (delta_u.array() < 0 && delta_u.array() < -tau_max_jump).any();

    if (condition1 || condition2)
    {
        error_flag = ErrorFlag::JUMP_DETECTED;
        std::cout << "Jump in torque detected (tau = " << tau_full.transpose() << "). Output zero torque." << std::endl;
        tau_full.setZero(); // Set torque to zero
    }
    else
    {
        error_flag = ErrorFlag::NO_ERROR;
        tau_full_prev = tau_full; // Update previous torque
    }
}

void CommonBaseController::init_file_trajectory(uint traj_select, const double *x_k_ndof_ptr,
                                               double T_start, double T_poly, double T_end)
{
    trajectory_generator.init_file_trajectory(traj_select, x_k_ndof_ptr, T_start, T_poly, T_end);
    update_trajectory_data(x_k_ndof_ptr);
}

void CommonBaseController::init_custom_trajectory(ParamPolyTrajectory param)
{
    trajectory_generator.init_custom_trajectory(param);
    update_trajectory_data(param.x_init.data());
}

Eigen::VectorXd CommonBaseController::get_file_traj_x0_nq_init(casadi_uint traj_select)
{
    return *trajectory_generator.get_traj_file_x0_init(traj_select);
}

Eigen::VectorXd CommonBaseController::get_transient_traj_x0_init()
{
    return *trajectory_generator.get_traj_x0_init();
}

Eigen::VectorXd CommonBaseController::get_transient_traj_x0_red_init()
{
    Eigen::VectorXd x0_init = get_transient_traj_x0_init();
    Eigen::VectorXd x0_init_red = x0_init(n_x_indices);
    return x0_init_red;
}