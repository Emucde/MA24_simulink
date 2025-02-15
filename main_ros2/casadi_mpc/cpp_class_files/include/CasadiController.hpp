#ifndef CASADICONTROLLER_HPP
#define CASADICONTROLLER_HPP

#include "json.hpp"
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry> // For rotations and quaternions
#include "casadi_types.h"
#include "param_robot.h"
#include "mpc_config_types.h"
#include "TrajectoryGenerator.hpp"
#include "FullSystemTorqueMapper.hpp"
#include "CasadiMPC.hpp"
#include "error_flags.h"

class BaseSolver
{
public:
    BaseSolver(CasadiMPC *active_mpc) : active_mpc(active_mpc) {}
    virtual bool solveMPC(const casadi_real *const x_k_ndof_ptr) = 0;
    virtual void switch_controller(CasadiMPC *new_mpc) { active_mpc = new_mpc; }
    virtual ~BaseSolver() = default;
    protected:
        CasadiMPC *active_mpc;
};

class CasadiController // : public TrajectoryGenerator
{
private:
    robot_config_t robot_config;                       // Robot configuration
    std::vector<CasadiMPC> casadi_mpcs;                // MPC objects
    CasadiMPCType selected_mpc_type;                   // Active MPC type
    CasadiMPC *active_mpc;                             // Active MPC object
    mpc_config_t *active_mpc_config;                   // Active MPC configuration
    const mpc_input_config_t *active_mpc_input_config; // Active MPC input configuration

    std::string traj_file;    // Path to trajectory data file
    std::string x0_init_file; // Path to initial state data of trajectories

public:
    const casadi_uint nq;     // Number of degrees of freedom
    const casadi_uint nx;     // Number of reduced degrees of freedom
    const casadi_uint nq_red; // Number of reduced degrees of freedom
    const casadi_uint nx_red; // Number of reduced degrees of freedom

private:
    const Eigen::VectorXi n_indices;
    const Eigen::VectorXi n_x_indices;
    FullSystemTorqueMapper torque_mapper;      // Torque mapper
    TrajectoryGenerator trajectory_generator;  // Trajectory generator
    const std::string casadi_mpc_weights_file; // Path to the casadi mpc weights file
    nlohmann::json param_mpc_weight;
    bool use_planner;

    double *u_k_ptr; // - Pointer to the optimal control

    casadi_real *w_ptr;                // Workspace real address
    casadi_uint traj_data_per_horizon; // Trajectory data per horizon
    casadi_uint *mpc_traj_indices;     // Trajectory indices for not equidistant sampling
    casadi_real dt;

    // std::map<std::string, double> param_transient_traj_poly;
    // casadi_uint traj_rows;
    // casadi_uint transient_traj_cnt;
    // Eigen::MatrixXd transient_traj_data;
    // casadi_uint transient_traj_len;       // number of columns of the transient trajectory data
    // std::vector<Eigen::MatrixXd> all_traj_data;
    // std::vector<Eigen::VectorXd> all_traj_x0_init;
    // casadi_uint selected_trajectory;
    // Eigen::MatrixXd traj_data;
    // Eigen::VectorXd traj_data_x0_init;
    // casadi_uint traj_len;
    // casadi_uint traj_real_len; // number of columns of the singular trajectory data without additional samples for last prediction horizon

    Eigen::VectorXd tau_full_prev;
    ErrorFlag error_flag = ErrorFlag::NO_ERROR;
    double tau_max_jump = 5.0;

public:
    // Constructor
    CasadiController(const std::string &urdf_path,
                     const std::string &casadi_mpc_weights_file,
                     const std::string &tcp_frame_name,
                     bool use_gravity,
                     bool use_planner);

    // solve the MPC
    Eigen::VectorXd solveMPC(const casadi_real *const x_k_ndof_ptr);

    void update_mpc_weights();

    // Initialize trajectory data
    void init_file_trajectory(casadi_uint traj_select, const casadi_real *x_k_ndof_ptr,
                              double T_start, double T_poly, double T_end);

    // Method for creating a custom trajectory with extra samples for the last prediction horizon
    void init_custom_trajectory(ParamPolyTrajectory param);

    // Method to simulate the robot model
    void simulateModelEuler(casadi_real *const x_k_ndof_ptr, const casadi_real *const tau_ptr, double dt);
    void simulateModelRK4(casadi_real *const x_k_ndof_ptr, const casadi_real *const tau_ptr, double dt);

    void reset();

    // Getters and setters
    void setActiveMPC(CasadiMPCType mpc);
    // void setTransientTrajParams(double T_start, double T_poly, double T_end);

    void set_planner_mode(bool use_planner)
    {
        this->use_planner = use_planner;
        if (use_planner)
        {
            solver = &planner_solver;
        }
        else
        {
            solver = &standard_solver;
        }
    }

    // increase counter from casadi mpc if it solves too slow
    void increase_traj_count()
    {
        active_mpc->increase_traj_count();
    }

    const casadi_uint *get_n_indices()
    {
        return robot_config.n_indices;
    }

    // Method to switch the trajectory (for creating run init_trajectory)
    void switch_traj(casadi_uint traj_select);

    // Method to get n_x_indices
    const casadi_uint *get_n_x_indices()
    {
        return robot_config.n_x_indices;
    }

    // Method to get n_indices_fixed
    const casadi_uint *get_n_indices_fixed()
    {
        return robot_config.n_indices_fixed;
    }

    // Method to get n_x_indices_fixed
    const casadi_uint *get_n_x_indices_fixed()
    {
        return robot_config.n_x_indices_fixed;
    }

    // Method to output the optimal control
    casadi_real *get_optimal_control()
    {
        return active_mpc->get_optimal_control();
    }

    // Method to get x_k
    casadi_real *get_x_k()
    {
        return active_mpc->get_x_k();
    }

    // Method to get the real length of the trajectory data (= length of the trajectory data without additional samples for last prediction horizon)
    casadi_uint get_traj_data_real_len()
    {
        return trajectory_generator.get_traj_data_real_len();
    }

    // Method to get the length of the trajectory data
    casadi_uint get_traj_data_len()
    {
        return trajectory_generator.get_traj_data_len();
    }

    // Method to get transient trajectory data
    const Eigen::MatrixXd *get_transient_traj_data()
    {
        return trajectory_generator.get_transient_traj_data();
    }

    // Method to get the transient trajectory length
    casadi_uint get_transient_traj_len()
    {
        return trajectory_generator.get_transient_traj_len();
    }

    // Method to get the is kinematic mpc flag
    bool get_is_kinematic_mpc()
    {
        return active_mpc->is_kinematic_mpc;
    }

    // Method to get the error flag
    ErrorFlag get_error_flag()
    {
        return error_flag;
    }

    // Method to get w pointer
    casadi_real *get_w()
    {
        return active_mpc->get_w();
    }

    // Method to set the maximum torque jump
    void set_tau_max_jump(double tau_jump)
    {
        tau_max_jump = tau_jump;
    }

    const casadi_real *get_q_ref_nq()
    {
        return robot_config.q_0_ref;
    }

    const Eigen::MatrixXd *get_trajectory()
    {
        return trajectory_generator.get_traj_data();
    }

    const casadi_real *get_act_traj_data()
    {
        return active_mpc->get_act_traj_data();
    }

    const casadi_real *get_act_traj_x0_init()
    {
        return trajectory_generator.get_act_traj_x0_init()->data();
    }

    const casadi_real *get_traj_x0_init(casadi_uint traj_select)
    {
        return trajectory_generator.get_traj_file_x0_init(traj_select)->data();
    }

private:
    // Private methods
    void update_trajectory_data(const casadi_real *const x_k_ndof_ptr);

    class StandardSolver : public BaseSolver
    {
    public:
        StandardSolver(CasadiMPC *active_mpc) : BaseSolver(active_mpc) {}
        bool solveMPC(const casadi_real *const x_k_ptr) override
        {
            return active_mpc->solve(x_k_ptr);
        }
    };

    class PlannerSolver : public BaseSolver
    {
    public:
        PlannerSolver(CasadiMPC *active_mpc) : BaseSolver(active_mpc) {}
        bool solveMPC(const casadi_real *const) override
        {
            return active_mpc->solve_planner();
        }
    };

    BaseSolver *solver;
    StandardSolver standard_solver;
    PlannerSolver planner_solver;
};
#endif // CASADICONTROLLER_HPP