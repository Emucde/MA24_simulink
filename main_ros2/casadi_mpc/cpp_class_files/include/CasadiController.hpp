#ifndef CASADICONTROLLER_HPP
#define CASADICONTROLLER_HPP

#include "CommonBaseController.hpp"
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
#include "RobotModel.hpp"
#include "CasadiMPC.hpp"
#include "error_flags.h"

class BaseSolver
{
public:
    BaseSolver(CasadiMPC *active_mpc,
               robot_config_t &robot_config) : active_mpc(active_mpc),
                                               nq_red(robot_config.nq_red),
                                               nx_red(robot_config.nx_red)
    {
    }
    virtual bool solveMPC(const casadi_real *const x_k_in) = 0;
    virtual void switch_controller(CasadiMPC *new_mpc) {active_mpc = new_mpc;}
    virtual casadi_real *get_optimal_control() = 0;
    virtual ~BaseSolver() = default;

protected:
    CasadiMPC *active_mpc;
    bool planner_mpc;
    const casadi_uint nq_red, nx_red;
    Eigen::VectorXd u_opt = Eigen::VectorXd::Zero(nq_red);
};

class CasadiController : public CommonBaseController
{
private:
    std::vector<CasadiMPC> casadi_mpcs;                // MPC objects
    CasadiMPCType selected_mpc_type;                   // Active MPC type
    CasadiMPC *active_mpc;                             // Active MPC object
    mpc_config_t *active_mpc_config;                   // Active MPC configuration
    const mpc_input_config_t *active_mpc_input_config; // Active MPC input configuration

    std::string traj_file;    // Path to trajectory data file
    std::string x0_init_file; // Path to initial state data of trajectories

private:
    const std::string mpc_config_filename; // Path to the casadi mpc weights file
    nlohmann::json param_mpc_weight;
    bool use_planner;

    double *u_k_ptr; // - Pointer to the optimal control

    casadi_real *w_ptr;                // Workspace real address
    casadi_uint traj_data_per_horizon; // Trajectory data per horizon
    casadi_uint *mpc_traj_indices;     // Trajectory indices for not equidistant sampling

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

public:
    // Constructor
    CasadiController(const std::string &urdf_filename,
                     const std::string &mpc_config_filename,
                     const std::string &general_config_filename);

    // CasadiController: solve the MPC
    Eigen::VectorXd update_control(const Eigen::VectorXd &x_nq) override;
    void update_mpc_weights();
    void update_config() override;

    void collinearity_weight_x(const casadi_real *const x_k_ndof_ptr);
    void reset(const double *const x_k_in) override;
    void reset() override;

    // Getters and setters
    void setActiveMPC(CasadiMPCType mpc);
    // void setTransientTrajParams(double T_start, double T_poly, double T_end);

    void set_planner_mode(bool use_planner_new);

    // increase counter from casadi mpc if it solves too slow
    void increase_traj_count() override
    {
        active_mpc->increase_traj_count();
    }

    void set_traj_count(uint new_traj_count) override
    {
        active_mpc->set_traj_count(new_traj_count);
    }

    CasadiMPCType get_active_mpc_type()
    {
        return selected_mpc_type;
    }

    casadi_real *get_u_next()
    {
        return active_mpc->get_u_next();
    }

    casadi_real *get_u_opt_full()
    {
        return active_mpc->get_optimal_control();
    }

    casadi_uint get_N_step() override
    {
        return active_mpc->get_N_step();
    }

    uint get_traj_count() override
    {
        return active_mpc->get_traj_count();
    }

    casadi_uint get_traj_step() override
    {
        return active_mpc->get_traj_step();
    }

    // Method to switch the trajectory (for creating run init_trajectory)
    void switch_traj(uint traj_select) override;

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

    // Method to get the is kinematic mpc flag
    bool get_is_kinematic_mpc()
    {
        return active_mpc->is_kinematic_mpc;
    }

    // Method to get w pointer
    casadi_real *get_w()
    {
        return active_mpc->get_w();
    }

    const casadi_real *get_q_ref_nq()
    {
        return robot_config.q_0_ref;
    }

    const casadi_real *get_act_traj_data() override
    {
        return active_mpc->get_act_traj_data();
    }

private:
    // Private methods
    void update_trajectory_data(const casadi_real *const x_k_ndof_ptr) override;

    class StandardSolver : public BaseSolver
    {
    public:
        StandardSolver(CasadiMPC *active_mpc,
                       robot_config_t &robot_config) : BaseSolver(active_mpc,
                                                                  robot_config) {}
        bool solveMPC(const casadi_real *const x_k_in) override
        {
            return active_mpc->solve(x_k_in);
        }
        casadi_real *get_optimal_control() override
        {
            return active_mpc->get_optimal_control();
        }
    };

    class PlannerSolver : public BaseSolver
    {
    public:
        PlannerSolver(CasadiMPC *active_mpc,
                      nlohmann::json &param_mpc_weight,
                      robot_config_t &robot_config) : BaseSolver(active_mpc, robot_config),
                                              param_mpc_weight(param_mpc_weight),
                                              planner_only_solver(active_mpc),
                                              planner_and_controller_solver(active_mpc) {}
        class Solver
        {
            public:
                Solver(CasadiMPC *active_mpc): active_mpc(active_mpc) {}
                virtual bool solveMPC(const casadi_real *const x_k_in) = 0;
                CasadiMPC *active_mpc;
        };

        class PlannerOnlySolver : public Solver
        {
            public:
                PlannerOnlySolver(CasadiMPC *active_mpc): Solver(active_mpc) {}
                bool solveMPC(const casadi_real *const) override
                {
                    return active_mpc->solve_planner();
                }
        };

        class PlannerAndControllerSolver : public Solver
        {
            public:
                PlannerAndControllerSolver(CasadiMPC *active_mpc): Solver(active_mpc) {}
                bool solveMPC(const casadi_real *const x_k_in) override
                {
                    return active_mpc->solve(x_k_in);
                }
        };

        bool solveMPC(const casadi_real *const x_k_in) override;
        void switch_controller(CasadiMPC *new_mpc) override;
        void update_planner_params();
        casadi_real *get_optimal_control() override
        {
            return u_opt.data();
        }

    private:
        nlohmann::json &param_mpc_weight;
        bool planner_mpc;
        casadi_real *x_d_ptr;
        casadi_real *q_pp_d_ptr;
        Eigen::VectorXd K_P_q, K_D_q;
        Solver *solver;
        PlannerOnlySolver planner_only_solver;
        PlannerAndControllerSolver planner_and_controller_solver;
    };

    BaseSolver *solver;
    StandardSolver standard_solver;
    PlannerSolver planner_solver;
};
#endif // CASADICONTROLLER_HPP