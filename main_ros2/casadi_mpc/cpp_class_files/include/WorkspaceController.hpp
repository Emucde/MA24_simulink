#include "param_robot.h"
#include "robot_data.hpp"
#include "workspace_controller_config.hpp"
#include <iostream>
#include "json.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> // For rotations and quaternions
#include <vector>
#include "eigen_templates.hpp"
#include "error_flags.h"

#include "RobotModel.hpp"
#include "TrajectoryGenerator.hpp"

class BaseWorkspaceController
{
public:
    BaseWorkspaceController(RobotModel &robot_model,
                   const std::string &workspace_config_path,
                   TrajectoryGenerator &trajectory_generator)
        : robot_model(robot_model), workspace_config_path(workspace_config_path),
          trajectory_generator(trajectory_generator),
          nq(robot_model.nq), nx(robot_model.nx),
          J(Eigen::MatrixXd::Zero(6, nq)), J_pinv(Eigen::MatrixXd::Zero(nq, 6)), J_p(Eigen::MatrixXd::Zero(6, nq)),
          M(Eigen::MatrixXd::Zero(nq, nq)), C(Eigen::MatrixXd::Zero(nq, nq)), C_rnea(Eigen::VectorXd::Zero(nq)),
          g(Eigen::VectorXd::Zero(nq)),
          q(Eigen::VectorXd::Zero(nq)), q_p(Eigen::VectorXd::Zero(nq)),
          x_err(Eigen::VectorXd::Zero(6)), x_err_p(Eigen::VectorXd::Zero(6)),
          x_d_p(Eigen::VectorXd::Zero(6)), x_d_pp(Eigen::VectorXd::Zero(6)),
          n_indices(ConstIntVectorMap(robot_model.robot_config.n_indices, robot_model.robot_config.nq_red)),
          dt(robot_model.robot_config.dt), traj_count(0)
    {
        update_controller_settings();
    }
    virtual Eigen::VectorXd control(const Eigen::VectorXd &x) = 0; // Pure virtual function
    nlohmann::json read_config(std::string file_path);
    void update_controller_settings();
    Eigen::MatrixXd computeJacobianRegularization();       // Common method in BaseWorkspaceController
    void calculateControlData(const Eigen::VectorXd &x);   // Common method in BaseWorkspaceController
    void calculateControlDataID(const Eigen::VectorXd &x, const Eigen::VectorXd &x_d);   // Common method in BaseWorkspaceController
    virtual ~BaseWorkspaceController() = default; // Virtual destructor
protected:
    RobotModel &robot_model;
    std::string workspace_config_path;
    ControllerSettings controller_settings;
    RegularizationSettings regularization_settings;
    RegularizationMode sing_method;
    TrajectoryGenerator &trajectory_generator;
    const int nq, nx; // this is nq_red and nx_red!!!
    Eigen::MatrixXd J, J_pinv, J_p;
    Eigen::MatrixXd M, C, C_rnea;
    Eigen::VectorXd g;
    Eigen::VectorXd q, q_p;
    Eigen::VectorXd x_err, x_err_p;
    Eigen::VectorXd x_d_p, x_d_pp;
    Eigen::VectorXd column_weights = Eigen::VectorXd::Ones(nq);
    const Eigen::VectorXi n_indices;
    double dt;

public:
    uint traj_count;
};

class WorkspaceController
{
public:
    // Constructor
    WorkspaceController(const std::string &urdf_path,
                        const std::string &workspace_config_path,
                        const std::string &tcp_frame_name,
                        bool use_gravity);

    ControllerType get_classic_controller_type(const std::string &controller_type_str);
    std::string get_classic_controller_string(ControllerType controller_type);
    void switchController(ControllerType type);
    void update_controller_settings()
    {
        ct_controller.update_controller_settings();
        pd_plus_controller.update_controller_settings();
        inverse_dyn_controller.update_controller_settings();
    }

    void increase_traj_count()
    {
        active_controller->traj_count++;
    }

    void set_traj_count(casadi_uint new_traj_count)
    {
        active_controller->traj_count = new_traj_count;
    }

    void switch_traj(uint traj_select)
    {
        trajectory_generator.switch_traj(traj_select);
    }
    void simulateModelEuler(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt);
    void simulateModelRK4(double *const x_k_ndof_ptr, const double *const tau_ptr, double dt);
    Eigen::VectorXd update(const double *const x_nq);
    void reset();
    void reset(const casadi_real *const x_k_in);

    // Initialize trajectory data
    void init_file_trajectory(uint traj_select, const double *x_k_ndof_ptr,
                              double T_start, double T_poly, double T_end)
    {
        trajectory_generator.init_file_trajectory(traj_select, x_k_ndof_ptr, T_start, T_poly, T_end);
    }

    // Method for creating a custom trajectory with extra samples for the last prediction horizon
    void init_custom_trajectory(ParamPolyTrajectory param_target)
    {
        trajectory_generator.init_custom_trajectory(param_target);
    }

    // Method to set the maximum torque jump
    void set_tau_max_jump(double tau_jump)
    {
        tau_max_jump = tau_jump;
    }

    // Method to get the error flag
    ErrorFlag get_error_flag()
    {
        return error_flag;
    }

    void reset_error_flag()
    {
        error_flag = ErrorFlag::NO_ERROR;
    }

    uint get_traj_count()
    {
        return active_controller->traj_count;
    }

    uint get_traj_step()
    {
        return 1; // always 1 because it is extremely fast
    }

    uint get_N_step()
    {
        return 1;
    }

    const Eigen::MatrixXd *get_trajectory()
    {
        return trajectory_generator.get_traj_data();
    }

    const double *get_act_traj_data()
    {
        uint traj_count = active_controller->traj_count;
        return trajectory_generator.get_traj_data()->col((traj_count > 0 ? traj_count - 1 : traj_count)).data();
    }

    int get_traj_data_real_len()
    {
        return trajectory_generator.get_traj_data_real_len();
    }

    uint get_transient_traj_len()
    {
        return trajectory_generator.get_transient_traj_len();
    }

    const double *get_traj_x0_init(uint traj_select)
    {
        return trajectory_generator.get_traj_file_x0_init(traj_select)->data();
    }

    FullSystemTorqueMapper* get_torque_mapper()
    {
        return &torque_mapper;
    }

    void set_torque_mapper_config(FullSystemTorqueMapper::Config &config)
    {
        torque_mapper.setConfiguration(config);
    }

    Eigen::VectorXd get_traj_x0_red_init(casadi_uint traj_select);
    Eigen::VectorXd get_act_traj_x0_red_init();

private:
    const std::string urdf_path;
    const std::string workspace_config_path;
    const std::string tcp_frame_name;
    robot_config_t robot_config;
    const Eigen::VectorXi n_indices, n_x_indices;
    const int nq, nx, nq_red, nx_red;
    RobotModel robot_model;
    FullSystemTorqueMapper torque_mapper;
    TrajectoryGenerator trajectory_generator;

    // Nested classes inheriting from BaseWorkspaceController
    class CTController : public BaseWorkspaceController
    {
    public:
        CTController(RobotModel &robot_model,
                     const std::string &workspace_config_path,
                     TrajectoryGenerator &trajectory_generator)
            : BaseWorkspaceController(robot_model, workspace_config_path, trajectory_generator) {}
        Eigen::VectorXd control(const Eigen::VectorXd &x) override;
        ~CTController() override = default;
    };

    class PDPlusController : public BaseWorkspaceController
    {
    public:
        PDPlusController(RobotModel &robot_model,
                         const std::string &workspace_config_path,
                         TrajectoryGenerator &trajectory_generator)
            : BaseWorkspaceController(robot_model, workspace_config_path, trajectory_generator) {}
        Eigen::VectorXd control(const Eigen::VectorXd &x) override;
        ~PDPlusController() override = default;
    };

    class InverseDynamicsController : public BaseWorkspaceController
    {
    public:
        InverseDynamicsController(RobotModel &robot_model,
                                  const std::string &workspace_config_path,
                                  TrajectoryGenerator &trajectory_generator)
            : BaseWorkspaceController(robot_model, workspace_config_path, trajectory_generator) {}
        Eigen::VectorXd control(const Eigen::VectorXd &x) override;
        Eigen::VectorXd q_d_prev, q_p_d_prev;
        bool init = true;
        ~InverseDynamicsController() override = default;
    };

    // Instances of nested classes
    CTController ct_controller;                       // Instance of CTController
    PDPlusController pd_plus_controller;              // Instance of PDPlusController
    InverseDynamicsController inverse_dyn_controller; // Instance of InverseDynamicsController
    BaseWorkspaceController *active_controller=0;     // I do not need a smart pointer because I store all instances in the class.

    ErrorFlag error_flag = ErrorFlag::NO_ERROR;
    Eigen::VectorXd tau_full_prev = Eigen::VectorXd::Zero(nq);
    double tau_max_jump = 5.0; // Maximum jump in torque (Nm/dt)
    void error_check(Eigen::VectorXd &tau_full);
};