#ifndef CROCODDYL_MPC_HPP
#define CROCODDYL_MPC_HPP

#include "param_robot.h"
#include "robot_data.hpp"
#include <iostream>
#include "json.hpp"

#include <crocoddyl/core/solvers/ddp.hpp>
#include <crocoddyl/core/solvers/box-ddp.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/integrator/rk.hpp> // Include the RK Integrator
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/activations/weighted-quadratic-barrier.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <boost/shared_ptr.hpp>

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

class BaseCrocoddylIntegrator
{
public:
    BaseCrocoddylIntegrator(const std::string &int_type) : int_type(int_type) {};

    // Pure virtual function
    virtual boost::shared_ptr<crocoddyl::IntegratedActionModelAbstract> integrate(
        const boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> &DAM, double dt) = 0;
    virtual ~BaseCrocoddylIntegrator() = default;

protected:
    const std::string int_type;
};

class IntegratorEuler : public BaseCrocoddylIntegrator
{
public:
    IntegratorEuler(std::string int_type) : BaseCrocoddylIntegrator(int_type) {}
    boost::shared_ptr<crocoddyl::IntegratedActionModelAbstract> integrate(
        const boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> &DAM, double dt) override
    {
        return boost::make_shared<crocoddyl::IntegratedActionModelEuler>(DAM, dt);
    }
};

class IntegratorRK2 : public BaseCrocoddylIntegrator
{
public:
    IntegratorRK2(std::string int_type) : BaseCrocoddylIntegrator(int_type) {}
    boost::shared_ptr<crocoddyl::IntegratedActionModelAbstract> integrate(
        const boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> &DAM, double dt) override
    {
        return boost::make_shared<crocoddyl::IntegratedActionModelRK>(DAM, crocoddyl::RKType(2), dt);
    }
};

// Example class for Runge-Kutta 3rd order integration
class IntegratorRK3 : public BaseCrocoddylIntegrator
{
public:
    IntegratorRK3(std::string int_type) : BaseCrocoddylIntegrator(int_type) {}
    boost::shared_ptr<crocoddyl::IntegratedActionModelAbstract> integrate(
        const boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> &DAM, double dt) override
    {
        return boost::make_shared<crocoddyl::IntegratedActionModelRK>(DAM, crocoddyl::RKType(3), dt);
    }
};

// Example class for Runge-Kutta 4th order integration
class IntegratorRK4 : public BaseCrocoddylIntegrator
{
public:
    IntegratorRK4(std::string int_type) : BaseCrocoddylIntegrator(int_type) {}
    boost::shared_ptr<crocoddyl::IntegratedActionModelAbstract> integrate(
        const boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> &DAM, double dt) override
    {
        return boost::make_shared<crocoddyl::IntegratedActionModelRK>(DAM, crocoddyl::RKType(4), dt);
    }
};







class BaseCrocoddylMPC
{
public:
    BaseCrocoddylMPC(RobotModel &robot_model,
                            const std::string &crocoddyl_config_path,
                            const std::string &tcp_frame_name,
                            TrajectoryGenerator &trajectory_generator)
        : robot_model(robot_model),
          crocoddyl_config_path(crocoddyl_config_path),
          tcp_frame_name(tcp_frame_name),
          trajectory_generator(trajectory_generator),
          nq(robot_model.nq), nx(robot_model.nx),
          dt(robot_model.robot_config.dt), traj_count(0)
    {
    }
    template <typename T>
    T json2value(const std::string &name)
    {
        return param_mpc_weight[name].get<T>();
    }

    template <typename T = Eigen::VectorXd,
              typename std::enable_if<std::is_same<T, Eigen::VectorXd>::value, int>::type = 0>
    Eigen::VectorXd json2vec(const std::string &name)
    {
        std::vector<double> vector_data = param_mpc_weight[name].get<std::vector<double>>();
        Eigen::VectorXd eigen_vector = Eigen::Map<Eigen::VectorXd>(vector_data.data(), vector_data.size());
        return eigen_vector;
    }

    virtual std::shared_ptr<BaseCrocoddylIntegrator> create_integrator(const std::string &int_type);
    virtual void create_mpc_solver() = 0; // Pure virtual function
    virtual void set_references(casadi_real *x_k_in) = 0; // Pure virtual function
    virtual void solve(const Eigen::VectorXd &x);                 // Common method in BaseCrocoddylMPC
    virtual void increase_traj_count() { traj_count++; }
    virtual void read_config_file();
    virtual ~BaseCrocoddylMPC() = default; // Virtual destructor
protected:
    RobotModel &robot_model;
    const std::string crocoddyl_config_path;
    const std::string tcp_frame_name;
    nlohmann::json mpc_settings;
    nlohmann::json param_mpc_weight;
    TrajectoryGenerator &trajectory_generator;
    const int nq, nx; // this is nq_red and nx_red!!!
    double dt;
    boost::shared_ptr<crocoddyl::SolverAbstract> ddp;

public:
    uint traj_count;
};

class ClassicDynMPC : public BaseCrocoddylMPC
{
public:
    ClassicDynMPC(RobotModel &robot_model,
                    const std::string &crocoddyl_config_path,
                    const std::string &tcp_frame_name,
                    TrajectoryGenerator &trajectory_generator)
        : BaseCrocoddylMPC(robot_model, crocoddyl_config_path, tcp_frame_name, trajectory_generator)
    {
        read_config_file();
        create_mpc_solver();
    }

public:
    void create_mpc_solver() override;
    void set_references(casadi_real *x_k_in) override;
};

#endif // CROCODDYL_MPC_HPP