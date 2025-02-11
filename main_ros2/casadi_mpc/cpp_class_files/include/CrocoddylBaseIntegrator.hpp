#ifndef CROCODDYL_BASE_INTEGRATOR_HPP
#define CROCODDYL_BASE_INTEGRATOR_HPP

#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/integrator/rk.hpp> // Include the RK Integrator
#include <iostream>
#include <boost/shared_ptr.hpp>

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

#endif // CROCODDYL_BASE_INTEGRATOR_HPP