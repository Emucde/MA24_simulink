/*
This file defines the base class for Crocoddyl integrators and specific implementations
for Euler and Runge-Kutta methods. The base class is abstract and requires derived classes to implement the integrate method.
*/

#ifndef CROCODDYL_BASE_INTEGRATOR_HPP
#define CROCODDYL_BASE_INTEGRATOR_HPP

#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/integrator/rk.hpp> // Include the RK Integrator
#include <iostream>
#include <memory>

/*
This class serves as a base for different integrators in the Crocoddyl framework.
*/
class BaseCrocoddylIntegrator
{
public:
    BaseCrocoddylIntegrator(const std::string &int_type) : int_type(int_type) {};

    // Pure virtual function
    virtual std::shared_ptr<crocoddyl::IntegratedActionModelAbstract> integrate(
        const std::shared_ptr<crocoddyl::DifferentialActionModelAbstract> &DAM, double dt) = 0;
    virtual ~BaseCrocoddylIntegrator() = default;

protected:
    const std::string int_type;
};

/*
This class implements the Euler integrator for the Crocoddyl framework.
It inherits from BaseCrocoddylIntegrator and implements the integrate method.
It uses the IntegratedActionModelEulerTpl class to perform the integration.
*/
class IntegratorEuler : public BaseCrocoddylIntegrator
{
public:
    IntegratorEuler(std::string int_type) : BaseCrocoddylIntegrator(int_type) {}
    std::shared_ptr<crocoddyl::IntegratedActionModelAbstract> integrate(
        const std::shared_ptr<crocoddyl::DifferentialActionModelAbstract> &DAM, double dt) override
    {
        return std::make_shared<crocoddyl::IntegratedActionModelEuler>(DAM, dt);
    }
};

/*
This class implements the Runge-Kutta 2nd order integrator for the Crocoddyl framework.
It inherits from BaseCrocoddylIntegrator and implements the integrate method.
It uses the IntegratedActionModelRKTpl class to perform the integration.
*/
class IntegratorRK2 : public BaseCrocoddylIntegrator
{
public:
    IntegratorRK2(std::string int_type) : BaseCrocoddylIntegrator(int_type) {}
    std::shared_ptr<crocoddyl::IntegratedActionModelAbstract> integrate(
        const std::shared_ptr<crocoddyl::DifferentialActionModelAbstract> &DAM, double dt) override
    {
        return std::make_shared<crocoddyl::IntegratedActionModelRK>(DAM, crocoddyl::RKType(2), dt);
    }
};

/*
This class implements the Runge-Kutta 3rd order integrator for the Crocoddyl framework.
It inherits from BaseCrocoddylIntegrator and implements the integrate method.
It uses the IntegratedActionModelRKTpl class to perform the integration.
*/
class IntegratorRK3 : public BaseCrocoddylIntegrator
{
public:
    IntegratorRK3(std::string int_type) : BaseCrocoddylIntegrator(int_type) {}
    std::shared_ptr<crocoddyl::IntegratedActionModelAbstract> integrate(
        const std::shared_ptr<crocoddyl::DifferentialActionModelAbstract> &DAM, double dt) override
    {
        return std::make_shared<crocoddyl::IntegratedActionModelRK>(DAM, crocoddyl::RKType(3), dt);
    }
};

/*
This class implements the Runge-Kutta 4th order integrator for the Crocoddyl framework.
It inherits from BaseCrocoddylIntegrator and implements the integrate method.
It uses the IntegratedActionModelRKTpl class to perform the integration.
*/
class IntegratorRK4 : public BaseCrocoddylIntegrator
{
public:
    IntegratorRK4(std::string int_type) : BaseCrocoddylIntegrator(int_type) {}
    std::shared_ptr<crocoddyl::IntegratedActionModelAbstract> integrate(
        const std::shared_ptr<crocoddyl::DifferentialActionModelAbstract> &DAM, double dt) override
    {
        return std::make_shared<crocoddyl::IntegratedActionModelRK>(DAM, crocoddyl::RKType(4), dt);
    }
};

#endif // CROCODDYL_BASE_INTEGRATOR_HPP