#ifndef CASADIEKF_HPP
#define CASADIEKF_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

#include "param_robot.h"
#include "json.hpp"
#include "fr3_ekf.h"
#include "fr3_ekf_addressdef.h"
#include "eigen_templates.hpp"

/*
This class implements an Extended Kalman Filter (EKF) using the CasADi library.
It is designed to work with a specific robot configuration and uses JSON for configuration management.
The EKF is initialized with a configuration file that defines the system parameters, noise characteristics, and
measurement models.
The class provides methods for predicting the next state based on control inputs and updating the state based on
measurements.
*/
class CasadiEKF
{
public:
    CasadiEKF(std::string ekf_config_path);

    void predict(const double* u_k, const double* y_k);
    void update_config();
    void initialize(double *x_k_ptr);

    template <typename T = Eigen::MatrixXd,
              typename std::enable_if<std::is_same<T, Eigen::MatrixXd>::value, int>::type = 0>
    Eigen::MatrixXd get_param_matrix(const std::string &name)
    {
        std::vector<std::vector<double>> matrix_data = get_config_value<nlohmann::json>(ekf_settings, name).get<std::vector<std::vector<double>>>();
        Eigen::MatrixXd eigen_matrix(matrix_data.size(), matrix_data[0].size());
        for (size_t i = 0; i < matrix_data.size(); ++i)
        {
            eigen_matrix.row(i) = Eigen::VectorXd::Map(matrix_data[i].data(), matrix_data[i].size());
        }
        return eigen_matrix;
    }

    double *get_x_k_plus_ptr()
    {
        return w+FR3_EKF_XK_PLUS_ADDR;
    }

private:
    const int nq = PARAM_ROBOT_N_DOF;
    const int nx = 2 * PARAM_ROBOT_N_DOF;
    int mem = 0;

    std::string ekf_config_path;
    nlohmann::json ekf_settings;

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(nx, nx);
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(nx, nx);
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Zero(nx, nx);

    const casadi_real *arg[FR3_EKF_ARG_LEN];
    casadi_real *res[FR3_EKF_RES_LEN];
    casadi_int *iw = 0;
    casadi_real w[FR3_EKF_W_LEN];

    void update();
};

#endif // CASADIEKF_HPP