#ifndef PINOCCHIO_UTILS_HPP
#define PINOCCHIO_UTILS_HPP

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Dense>
#include <vector>
#include <utility> // For std::pair

namespace pinocchio_utils
{

    // Function to calculate n-DOF torque with feedforward
    Eigen::VectorXd calculateNdofTorqueWithFeedforward(const Eigen::VectorXd &u,
                                                       const Eigen::VectorXd &x_k_ndof,
                                                       const pinocchio::Model &robot_model_full,
                                                       pinocchio::Data &robot_data_full,
                                                       int n,
                                                       const Eigen::Map<Eigen::VectorXi> &n_indices,
                                                       bool kin_model);

    // Function for PD control
    Eigen::VectorXd applyPDControl(const Eigen::VectorXd &q_fixed,
                                   const Eigen::VectorXd &q_p_fixed,
                                   const Eigen::VectorXd &q_0_ref_fixed,
                                   const Eigen::MatrixXd &K_d_fixed,
                                   const Eigen::MatrixXd &D_d_fixed);

    void initRobot(const std::string &urdf_filename, pinocchio::Model &robot_model, pinocchio::Data &robot_data, bool use_gravity);

} // namespace pinocchio_utils

#endif // PINOCCHIO_UTILS_HPP