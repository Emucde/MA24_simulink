#include "RobotModel.hpp"
#include "eigen_templates.hpp"
#include <pinocchio/spatial/se3.hpp>

// Constructor
RobotModel::RobotModel(const std::string &urdf_filename,
               const std::string &tcp_frame_name,
               robot_config_t &robot_config,
               bool use_gravity,
               bool reduced_model)
               : urdf_filename(urdf_filename),
                 tcp_frame_name(tcp_frame_name),
                 robot_config(robot_config)
{
    pinocchio::Model robot_model_full;

    // Load the URDF file into robot_model_full
    pinocchio::urdf::buildModel(urdf_filename, robot_model_full);

    if (reduced_model)
    {
        nx = robot_config.nx_red;
        nq = robot_config.nq_red;
        const Eigen::VectorXd q_ref_nq = ConstDoubleVectorMap(robot_config.q_0_ref, robot_config.nq);
        const Eigen::VectorXi joint_id_fixed = ConstIntVectorMap(robot_config.n_indices_fixed, robot_config.nq_fixed) + Eigen::VectorXi::Ones(robot_config.nq_fixed);
        const std::vector<pinocchio::JointIndex> joints_to_lock(robot_config.n_indices_fixed, robot_config.n_indices_fixed + robot_config.nq_fixed);
        
        robot_model = pinocchio::buildReducedModel(robot_model_full, joints_to_lock, q_ref_nq);
    }
    else
    {
        nx = robot_config.nx;
        nq = robot_config.nq;
        robot_model = robot_model_full;
    }

    // Initialize the corresponding robot_data structure
    robot_data = pinocchio::Data(robot_model);

    if (use_gravity)
    {
        robot_model.gravity.linear() << 0.0, 0.0, -9.81;
    }
    else
    {
        robot_model.gravity.linear() << 0.0, 0.0, 0.0;
    }

    // Initialize the TCP frame Id
    tcp_frame_id = robot_model.getFrameId(tcp_frame_name);
}

// Update state with joint positions and velocities
void RobotModel::updateState(const Eigen::VectorXd &x)
{
    // Set joint positions and velocities
    jointData.q = x.head(nq);
    jointData.q_p = x.tail(nq);

    // Update Pinocchio robot_data
    pinocchio::forwardKinematics(robot_model, robot_data, jointData.q, jointData.q_p);
    pinocchio::updateFramePlacements(robot_model, robot_data);

    // Compute Kinematics
    computeKinematics();

    // Compute Dynamics
    computeDynamics();
}

// Compute kinematic parameters
void RobotModel::computeKinematics()
{
    // Compute the Jacobian
    pinocchio::computeJointJacobians(robot_model, robot_data, jointData.q);

    kinematicsData.J = pinocchio::getFrameJacobian(robot_model, robot_data, tcp_frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

    Eigen::MatrixXd dJ(6, nq);
    pinocchio::getFrameJacobianTimeVariation(robot_model, robot_data, tcp_frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ);
    kinematicsData.J_p = dJ;
    kinematicsData.H = robot_data.oMf[tcp_frame_id].toHomogeneousMatrix();

    Eigen::Quaterniond quat(kinematicsData.R);

    kinematicsData.R = kinematicsData.H.topLeftCorner<3, 3>();
    kinematicsData.p = kinematicsData.H.topRightCorner<3, 1>();
    kinematicsData.quat = quat;
}

// Compute dynamic parameters
void RobotModel::computeDynamics()
{
    // Compute Mass Matrix
    dynamicsData.M = pinocchio::crba(robot_model, robot_data, jointData.q);

    // Compute Coriolis forces
    dynamicsData.C = pinocchio::computeCoriolisMatrix(robot_model, robot_data, jointData.q, jointData.q_p);

    // Compute gravitational effects
    dynamicsData.C_rnea = pinocchio::rnea(robot_model, robot_data, jointData.q, jointData.q_p, Eigen::VectorXd::Zero(nq));

    // Get gravitational forces (assuming a gravity vector)
    dynamicsData.g = pinocchio::computeGeneralizedGravity(robot_model, robot_data, jointData.q);
    // equivalent to inocchio::rnea(robot_model, robot_data, q, Eigen::VectorXd::Zero(nq), Eigen::VectorXd::Zero(nq));
}