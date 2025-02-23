#include "RobotModel.hpp"
#include "eigen_templates.hpp"
#include <pinocchio/spatial/se3.hpp>

// Constructor
RobotModel::RobotModel(const std::string &urdf_filename,
               robot_config_t &robot_config,
                const std::string &general_config_file,
               bool reduced_model)
               : urdf_filename(urdf_filename),
                 general_config_file(general_config_file),
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

        //iterate all joints and lock the ones that are fixed
        // Joint 0 is universe joint, so we add 1 to indices
        std::vector<pinocchio::JointIndex> joints_to_lock;
        for (uint32_t i = 0; i < robot_config.nq_fixed; i++)
        {
            const std::string & joint_name = robot_model_full.names[robot_config.n_indices_fixed[i]+1];
            joints_to_lock.push_back(robot_model_full.getJointId(joint_name));
        }

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

    nlohmann::json general_config = read_config(general_config_file);
    bool use_gravity = get_config_value<bool>(general_config, "use_gravity");
    tcp_frame_name = get_config_value<std::string>(general_config, "tcp_frame_name");

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

nlohmann::json RobotModel::read_config(std::string file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cerr << "Error: Could not open JSON file." << std::endl;
        return {};
    }
    nlohmann::json jsonData;
    file >> jsonData; // Parse JSON file
    file.close();
    return jsonData;
}

// Update state with joint positions and velocities
void RobotModel::updateState(const Eigen::VectorXd &x)
{
    // Set joint positions and velocities
    jointData.q = x.head(nq);
    jointData.q_p = x.tail(nq);

    // Compute Kinematics
    computeKinematics();

    // Compute Dynamics
    computeDynamics();
}

// Compute kinematic parameters
void RobotModel::computeKinematics()
{
    // Update Pinocchio Kinematics
    pinocchio::forwardKinematics(robot_model, robot_data, jointData.q, jointData.q_p);
    pinocchio::updateFramePlacements(robot_model, robot_data);

    // Compute the Jacobian
    pinocchio::computeJointJacobians(robot_model, robot_data, jointData.q);

    kinematicsData.J = pinocchio::getFrameJacobian(robot_model, robot_data, tcp_frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

    Eigen::MatrixXd dJ(6, nq);
    pinocchio::getFrameJacobianTimeVariation(robot_model, robot_data, tcp_frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ);
    kinematicsData.J_p = dJ;
    kinematicsData.H = robot_data.oMf[tcp_frame_id].toHomogeneousMatrix();

    kinematicsData.R = kinematicsData.H.topLeftCorner<3, 3>();
    kinematicsData.p = kinematicsData.H.topRightCorner<3, 1>();
    Eigen::Quaterniond quat(kinematicsData.R);
    kinematicsData.quat = quat;
}

// Compute dynamic parameters
void RobotModel::computeDynamics()
{
     // Compute upper triangular part of M using CRBA
    pinocchio::crba(robot_model, robot_data, jointData.q);
    robot_data.M.triangularView<Eigen::StrictlyLower>() = robot_data.M.transpose().triangularView<Eigen::StrictlyLower>();
    dynamicsData.M = robot_data.M;

    // Compute Coriolis forces
    dynamicsData.C = pinocchio::computeCoriolisMatrix(robot_model, robot_data, jointData.q, jointData.q_p);

    // Compute gravitational effects
    dynamicsData.C_rnea = pinocchio::rnea(robot_model, robot_data, jointData.q, jointData.q_p, Eigen::VectorXd::Zero(nq));

    // Get gravitational forces (assuming a gravity vector)
    dynamicsData.g = pinocchio::computeGeneralizedGravity(robot_model, robot_data, jointData.q);
    // equivalent to pinocchio::rnea(robot_model, robot_data, q, Eigen::VectorXd::Zero(nq), Eigen::VectorXd::Zero(nq));
}