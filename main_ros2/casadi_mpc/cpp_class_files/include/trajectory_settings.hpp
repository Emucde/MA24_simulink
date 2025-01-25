#ifndef TRAJECTORY_SETTINGS_HPP
#define TRAJECTORY_SETTINGS_HPP

#include <Eigen/Dense>

// This struct defines the settings for a polynomial Trajectory that generates a Trajectory
// from the current pose of the robot to the initial pose of the trajectory to avoid jumps in torque

// Example for possible Trajectory (only 1D Position over time shown here)
// p_init ... Initial Position (Current Pose of measurement)
// p_d    ... Target Position (Initial Pose of the Trajectory from file)
// 
//        Position
//            ^
//            |                 |            |                        ________         |             |
//            |                 |            |                 _______        _________|_____________|
// p_d........|           ______|____________|_________________                        |             |
//            |       ___/      |            |                                         |             |
// p_init.....|______/          |            |                                         |             |
//            |                 |            |                                         |             |
//            |-----------------|------------|-----------------------------------------|-------------|--------> Time
//          T_start           T_poly       T_end                               T_end+T_true     T_end+T_true+T_max_horizon_length
//            |<--------------->|<---------->|<--------------------------------------->|<----------->|
//            |    Polynomial   |  Constant  | Trajectory from File without            | Last Prediction Horizon
//            |    transient    |  trajectory| extra samples at end                    | Horizon Trajectory
//            |    trajectory   |  at end    | Member: None                            | Length: Default: 2000 (set in 'create_trajectories.m' 
//            |    from current |  of traj   | Length: robot_config.traj_data_real_len | Member: None|          with Variable T_horizon_max
//            |<---------------------------->| <---------------------------------------------------->|          (e.g. T_horizon_max = 2s means
//            |   Transient Trajectory       |       Full Trajectory from File                       |           2000 samples at Ts=1ms))
//            |   Member: transient_traj_data|       Member: all_traj_data[selected_trajectory]      |
//            |   Length: transient_traj_len |       Length: robot_config.traj_data_real_len+2000    |
//            |                              |                                                       |
//            |<---------------------------------------------------------------------->|             |
//            |          Total Trajectory without extra samples at end                 |             |
//            |          Member: None                                                  |             |
//            |          Length: traj_real_len (only for checks and plotting needed)   |             |
//            |<------------------------------------------------------------------------------------>|
//                                                Total Trajectory for MPC
//                                                Member: traj_data
//                                                Length: traj_len

struct ParamInitTrajectory {
    double T_start; // T_start must be greater than or equal to 0 (start time of the transient trajectory)
    double T_poly;  // T_start must be less than or equal to T_poly (total time of the polynomial trajectory)
    double T_end;   // T_poly must be less than or equal to T_end (total time of the transient trajectory)

    Eigen::VectorXd x_init;  // Initial state of the robot
    Eigen::Vector3d p_init;  // Initial position (Current Pose of measurement)
    Eigen::Matrix3d R_init;  // Initial orientation (Current Pose of measurement)
};

// struct ParamInitTrajectory {
//     Eigen::VectorXd x_init;  // Initial state of the robot
//     Eigen::Vector3d p_init; // Target position (Initial Pose of the Trajectory from file)
//     Eigen::Matrix3d R_init; // Target orientation (Initial Pose of the Trajectory from file)
// };

struct ParamTargetTrajectory {
    Eigen::Vector3d p_target; // Target position (Initial Pose of the Trajectory from file)
    Eigen::Matrix3d R_target; // Target orientation (Initial Pose of the Trajectory from file)
};

#endif // TRAJECTORY_SETTINGS_HPP