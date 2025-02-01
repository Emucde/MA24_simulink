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
//            ^     5th order Polynomial     |          Trajectory from File           |Extra Samples|
//            |                              |                                         |             |
//            |                 |            |                        ________         |             |
//            |                 |            |                 _______        _________|_____________|
// p_target...|...........______|____________|_________________                        |             |
//            |       ___/      |            |                                         |             |
// p_init.....|______/          |            |                                         |             |
//            |                 |            |                                         |             |
//            |-----------------|------------|-----------------------------------------|-------------|--------> Time
//          T_start           T_poly       T_end                               T_end+T_true     T_end+T_true+T_horizon_max
//            |<--------------->|<---------->|<--------------------------------------->|<----------->|
//            |   Polynomial    | Constant   | Trajectory from File without            | Last Prediction Horizon
//            |   transient     | trajectory | extra samples at end                    | Horizon Trajectory
//            |   trajectory    | at end     | Member: None                            | Length: Default: T_horizon_max = 2s
//            |   from current  | of traj    | Length: robot_config.traj_data_real_len | Member: None|    - By using trajectory from
//            |<---------------------------->| <---------------------------------------------------->|      file T_horizon_max is set
//            |  Transient Trajectory        | Full Trajectory from File                             |      in "create_trajectories.m"
//            |  Member: traj_data_transient | Member: all_traj_data_file, traj_file_data            |    - By using custom trajectory
//            |  Length:                     | Length: robot_config.traj_data_real_len+T_horizon_max |      init_custom_trajectory(..)
//            |  traj_data_transient.cols()  | or      traj_data_file_real_len+T_horizon_max         |      T_horizon_max can be
//            |<---------------------------------------------------------------------->|             |      directly set.
//            |     Total Trajectory without extra samples at end                      |             |
//            |     Member: None                                                       |             |
//            |     Length: traj_data_file_real_len (only for checks and plotting)     |             |
//            |<------------------------------------------------------------------------------------>|
//                                                Total Trajectory for MPC
//                                                Member: traj_data_out
//                                                Length: traj_data_out_len;
struct ParamPolyTrajectory {
    double T_start; // T_start must be greater than or equal to 0 (start time of the transient trajectory)
    double T_poly;  // T_start must be less than or equal to T_poly (total time of the polynomial trajectory)
    double T_end;   // T_poly must be less than or equal to T_end (total time of the transient trajectory)
    double T_horizon_max; // Extra samples for the last prediction horizon

    Eigen::VectorXd x_init;  // Initial state of the robot
    Eigen::Vector3d p_init;  // Initial position (Current Pose of measurement)
    Eigen::Matrix3d R_init;  // Initial orientation (Current Pose of measurement)

    Eigen::Vector3d p_target; // Target position (Initial Pose of the Trajectory from file)
    Eigen::Matrix3d R_target; // Target orientation (Initial Pose of the Trajectory from file)
};
#endif // TRAJECTORY_SETTINGS_HPP