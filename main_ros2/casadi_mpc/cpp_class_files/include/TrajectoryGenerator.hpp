#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include "param_robot.h"
#include "eigen_templates.hpp"
#include "FullSystemTorqueMapper.hpp"
#include "trajectory_settings.hpp"
#include <vector>
#include <map>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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
//            |   Member: traj_data_transient|       Member: all_traj_data_file, traj_file_data      |
//            |   Length:                    |       Length: robot_config.traj_data_real_len+2000    |
//            |   traj_data_transient_len    |                                                       |
//            |<---------------------------------------------------------------------->|             |
//            |          Total Trajectory without extra samples at end                 |             |
//            |          Member: None                                                  |             |
//            |          Length: traj_data_file_real_len (only for checks and plotting)|             |
//            |<------------------------------------------------------------------------------------>|
//                                                Total Trajectory for MPC
//                                                Member: traj_data_out
//                                                Length: traj_data_out_len;

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(FullSystemTorqueMapper &torque_mapper, double dt);

    void init_trajectory_custom_target(ParamInitTrajectory param_init_traj_poly,
                                       ParamTargetTrajectory param_target,
                                       double T_horizon_max = 2);
    void init_trajectory(uint traj_select, ParamInitTrajectory param_init_traj_poly);
    void init_trajectory(uint traj_select);
    void switch_traj(uint traj_select);

    // Setters
    void setTransientTrajParams(ParamInitTrajectory param_init_traj_poly);

    // Getters
    const Eigen::MatrixXd *get_traj_data() const { return &traj_data_out; }
    const Eigen::VectorXd *get_traj_x0_init() const { return &traj_data_out_x0_init; }
    const Eigen::MatrixXd *get_transient_traj_data() const { return &traj_data_transient; }
    const Eigen::VectorXd *get_act_traj_x0_init() const { return &traj_data_file_x0_init; }
    const Eigen::VectorXd *get_traj_file_x0_init(uint traj_select) const
    {
        if (traj_select < 1 || traj_select > all_traj_data_x0_init.size())
        {
            std::cerr << "Invalid trajectory selection. Selecting Trajectory 1" << std::endl;
            traj_select = 1;
        }
        return &all_traj_data_x0_init[traj_select - 1];
    }
    int get_traj_data_len() const
    {
        return traj_data_out_len;
        ;
    }
    int get_traj_file_real_len() const { return robot_config.traj_data_real_len; }
    int get_traj_data_real_len() const { return traj_data_file_real_len; }
    int get_transient_traj_len() const { return traj_data_transient_len; }

    // Setters

private:
    Eigen::Vector4d trajectory_poly(double t, const Eigen::Vector4d &y0, const Eigen::Vector4d &yT, double T);
    Eigen::VectorXd create_poly_traj(double t, const Eigen::Vector3d &rot_ax, double rot_alpha_scale);
    std::vector<Eigen::MatrixXd> readTrajectoryData(const std::string &traj_file);
    std::vector<Eigen::VectorXd> read_x0_init(const std::string &x0_init_file);

    // Member variables
    FullSystemTorqueMapper &torque_mapper;
    robot_config_t &robot_config;
    int nq, nx, nq_red, nx_red;
    ParamInitTrajectory param_init_traj_poly;
    ParamTargetTrajectory param_target_traj_poly;
    std::string traj_file, x0_init_file;

    std::vector<Eigen::MatrixXd> all_traj_data_file;
    std::vector<Eigen::VectorXd> all_traj_data_x0_init;

    Eigen::MatrixXd traj_data_out;       // Total trajectory data = transient trajectory + trajectory from file
    Eigen::MatrixXd traj_data_transient; // Transient trajectory data
    Eigen::MatrixXd traj_data_file;      // Trajectory data from file

    Eigen::VectorXd traj_data_file_x0_init; // Initial state of the trajectory from file "traj_file" in "x0_init_file"
    Eigen::VectorXd traj_data_out_x0_init;  // True initial state of total trajectory (including transient trajectory)
                                            // - in case of no transient trajectory it is equal to traj_file_x0_init
    int selected_trajectory;
    int traj_rows;
    int traj_data_out_len;
    int traj_data_file_real_len;
    int traj_data_transient_len;
    double dt;

    Eigen::MatrixXd generate_transient_trajectory(double T_horizon_max = 0);
    void init_trajectory(bool custom_traj = false, double T_horizon_max = 0);
};

#endif // TRAJECTORY_GENERATOR_HPP
