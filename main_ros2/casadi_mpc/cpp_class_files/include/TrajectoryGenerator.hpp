#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP
#include "param_robot.h"
#include "eigen_templates.hpp"
#include "RobotModel.hpp"
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

/*
This class generates trajectories for the robot model.
It can create trajectories based on polynomial functions or read them from files.
It supports both transient trajectories and full trajectories from files.
*/
class TrajectoryGenerator
{
public:
    TrajectoryGenerator(RobotModel &robot_model,
                        double dt,
                        const std::string &general_config_filename);

    Eigen::MatrixXd p_d;       // Target position
    Eigen::MatrixXd p_d_p;     // Target velocity
    Eigen::MatrixXd p_d_pp;    // Target acceleration
    Eigen::MatrixXd q_d;       // Target quaternion
    Eigen::MatrixXd omega_d;   // Target angular velocity
    Eigen::MatrixXd omega_d_p; // Target angular acceleration
    Eigen::MatrixXd phi_d;     // Target rpy orientation
    Eigen::MatrixXd phi_d_p;   // Target rpy orientation velocity
    Eigen::MatrixXd phi_d_pp;  // Target rpy orientation acceleration

    const Eigen::Vector3i p_d_rows{0, 1, 2};
    const Eigen::Vector3i p_d_p_rows{3, 4, 5};
    const Eigen::Vector3i p_d_pp_rows{6, 7, 8};
    const Eigen::Vector4i q_d_rows{9, 10, 11, 12};
    const Eigen::Vector3i omega_d_rows{13, 14, 15};
    const Eigen::Vector3i omega_d_p_rows{16, 17, 18};
    const Eigen::Vector3i phi_d_rows{19, 20, 21};
    const Eigen::Vector3i phi_d_p_rows{22, 23, 24};
    const Eigen::Vector3i phi_d_pp_rows{25, 26, 27};
    const Eigen::VectorXi pq_d_rows = (Eigen::VectorXi(7) << 0, 1, 2, 9, 10, 11, 12).finished();

    void init_custom_trajectory(ParamPolyTrajectory param);
    void init_file_trajectory(int traj_select, const double *x_k_ndof_ptr, double T_start, double T_poly, double T_end);

    void switch_traj(int traj_select);

    // Setters
    void check_param_poly_traj(ParamPolyTrajectory param);

    // Getters
    // getter for traj select
    int get_traj_select() const { return selected_trajectory; }
    const Eigen::MatrixXd *get_traj_data() const { return &traj_data_out; }
    const Eigen::VectorXd *get_traj_x0_init() const { return &traj_data_out_x0_init; }
    const Eigen::MatrixXd *get_transient_traj_data() const { return &traj_data_transient; }
    const Eigen::VectorXd *get_act_traj_x0_init() const { return &traj_data_file_x0_init; }
    const Eigen::VectorXd *get_traj_file_x0_init(int traj_select) const
    {
        if (traj_select < 1 || traj_select > static_cast<int>(all_traj_data_x0_init.size()))
        {
            std::cerr << "Invalid trajectory selection. Selecting Trajectory 1" << std::endl;
            traj_select = 1;
        }
        return &all_traj_data_x0_init[traj_select - 1];
    }
    int get_traj_data_len() const
    {
        return traj_data_out_len;
    }
    int get_traj_file_real_len() const { return robot_config.traj_data_real_len; }
    int get_traj_data_real_len() const { return traj_data_file_real_len; } // without extra samples for last prediction horizon
    int get_transient_traj_len() const { return traj_data_transient.cols(); }

    double get_dt() const { return dt; }

    // Setters

private:
    std::vector<Eigen::MatrixXd> read_trajectory_data(const std::string &traj_file);
    std::vector<Eigen::VectorXd> read_x0_init(const std::string &x0_init_file);

    void update_traj_values();

    const std::string general_config_filename;

    // Member variables
    RobotModel &robot_model;
    robot_config_t &robot_config;
    int nq, nx, nq_red, nx_red;
    Eigen::VectorXi n_x_indices;
    ParamPolyTrajectory param_poly_traj;
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
    double dt;

    Eigen::VectorXd poly_5th_order(double t, const Eigen::Vector4d &y0, const Eigen::Vector4d &yT, double T);
    Eigen::VectorXd get_poly_traj_point(double t, const Eigen::Vector3d &rot_ax, double rot_alpha_scale);
    Eigen::MatrixXd generate_poly_trajectory();
};

#endif // TRAJECTORY_GENERATOR_HPP
