#include "CasadiMPC.hpp"
#include <cstring> // for memcpy
#include <iostream>
#include <fstream>
#include <vector>
#include <memory> // for std::make_unique
#include "eigen_templates.hpp"

// #define DEBUG 1

mpc_config_t const* invalid_config(const std::string &mpc_name)
{
    throw std::runtime_error("\"" + mpc_name + "\" is not a valid MPC name. Use one of the following: MPC01, MPC6, MPC7, MPC8, MPC9, MPC10, MPC11, MPC12, MPC13, MPC14");
    return nullptr;
}

// Constructor implementation
CasadiMPC::CasadiMPC(const std::string &mpc_name,
                     robot_config_t &robot_config,
                     const Eigen::MatrixXd* traj_data,
                     const casadi_uint traj_data_real_len) : mpc_name(mpc_name),
                                                     mpc_config(mpc_name == "MPC01" ? get_MPC01_config() : mpc_name == "MPC6" ? get_MPC6_config()
                                                                                                       : mpc_name == "MPC7"   ? get_MPC7_config()
                                                                                                       : mpc_name == "MPC8"   ? get_MPC8_config()
                                                                                                       : mpc_name == "MPC9"   ? get_MPC9_config()
                                                                                                       : mpc_name == "MPC10"  ? get_MPC10_config()
                                                                                                       : mpc_name == "MPC11"  ? get_MPC11_config()
                                                                                                       : mpc_name == "MPC12"  ? get_MPC12_config()
                                                                                                       : mpc_name == "MPC13"  ? get_MPC13_config()
                                                                                                       : mpc_name == "MPC14"  ? get_MPC14_config()
                                                                                                                              : invalid_config(mpc_name)),
                                                     robot_config(robot_config),
                                                     traj_data(traj_data),
                                                     traj_data_real_len(traj_data_real_len),
                                                     is_kinematic_mpc(mpc_config->kinematic_mpc == 1),
                                                     nq(robot_config.nq), nx(robot_config.nx), nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
                                                     casadi_fun(mpc_config->casadi_fun),
                                                     arg(mpc_config->arg), res(mpc_config->res), iw(mpc_config->iw), w(mpc_config->w),
                                                     u_opt(w + mpc_config->u_opt_addr), w_end(w + mpc_config->w_end_addr),
                                                     in_init_guess(w + mpc_config->in_init_guess_addr),
                                                     out_init_guess(w + mpc_config->out_init_guess_addr),
                                                     x_out(w + mpc_config->output_config.x_out_addr),
                                                     u_out(w + mpc_config->output_config.u_out_addr),
                                                     x_k(w + mpc_config->x_k_addr),
                                                     y_d(w + mpc_config->y_d_addr),
                                                     x_prev(w + mpc_config->input_config.x_prev_addr),
                                                     u_prev(w + mpc_config->input_config.u_prev_addr),
                                                     param_weight(w + mpc_config->in_param_weight_addr),
                                                     horizon_len(mpc_config->traj_data_per_horizon),
                                                     mpc_traj_indices(ConstIntVectorMap(mpc_config->traj_indices, horizon_len)),
                                                     init_guess_len(mpc_config->init_guess_len),
                                                     x_prev_len(mpc_config->input_config.x_prev_len),
                                                     u_prev_len(mpc_config->input_config.u_prev_len),
                                                     traj_data_per_horizon(mpc_config->traj_data_per_horizon),
                                                     traj_count(0), traj_select(1),
                                                     mem(mpc_config->mem), dt(mpc_config->dt)
{
    // Check if the configuration is valid
    if (mpc_config->casadi_fun == nullptr)
    {
        throw std::runtime_error(mpc_name + "is not a valid MPC name. Use one of the following: MPC01, MPC6, MPC7, MPC8, MPC9, MPC10, MPC11, MPC12, MPC13, MPC14");
    }

    // Initialize arg with pointers based on provided indices
    for (casadi_uint i = 0; i < mpc_config->arg_in_len; i++)
    {
        arg[i] = w + mpc_config->arg_indices[i]; // Use each index from arg_indices
    }

    // Initialize res with pointers based on provided indices
    for (casadi_uint i = 0; i < mpc_config->res_out_len; i++)
    {
        res[i] = w + mpc_config->res_indices[i]; // Use each index from res_indices
    }

    // Check trajectory pointer
    if (traj_data == nullptr)
    {
        throw std::runtime_error("Trajectory data is not provided.");
    }
    traj_rows = traj_data->rows();
    traj_cols = traj_data->cols();

    // check trajectory lengths
    if (traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1] > traj_cols)
    {
        throw std::runtime_error("Trajectory data length + mpc_traj_indices[end] = " +
                                 std::to_string(traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1]) +
                                 " exceeds total length = " +
                                 std::to_string(traj_cols) +
                                 ". Please generate new.");
    }

    // Copy the parameter weights
    memcpy(param_weight, mpc_config->param_weight, mpc_config->param_weight_len * sizeof(casadi_real));

#ifdef DEBUG
    std::cout << "w: ";
    for (int i = 0; i < mpc_config->u_opt_addr; i++)
    {
        std::cout << w[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "MPC object created." << std::endl;
#endif
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PUBLIC METHODS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////                                                                             /////
/////                ||||||   ||   ||  ||||    ||      ||    |||||                /////
/////                ||   ||  ||   ||  ||  ||  ||      ||  ||                     /////
/////                ||||||   ||   ||  ||||||  ||      ||  ||                     /////
/////                ||       ||   ||  ||  ||  ||      ||  ||                     /////
/////                ||        |||||   ||||    ||||||  ||    |||||                /////
/////                                                                             /////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

// Method to run the MPC
int CasadiMPC::solve(casadi_real *x_k_in)
{
    // Copy the state to the MPC object

    set_references(x_k_in);

    #ifdef DEBUG
    std::cout << "int CasadiMPC::solve()\nw:\n";
    for (int i = 0; i < mpc_config->u_opt_addr; i++)
    {
        std::cout << w[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "MPC object created." << std::endl << std::endl;
    #endif
    
    // Call the Casadi function
    int flag = casadi_fun(arg, res, iw, w_end, mem);

    // Set initial guess and prev ref values
    if (!flag)
    {
        memcpy(x_prev, x_out, x_prev_len * sizeof(casadi_real));
        memcpy(u_prev, u_out, u_prev_len * sizeof(casadi_real));
        memcpy(in_init_guess, out_init_guess, init_guess_len * sizeof(casadi_real));
    }
    else
    {
        std::cerr << "Error in Casadi function call." << std::endl;
    }

    return flag;
}

// int CasadiMPC::solve(casadi_real *x_k_in)
// {
//     // Copy the initial state
//     memcpy(x_k, x_k_in, nx_red * sizeof(casadi_real));

//     return CasadiMPC::solve();
// }

int CasadiMPC::solve_planner()
{
    // Call the Casadi function
    int flag = casadi_fun(arg, res, iw, w_end, mem);

    // Read the next trajectory block
    if (!flag)
    {
        // read_trajectory_block();
        memcpy(in_init_guess, out_init_guess, init_guess_len * sizeof(casadi_real));
        
        // mpc planner: use x_k+1 as x_k for next iteration - not the measurement!
        set_references(w + mpc_config->output_config.x_out_addr + nx_red);
        // memcpy(x_k, w + mpc_config->output_config.x_out_addr + nx_red, nx_red * sizeof(casadi_real));
#ifdef DEBUG
        if (traj_count % 100 == 0)
        {
            std::cout << "x_k: " << x_k[0] << " " << x_k[1] << " " << x_k[2] << " " << x_k[3] << " " << x_k[4] << " " << x_k[5] << " " << x_k[6] << std::endl;

            for (int i = 0; i < 24; i++)
            {
                std::cout << u_opt[i] << " ";
            }
            std::cout << std::endl;
        }
#endif
    }
    else
    {
        std::cerr << "Error in Casadi function call." << std::endl;
    }

    return flag;
}

// Method for switching the trajectory
void CasadiMPC::switch_traj(const Eigen::MatrixXd* traj_data_new, const casadi_real *const x_k_ptr, casadi_uint traj_data_real_len_new)
{
    traj_data = traj_data_new;
    traj_rows = traj_data->rows();
    traj_cols = traj_data->cols();

    traj_data_real_len = traj_data_real_len_new;

    if (traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1] > traj_cols)
    {
        throw std::runtime_error("Trajectory data length + mpc_traj_indices[end] = " +
                                 std::to_string(traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1]) +
                                 " exceeds total length = " +
                                 std::to_string(traj_cols) +
                                 ". Please generate new.");
    }

    set_coldstart_init_guess(x_k_ptr);

    traj_count = 0;
}

// Method to set the cold start initial guess (assuming x_k was already set)
void CasadiMPC::set_coldstart_init_guess(const casadi_real *const x_k_ptr)
{
    set_x_k(x_k_ptr);

    memset(in_init_guess, 0, init_guess_len * sizeof(double));

    // set all x input values to the current state
    set_row_vector(mpc_config->input_config.x_addr, x_k, nx_red, mpc_config->input_config.x_len);

    // set all x_prev reference values to the current state
    set_row_vector(mpc_config->input_config.x_prev_addr, x_k, nx_red, mpc_config->input_config.x_prev_len);
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// PUBLIC GETTER METHODS //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PUBLIC SETTER METHODS ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PRIVATE METHODS ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////                                                                             /////
/////          ||||||   ||||||   ||  ||    ||   ||||    ||||||||  ||||||          /////
/////          ||   ||  ||   ||  ||  ||    ||  ||   ||     ||     ||              /////
/////          ||||||   ||||||   ||   ||  ||   |||||||     ||     ||||||          /////
/////          ||       || ||    ||    ||||    ||   ||     ||     ||              /////
/////          ||       ||   ||  ||     ||     ||   ||     ||     ||||||          /////
/////                                                                             /////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// PRIVATE GETTER METHODS /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// PRIVATE SETTER METHODS /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

// Method for setting all rows of matrix_data (rows x cols) to the (rows x 1) vector row_data. (length = rows * cols)
void CasadiMPC::set_row_vector(casadi_uint local_address, casadi_real *row_data, casadi_uint rows, casadi_uint length)
{
    casadi_uint cols = length / rows;
    casadi_real *matrix_data = w + local_address;
    for (casadi_uint i = 0; i < cols; i++)
    {
        memcpy(matrix_data + i * rows, row_data, rows * sizeof(casadi_real));
    }
}

void CasadiMPC::set_references(casadi_real *x_k_in)
{
    set_x_k(x_k_in); // set x_k to the reference pose
    if (traj_count < traj_data_real_len - 1)
    {
        for (casadi_uint j = 0; j < traj_data_per_horizon; j++)
        {
            memcpy(y_d + j * traj_rows,
                   traj_data->col(traj_count + mpc_traj_indices[j]).data(),
                   traj_rows * sizeof(double));
        }
        
        // this loop would be replaceable by the following line (1% slower)
        // Eigen::Map<Eigen::MatrixXd> (y_d, traj_rows, traj_data_per_horizon) = (*traj_data)(Eigen::all, mpc_traj_indices.array() + traj_count);
        traj_count++;
    }
}

// Destructor to clean up allocated memory
CasadiMPC::~CasadiMPC()
{
}