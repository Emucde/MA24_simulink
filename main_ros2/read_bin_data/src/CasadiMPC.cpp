#include "CasadiMPC.hpp"
#include <cstring> // for memcpy
#include <iostream>
#include <fstream>
#include <vector>
#include <memory> // for std::make_unique

mpc_config_t invalid_config(const std::string &mpc_name)
{
    throw std::runtime_error("\"" + mpc_name + "\" is not a valid MPC name. Use one of the following: MPC01, MPC6, MPC7, MPC8, MPC9, MPC10, MPC11, MPC12, MPC13, MPC14");
    return {};
}

// Constructor implementation
CasadiMPC::CasadiMPC(const std::string &mpc_name,
                     robot_config_t &robot_config) : mpc_name(mpc_name),
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
                                                     nq(robot_config.n_dof), nx(2 * robot_config.n_dof), nq_red(robot_config.n_red), nx_red(2 * robot_config.n_red),
                                                     casadi_fun(mpc_config.casadi_fun),
                                                     arg(mpc_config.arg), res(mpc_config.res), iw(mpc_config.iw), w(mpc_config.w),
                                                     u_opt(w + mpc_config.u_opt_addr), w_end(w + mpc_config.w_end_addr),
                                                     in_init_guess(w + mpc_config.in_init_guess_addr),
                                                     out_init_guess(w + mpc_config.out_init_guess_addr),
                                                     x_k(w + mpc_config.x_k_addr),
                                                     y_d(w + mpc_config.y_d_addr),
                                                     param_weight(w + mpc_config.in_param_weight_addr),
                                                     traj_data_real_len(mpc_config.traj_data_real_len),
                                                     mpc_traj_indices(mpc_config.traj_indices),
                                                     horizon_len(mpc_config.traj_data_per_horizon),
                                                     init_guess_len(mpc_config.init_guess_len),
                                                     traj_file(mpc_config.traj_data_path),
                                                     traj_data_per_horizon(mpc_config.traj_data_per_horizon),
                                                     n_indices(robot_config.n_indices),
                                                     n_x_indices(robot_config.n_x_indices),
                                                     n_indices_fixed(robot_config.n_indices_fixed),
                                                     n_x_indices_fixed(robot_config.n_x_indices_fixed),
                                                     traj_count(0), traj_select(1), mem(mpc_config.mem)
{
    // Check if the configuration is valid
    if (mpc_config.casadi_fun == nullptr)
    {
        throw std::runtime_error(mpc_name + "is not a valid MPC name. Use one of the following: MPC01, MPC6, MPC7, MPC8, MPC9, MPC10, MPC11, MPC12, MPC13, MPC14");
    }

    // Initialize arg with pointers based on provided indices
    for (casadi_uint i = 0; i < mpc_config.arg_in_len; i++)
    {
        arg[i] = w + mpc_config.arg_indices[i]; // Use each index from arg_indices
    }

    // Initialize res with pointers based on provided indices
    for (casadi_uint i = 0; i < mpc_config.res_out_len; i++)
    {
        res[i] = w + mpc_config.res_indices[i]; // Use each index from res_indices
    }

    // read first initial guess from file
    load_initial_guess(mpc_config.init_guess_path, in_init_guess);

    // read first x0 from file
    x_ref_nq.resize(nx);
    read_x0_init(mpc_config.x0_init_path, x_ref_nq.data());

    // Copy the initial state
    int cnt = 0;
    for (casadi_uint i = 0; i < nx; i++)
    {
        if (i == n_x_indices[cnt])
        {
            x_k[cnt] = x_ref_nq[i];
            cnt++;
        }
    }

    // read first trajectory data from file
    traj_data_startbyte = get_traj_dims();
    read_trajectory_block();

    // check trajectory lengths
    if (traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1] > traj_data_total_len)
    {
        throw std::runtime_error("Trajectory data length + mpc_traj_indices[end] = " +
                                 std::to_string(traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1]) +
                                 " exceeds total length = " +
                                 std::to_string(traj_data_total_len) +
                                 ". Please generate new.");
    }

    // Copy the parameter weights
    memcpy(param_weight, mpc_config.param_weight, mpc_config.param_weight_len * sizeof(casadi_real));

#ifdef DEBUG
    std::cout << "w: ";
    for (int i = 0; i < mpc_config.u_opt_addr; i++)
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
    // Copy the initial state
    memcpy(x_k, x_k_in, nx_red * sizeof(casadi_real));

    // Call the Casadi function
    int flag = casadi_fun(arg, res, iw, w_end, mem);

    // Read the next trajectory block
    if (!flag)
    {
        read_trajectory_block();
        memcpy(in_init_guess, out_init_guess, init_guess_len * sizeof(casadi_real));
    }
    else
    {
        std::cerr << "Error in Casadi function call." << std::endl;
    }

    return flag;
}

int CasadiMPC::solve() // only for testing
{
    // Call the Casadi function
    int flag = casadi_fun(arg, res, iw, w_end, mem);

    // Read the next trajectory block
    if (!flag)
    {
        read_trajectory_block();
        memcpy(in_init_guess, out_init_guess, init_guess_len * sizeof(casadi_real));

        // mpc planner: use x_k+1 as x_k for next iteration - not the measurement!
        memcpy(x_k, u_opt + nq_red, nx_red * sizeof(casadi_real));
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

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// SETTER METHODS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void CasadiMPC::set_x0(casadi_real *x0_in)
{
    memcpy(x_k, x0_in, nx_red * sizeof(casadi_real));
}

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

void CasadiMPC::read_file(std::ifstream &file, std::streampos data_start, casadi_real *data, int data_len)
{
    file.seekg(data_start);
    if (!file.good())
    {
        throw std::runtime_error("Error seeking to data start position.");
    }

    file.read(reinterpret_cast<char *>(data), data_len * sizeof(casadi_real));
    if (!file)
    {
        throw std::runtime_error("Error reading data from file.");
    }
}

int CasadiMPC::load_initial_guess(const std::string &init_guess_path, casadi_real *init_guess_data)
{
    std::ifstream init_guess_file(init_guess_path, std::ios::binary);
    if (!init_guess_file.is_open())
    {
        std::cerr << "Error opening init_guess_file" << std::endl;
        return 1;
    }

    unsigned int init_guess_rows, init_guess_cols;

    // Read dimensions
    init_guess_file.read(reinterpret_cast<char *>(&init_guess_rows), sizeof(init_guess_rows));
    init_guess_file.read(reinterpret_cast<char *>(&init_guess_cols), sizeof(init_guess_cols));

    if (!init_guess_file.good())
    {
        std::cerr << "Error reading dimensions from file" << std::endl;
        return 1;
    }

#ifdef DEBUG
    std::cout << "init_guess dimensions: " << init_guess_rows << " x " << init_guess_cols << std::endl;
#endif

    // Calculate the start byte for reading data
    std::streampos init_guess_startbyte = init_guess_file.tellg() + static_cast<std::streamoff>(init_guess_rows * (traj_select - 1) * sizeof(casadi_real));

    // Read the file data into the provided array
    read_file(init_guess_file, init_guess_startbyte, init_guess_data, init_guess_rows);

    return 0; // Return success
}

std::streamoff CasadiMPC::get_traj_dims()
{
    std::ifstream file(traj_file, std::ios::binary);
    std::streamoff traj_data_startbyte = 0;

    if (!file)
    {
        std::cerr << "Error opening file: " << traj_file << std::endl;
        return 0;
    }

    // Read dimensions
    file.read(reinterpret_cast<char *>(&traj_rows), sizeof(traj_rows));
    file.read(reinterpret_cast<char *>(&traj_data_total_len), sizeof(traj_data_total_len));
    file.read(reinterpret_cast<char *>(&traj_amount), sizeof(traj_amount));

#ifdef DEBUG
    std::cout << "Rows: " << traj_rows << ", Cols: " << traj_data_total_len << ", Trajectory Amount: " << traj_amount << std::endl;
#endif

    traj_data_startbyte = file.tellg() + static_cast<std::streamoff>(traj_rows * traj_data_total_len * (traj_select - 1) * sizeof(casadi_real));

    // Close the file
    file.close();
    return traj_data_startbyte;
}

void CasadiMPC::read_trajectory_block()
{
    std::ifstream file(traj_file, std::ios::binary);

    if (!file)
    {
        std::cerr << "Error opening file: " << traj_file << std::endl;
        return;
    }

    // Set the file pointer to the starting position
    file.seekg(traj_data_startbyte);

    for (casadi_uint j = 0; j < traj_data_per_horizon; j++)
    {
        // Move the read position according to mpc_traj_indices[j]
        file.seekg((traj_count + mpc_traj_indices[j]) * traj_rows * sizeof(double), std::ios::cur);

        // Read data into array y_d
        file.read(reinterpret_cast<char *>(&y_d[j * traj_rows]), traj_rows * sizeof(double));

        // Check for read errors
        if (!file)
        {
            std::cerr << "Error reading trajectory data at column: " << j << std::endl;
            return;
        }

        // Return to the starting position for next column
        file.seekg(traj_data_startbyte, std::ios::beg);
    }
    if (traj_count < traj_data_real_len - 1)
    {
        traj_count++;
    }
#ifdef DEBUG
    for (int i = 0; i < traj_data_per_horizon * traj_rows; i++)
    {
        std::cout << y_d[i] << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;
#endif
    file.close();
}

void CasadiMPC::read_x0_init(const std::string &x0_init_file, casadi_real *x0_arr)
{
    std::ifstream file(x0_init_file, std::ios::binary);

    if (!file)
    {
        std::cerr << "Error opening file: " << x0_init_file << std::endl;
        return;
    }

    // Read dimensions
    casadi_uint rows, cols;
    file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
    file.read(reinterpret_cast<char *>(&cols), sizeof(cols));

#ifdef DEBUG
    std::cout << "Rows: " << rows << ", Cols: " << cols << std::endl;
#endif

    // Calculate total number of elements
    size_t total_elements = rows * sizeof(casadi_real);

    // Skip to the correct column
    file.seekg(sizeof(rows) + sizeof(cols) + (traj_select - 1) * total_elements, std::ios::beg);

    // Read initial condition data directly into x0_arr
    file.read(reinterpret_cast<char *>(x0_arr), total_elements);

    // Check for read errors
    if (!file)
    {
        std::cerr << "Error reading data from file." << std::endl;
        return;
    }

    // Close the file
    file.close();
}

// Destructor to clean up allocated memory
CasadiMPC::~CasadiMPC()
{
}