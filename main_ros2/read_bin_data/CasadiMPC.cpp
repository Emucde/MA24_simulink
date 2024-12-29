#include "CasadiMPC.hpp"
#include <cstring> // for memcpy
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>  // for std::make_unique
#include "MPC8_param.h" // Include for MPC8
#include "casadi_types.h" // Include for casadi types

// Constructor implementation
CasadiMPC::CasadiMPC(const std::string& mpc_name) : traj_select(1) {
    if(mpc_name == "MPC8") {
        mpc_config = std::make_unique<mpc_config_t>(get_MPC8_config());
    } else {
        throw std::runtime_error("MPC not supported");
    }

    mpc_config_t config = *this->mpc_config.get();

    w = config.w;
    arg = config.arg;
    res = config.res;
    iw = config.iw;
    u_opt = config.u_opt;
    mem = config.mem;
    casadi_fun = config.casadi_fun;
    // Initialize arg with pointers based on provided indices
    for (int i=0; i<config.arg_in_len; i++) {
        arg[i] = w + config.arg_indices[i]; // Use each index from arg_indices
    }
    
    // Initialize res with pointers based on provided indices
    for (int i=0; i<config.res_out_len; i++) {
        res[i] = w + config.res_indices[i]; // Use each index from res_indices
    }

    u_opt = w+config.u_opt_addr_len; // Set u_opt to the address at u_opt_addr_len
    w_end = w+config.w_end_addr_len; // Set w_end to the address at w_end_addr_len

    init_guess = w + config.in_init_guess_addr;
    x_k = w + config.x_k_addr;
    y_d = w + config.y_d_addr;
    param_weight = w + config.in_param_weight_addr;

    // read first initial guess from file
    load_initial_guess(config.init_guess_path, init_guess);

    // read first x0 from file
    read_x0_init(config.x0_init_path, x_k);

    // read first trajectory data from file
    uint32_t rows, cols, traj_amount;
    std::streamoff traj_data_startbyte = get_traj_dims(rows, cols, traj_amount, config.traj_data_path);
    read_trajectory_block(config.traj_data_path, traj_data_startbyte, rows, config.traj_data_per_horizon, y_d, config.traj_indices);

    memcpy(param_weight, config.param_weight, sizeof(config.param_weight)); // Copy the parameter weights
}
// Method to run the MPC
int CasadiMPC::solve() {
    // Call the Casadi function
    return casadi_fun(arg, res, iw, w_end, mem);
}

void CasadiMPC::read_file(std::ifstream& file, std::streampos data_start, casadi_real* data, int data_len) {
    file.seekg(data_start);
    if (!file.good()) {
        throw std::runtime_error("Error seeking to data start position.");
    }

    file.read(reinterpret_cast<char*>(data), data_len * sizeof(casadi_real));
    if (!file) {
        throw std::runtime_error("Error reading data from file.");
    }
}

int CasadiMPC::load_initial_guess(const std::string& init_guess_path, casadi_real* init_guess_data) {
    std::ifstream init_guess_file(init_guess_path, std::ios::binary);
    if (!init_guess_file.is_open()) {
        std::cerr << "Error opening init_guess_file" << std::endl;
        return 1;
    }

    unsigned int init_guess_rows, init_guess_cols;

    // Read dimensions
    init_guess_file.read(reinterpret_cast<char*>(&init_guess_rows), sizeof(init_guess_rows));
    init_guess_file.read(reinterpret_cast<char*>(&init_guess_cols), sizeof(init_guess_cols));

    if (!init_guess_file.good()) {
        std::cerr << "Error reading dimensions from file" << std::endl;
        return 1;
    }

    std::cout << "init_guess dimensions: " << init_guess_rows << " x " << init_guess_cols << std::endl;

    // Calculate the start byte for reading data
    std::streampos init_guess_startbyte = init_guess_file.tellg() + static_cast<std::streamoff>(init_guess_rows * (traj_select - 1) * sizeof(casadi_real));

    // Read the file data into the provided array
    read_file(init_guess_file, init_guess_startbyte, init_guess_data, init_guess_rows);

    return 0; // Return success
}

std::streamoff CasadiMPC::get_traj_dims(uint32_t& rows, uint32_t& cols, uint32_t& traj_amount, const std::string& traj_file) {
    std::ifstream file(traj_file, std::ios::binary);
    std::streamoff traj_data_startbyte = 0;
    
    if (!file) {
        std::cerr << "Error opening file: " << traj_file << std::endl;
        return 0;
    }

    // Read dimensions
    file.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    file.read(reinterpret_cast<char*>(&cols), sizeof(cols));
    file.read(reinterpret_cast<char*>(&traj_amount), sizeof(traj_amount));

    std::cout << "Rows: " << rows << ", Cols: " << cols << ", Trajectory Amount: " << traj_amount << std::endl;

    traj_data_startbyte = file.tellg() + static_cast<std::streamoff>(rows*cols*(traj_select-1) * sizeof(casadi_real));


    // Close the file
    file.close();
    return traj_data_startbyte;
}

void CasadiMPC::read_trajectory_block(const std::string& traj_file, unsigned int traj_data_startbyte, uint32_t rows, uint32_t cols, double* data, const uint32_t* indices) {
    std::ifstream file(traj_file, std::ios::binary);

    if (!file) {
        std::cerr << "Error opening file: " << traj_file << std::endl;
        return;
    }

    // Set the file pointer to the starting position
    file.seekg(traj_data_startbyte);

    for (uint32_t j = 0; j < cols; j++) {
        // Move the read position according to indices[j]
        file.seekg(indices[j] * rows * sizeof(double), std::ios::cur);

        // Read data into the provided array
        file.read(reinterpret_cast<char*>(&data[j * rows]), rows * sizeof(double));

        // Check for read errors
        if (!file) {
            std::cerr << "Error reading data at column: " << j << std::endl;
            return;
        }

        // Return to the starting position for next column
        file.seekg(traj_data_startbyte, std::ios::beg);
    }
    file.close();
}

void CasadiMPC::read_x0_init(const std::string& q0_init_file, casadi_real* x0_arr) {
    std::ifstream file(q0_init_file, std::ios::binary);
    
    if (!file) {
        std::cerr << "Error opening file: " << q0_init_file << std::endl;
        return;
    }

    // Read dimensions
    uint32_t rows, cols;
    file.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    file.read(reinterpret_cast<char*>(&cols), sizeof(cols));

    std::cout << "Rows: " << rows << ", Cols: " << cols << std::endl;

    // Calculate total number of elements
    size_t total_elements = rows * cols;

    // Read initial condition data
    file.read(reinterpret_cast<char*>(x0_arr), total_elements * sizeof(double));

    // Check for read errors
    if (!file) {
        std::cerr << "Error reading data from file." << std::endl;
        return;
    }

    std::cout << std::endl;

    // Close the file
    file.close();
}

// Destructor to clean up allocated memory
CasadiMPC::~CasadiMPC() {
}