#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy

#include "CasadiMPC.hpp"

#include "MPC8.h" // Include for MPC8
#include "MPC8_param.h" // Include for MPC8
#include "MPC8_addressdef.h"  // Include for MPC8

#define TRAJ_SELECT 1

void readTrajectoryData(const std::string& traj_file) {
    std::ifstream file(traj_file, std::ios::binary);
    
    if (!file) {
        std::cerr << "Error opening file: " << traj_file << std::endl;
        return;
    }

    // Read dimensions
    uint32_t rows, cols, traj_amount;
    file.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    file.read(reinterpret_cast<char*>(&cols), sizeof(cols));
    file.read(reinterpret_cast<char*>(&traj_amount), sizeof(traj_amount));

    std::cout << "Rows: " << rows << ", Cols: " << cols << ", Trajectory Amount: " << traj_amount << std::endl;

    // Calculate total number of elements
    size_t total_elements = rows * cols * traj_amount;
    std::vector<double> y_d(total_elements);

    // Read the trajectory data
    file.read(reinterpret_cast<char*>(y_d.data()), total_elements * sizeof(double));

    // Check for read errors
    if (!file) {
        std::cerr << "Error reading data from file." << std::endl;
        return;
    }

    // Process y_d as needed
    // For demonstration, print first few values
    for (size_t i = 0; i < std::min(total_elements, size_t(5*7)); ++i) {
        std::cout << y_d[i] << " ";
        if( i > 0 && (i+1) % 7 == 0)
        	std::cout << "\n";
    }
    std::cout << std::endl;

    // Close the file
    file.close();
}

void read_file(std::ifstream& file, std::streampos data_start, casadi_real* data, int data_len) {
    file.seekg(data_start);
    if (!file.good()) {
        throw std::runtime_error("Error seeking to data start position.");
    }

    file.read(reinterpret_cast<char*>(data), data_len * sizeof(casadi_real));
    if (!file) {
        throw std::runtime_error("Error reading data from file.");
    }
}

int load_initial_guess(const std::string& init_guess_path, casadi_real* init_guess_data) {
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
    std::streampos init_guess_startbyte = init_guess_file.tellg() + init_guess_rows * (TRAJ_SELECT - 1) * sizeof(casadi_real);

    // Read the file data into the provided array
    read_file(init_guess_file, init_guess_startbyte, init_guess_data, init_guess_rows);

    return 0; // Return success
}

unsigned int get_traj_dims(uint32_t& rows, uint32_t& cols, uint32_t& traj_amount, const std::string& traj_file) {
    std::ifstream file(traj_file, std::ios::binary);
    unsigned int traj_data_startbyte = 0;
    
    if (!file) {
        std::cerr << "Error opening file: " << traj_file << std::endl;
        return 0;
    }

    // Read dimensions
    file.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    file.read(reinterpret_cast<char*>(&cols), sizeof(cols));
    file.read(reinterpret_cast<char*>(&traj_amount), sizeof(traj_amount));

    std::cout << "Rows: " << rows << ", Cols: " << cols << ", Trajectory Amount: " << traj_amount << std::endl;

    traj_data_startbyte = file.tellg() + rows*cols*(TRAJ_SELECT-1) * sizeof(casadi_real);


    // Close the file
    file.close();
    return traj_data_startbyte;
}

void read_trajectory_block(const std::string& traj_file, unsigned int traj_data_startbyte, uint32_t rows, uint32_t cols, double* data, const uint32_t* indices) {
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

void readInitialConditionData(const std::string& q0_init_file) {
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
    std::vector<double> x0_arr(total_elements);

    // Read initial condition data
    file.read(reinterpret_cast<char*>(x0_arr.data()), total_elements * sizeof(double));

    // Check for read errors
    if (!file) {
        std::cerr << "Error reading data from file." << std::endl;
        return;
    }

    // Process x0_arr as needed
    // For demonstration, print first few values
    for (size_t i = 0; i < std::min(total_elements, size_t(rows)); ++i) {
        std::cout << x0_arr[i] << " ";
    }
    std::cout << std::endl;

    // Close the file
    file.close();
}

int main() {
    uint32_t rows = 0, cols = 0, traj_amount = 0, traj_data_startbyte = 0;
    uint32_t traj_indices[5] = {0};

    casadi_real x0_init[MPC8_X_K_LEN] = {0};
    casadi_real yd_init[MPC8_Y_D_LEN] = {0};

    casadi_real init_guess_data[MPC8_INIT_GUESS_LEN] = {0};

    traj_data_startbyte = get_traj_dims(rows, cols, traj_amount, TRAJ_DATA_PATH);
        
    readTrajectoryData(TRAJ_DATA_PATH);
    readInitialConditionData(X0_INIT_PATH);

    int result = load_initial_guess(MPC8_INIT_GUESS_PATH, init_guess_data);
    if (result) {
        std::cerr << "Error loading initial guess data." << std::endl;
        return result;
    }

    // print init_guess_data
    std::cout << "init_guess_data: \n";
    for (size_t i = 0; i < MPC8_INIT_GUESS_LEN; ++i) {
        std::cout << init_guess_data[i] << " ";
    }
    
    read_trajectory_block(TRAJ_DATA_PATH, traj_data_startbyte, rows, MPC8_traj_data_per_horizon, yd_init, MPC8_traj_indices);

    // print yd_init
    std::cout << "yd_init: \n";
    for (size_t i = 0; i < MPC8_Y_D_LEN; ++i) {
        std::cout << yd_init[i] << " ";
        if( i > 0 && (i+1) % 7 == 0)
        	std::cout << "\n";
    }

    // Create an instance for MPC8 using the defined constants
    const casadi_real* arg[MPC8_ARG_LEN];
    casadi_real* res[MPC8_RES_LEN];
    casadi_int iw[MPC8_IW_LEN];
    casadi_real w[MPC8_W_LEN];

    casadi_real u_opt[MPC8_U_OPT_LEN] = {0};

    memcpy(w+MPC8_X_K_ADDR, x0_init, sizeof(x0_init)); // init x0
    memcpy(w+MPC8_Y_D_ADDR, yd_init, sizeof(yd_init));
    memcpy(w+MPC8_IN_INIT_GUESS_ADDR, init_guess_data, sizeof(init_guess_data));
    memcpy(w+MPC8_IN_PARAM_WEIGHT_ADDR, MPC8_param_weight, sizeof(MPC8_param_weight));

    CasadiMPC MPC8_obj = CasadiMPC(&MPC8,
              arg,
              res,
              iw,
              w,
              MPC8_ARG,
              MPC8_RES,
              u_opt,
              sizeof(MPC8_ARG)/sizeof(uint32_t),
              sizeof(MPC8_RES)/sizeof(uint32_t),
              MPC8_U_OPT_LEN,
              MPC8_W_END_ADDRESS,
              MPC8_U_OPT_ADDR);

    int flag = MPC8_obj.solve();
    if (flag) return flag;

    // print flag
    std::cout << "flag: " << flag << std::endl;

    // print u_opt
    std::cout << "u_opt: \n";
    for (size_t i = 0; i < MPC8_U_OPT_LEN; ++i) {
        std::cout << u_opt[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}
