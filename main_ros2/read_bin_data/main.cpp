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

int main() {
    casadi_real* u_opt_out = nullptr;
    int u_opt_len = 0;
    CasadiMPC MPC8_obj = CasadiMPC("MPC8");

    int flag = MPC8_obj.solve();
    // if (flag) return flag;

    // print flag
    std::cout << "flag: " << flag << std::endl;

    // Get the optimal control
    MPC8_obj.get_optimal_control(u_opt_out, u_opt_len);

    // Print the optimal control
    std::cout << "Optimal control: ";
    for (int i = 0; i < u_opt_len; ++i) {
        std::cout << u_opt_out[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}
