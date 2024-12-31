#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy

#include "CasadiMPC.hpp"

// includes for time measurement
#include <chrono>

#define TRAJ_SELECT 1

int main() {
    casadi_real* u_opt_out = nullptr;
    int u_opt_len = 0, i=0;

    int flag = 0;
    uint32_t traj_data_real_len;

    CasadiMPC MPC8_obj = CasadiMPC("MPC8");

    // Get the length of the trajectory data
    traj_data_real_len = MPC8_obj.get_traj_data_len();

    // measure time
    auto start = std::chrono::high_resolution_clock::now();

    for(i=0; i<1; i++) {
        flag = MPC8_obj.solve();
        // // Get the optimal control
        // MPC8_obj.get_optimal_control(u_opt_out, u_opt_len);

        // // Print the optimal control
        // std::cout << "Optimal control: ";
        // for (int i = 0; i < u_opt_len; ++i) {
        //     std::cout << u_opt_out[i] << " ";
        // }
        // std::cout << std::endl;
    }

    // measure time
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: "
         << (double) duration.count()/1000000 << " s" << std::endl;

    return 0;
}
