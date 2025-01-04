// #include "CasadiController.hpp"

// CasadiController::CasadiController(const std::string& urdfFilePath, bool useGravity, bool fr3KinematicModel)
//     : useGravity(useGravity), 
//       fr3KinematicModel(fr3KinematicModel) { // Initialize with first MPC, will be updated later

//     // List of MPC names
//     const std::vector<std::string> mpcNames = {
//         "MPC01", "MPC6", "MPC7", "MPC8", "MPC9", 
//         "MPC10", "MPC11", "MPC12", "MPC13", "MPC14"
//     };

//     // Initialize each MPC instance
//     for (const auto& mpcName : mpcNames) {
//         mpcObjects[mpcName] = CasadiMPC(mpcName);
//     }

//     // Assume starting with MPC01 for the first run
//     u_opt = mpcObjects["MPC01"].get_optimal_control();
//     x_k = mpcObjects["MPC01"].get_x0();
//     trajDataLength = mpcObjects["MPC01"].get_traj_data_len();
    
//     // Setup indices and maps based on the selected MPC; for now using MPC01.
//     setupIndicesAndMaps();
// }

// void CasadiController::setupIndicesAndMaps() {
//     // Initialize Eigen vectors using the first MPC, can be adjusted to select different MPCs
//     x_k_ndof = Eigen::VectorXd::Zero(mpcObjects["MPC01"].nx);
//     tau_full = Eigen::VectorXd::Zero(mpcObjects["MPC01"].nq);

//     const casadi_uint* n_x_indices_ptr = mpcObjects["MPC01"].get_n_x_indices();
//     n_x_indices = Eigen::Map<Eigen::VectorXi>(reinterpret_cast<const int*>(const_cast<uint32_t*>(n_x_indices_ptr)), mpcObjects["MPC01"].nq_red);
    
//     std::vector<casadi_real> x_ref_nq_vec = mpcObjects["MPC01"].get_x_ref_nq();
//     x_ref_nq_map = Eigen::Map<const Eigen::VectorXd>(x_ref_nq_vec.data(), x_ref_nq_vec.size());
//     x_k_map = Eigen::Map<Eigen::VectorXd>(x_k, mpcObjects["MPC01"].nx_red);

//     // Create pointers to manage the updated state data
//     int cnt = 0;
//     for (casadi_uint i = 0; i < mpcObjects["MPC01"].nx; ++i) {
//         if (i == n_x_indices(cnt)) {
//             x_k_ndof_ptr.push_back(&x_k[cnt]);
//             cnt++;
//         } else {
//             x_k_ndof_ptr.push_back(const_cast<double*>(&x_ref_nq_vec[i]));
//         }
//     }
// }

// void CasadiController::run() {
//     // Initialize data files if plotting is enabled
// #ifdef PLOT_DATA
//     std::ofstream x_k_ndof_file("x_k_ndof_data.txt");
//     std::ofstream tau_full_file("tau_full_data.txt");
// #endif

//     // Measure execution time
//     auto start = std::chrono::high_resolution_clock::now();

//     for (casadi_uint i = 0; i < trajDataLength; i++) {
//         int flag = mpcObjects["MPC01"].solve(x_k);  // Replace with desired MPC if needed
//         if (flag) {
//             std::cerr << "Error in Casadi function call." << std::endl;
//             break;
//         }

//         // Update the state and calculate full torque
//         memcpy(x_k, u_opt + mpcObjects["MPC01"].nq_red, mpcObjects["MPC01"].nx_red * sizeof(casadi_real));
//         tau_full = torqueMapper.calc_full_torque(
//             Eigen::Map<Eigen::VectorXd>(u_opt, mpcObjects["MPC01"].nq_red), 
//             x_k_ndof, 
//             mpcObjects["MPC01"].nq, 
//             fr3KinematicModel
//         );

// #ifdef PLOT_DATA
//         writeDataToFiles(x_k_ndof_file, tau_full_file);
// #endif

//         // Print every 100 steps
//         if (i % 100 == 0) {
//             printCurrentState();
//         }
//     }

// #ifdef PLOT_DATA
//     x_k_ndof_file.close();
//     tau_full_file.close();
// #endif

//     // Measure and print execution time
//     auto stop = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//     std::cout << "Time taken by function: " << static_cast<double>(duration.count()) / 1000000 << " s" << std::endl;
// }

// void CasadiController::writeDataToFiles(std::ofstream& x_k_ndof_file, std::ofstream& tau_full_file) {
//     for (size_t i = 0; i < x_k_ndof_ptr.size(); ++i) {
//         x_k_ndof_file << *x_k_ndof_ptr[i];
//         if (i < x_k_ndof_ptr.size() - 1)
//             x_k_ndof_file << "\t";
//     }
//     x_k_ndof_file << std::endl;

//     tau_full_file << tau_full.transpose() << std::endl;
// }

// void CasadiController::printCurrentState() {
//     for (size_t i = 0; i < x_k_ndof_ptr.size(); ++i) {
//         std::cout << *x_k_ndof_ptr[i];
//         if (i < x_k_ndof_ptr.size() - 1)
//             std::cout << " ";
//     }
//     std::cout << std::endl;
// }