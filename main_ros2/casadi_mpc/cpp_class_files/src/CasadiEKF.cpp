#include "CasadiEKF.hpp"

CasadiEKF::CasadiEKF(std::string ekf_config_path) : ekf_config_path(ekf_config_path) {

    // Initialize arg with pointers based on provided indices
    for (casadi_uint i = 0; i < FR3_EKF_ARG_LEN; i++)
    {
        arg[i] = w + FR3_EKF_ARG[i]; // Use each index from arg_indices
    }

    // Initialize res with pointers based on provided indices
    for (casadi_uint i = 0; i < FR3_EKF_RES_LEN; i++)
    {
        res[i] = w + FR3_EKF_RES[i]; // Use each index from res_indices
    }

    update_config();
}

void CasadiEKF::update_config()
{
    std::ifstream file(ekf_config_path);
    if (!file.is_open())
    {
        std::cerr << "Error: Could not open JSON file." << std::endl;
        return;
    }
    nlohmann::json jsonData;
    file >> jsonData; // Parse JSON file
    file.close();
    ekf_settings = jsonData["param_EKF"];

    // Initialize matrices
    P0 = get_param_matrix<>("P0");
    Qk = get_param_matrix<>("Qk_FR3");
    Rk = get_param_matrix<>("Rk_FR3");

    memcpy(w + FR3_EKF_QK_ADDR, Qk.data(), FR3_EKF_QK_LEN * sizeof(casadi_real));
    memcpy(w + FR3_EKF_RK_ADDR, Rk.data(), FR3_EKF_RK_LEN * sizeof(casadi_real));
}

void CasadiEKF::initialize(double* x_k_ptr)
{
    // Initialize the EKF
    memcpy(w + FR3_EKF_XK_MINUS_ADDR, x_k_ptr, FR3_EKF_XK_MINUS_LEN * sizeof(casadi_real));
    memcpy(w + FR3_EKF_PK_MINUS_ADDR, P0.data(), FR3_EKF_PK_MINUS_LEN * sizeof(casadi_real));
}

void CasadiEKF::predict(const double* u_k, const double* y_k)
{
    memcpy(w + FR3_EKF_UK_ADDR, u_k, FR3_EKF_UK_LEN * sizeof(casadi_real));
    memcpy(w + FR3_EKF_YK_ADDR, y_k, FR3_EKF_YK_LEN * sizeof(casadi_real));

    // std::cout << "w:" << std::endl;
    // for(int i=0; i<FR3_EKF_W_END_ADDR; i++)
    // {
    //     std::cout << w[i] << " ";
    // }

    // std::cout << std::endl << std::endl;

    int flag = ekf_fun(arg, res, iw, w+FR3_EKF_W_END_ADDR, mem);

    if (flag)
    {
        std::cerr << "Error in Casadi function call." << std::endl;
    }

    update();
}
void CasadiEKF::update()
{
    memcpy(w + FR3_EKF_XK_MINUS_ADDR, w + FR3_EKF_XKP1_MINUS_ADDR, FR3_EKF_XK_MINUS_LEN * sizeof(casadi_real));
    memcpy(w + FR3_EKF_PK_MINUS_ADDR, w + FR3_EKF_PKP1_MINUS_ADDR, FR3_EKF_PK_MINUS_LEN * sizeof(casadi_real));
}
