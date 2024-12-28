#include "CasadiMPC.hpp"
#include <cstring> // for memcpy

// Constructor implementation
CasadiMPC::CasadiMPC(   CasadiFunPtr_t casadi_fun, 
                        const casadi_real** arg, 
                        casadi_real** res, 
                        casadi_int* iw, 
                        casadi_real* w,
                        const uint32_t* arg_indices, 
                        const uint32_t* res_indices,
                        casadi_real* u_opt,
                        const uint32_t arg_len,
                        const uint32_t res_len,
                        const uint32_t u_opt_len,
                        const uint32_t w_end_addr_len,
                        const uint32_t u_opt_addr_len)
    : casadi_fun(casadi_fun), arg(arg), res(res), iw(iw), w(w), u_opt(u_opt),
    arg_len(arg_len), res_len(res_len), u_opt_len(u_opt_len),
    w_end_addr_ptr(w+w_end_addr_len), u_opt_res_ptr(w+u_opt_addr_len), mem(0) {

    // Initialize arg with pointers based on provided indices
    for (int i=0; i<arg_len; i++) {
        arg[i] = w + arg_indices[i]; // Use each index from arg_indices
    }
    
    // Initialize res with pointers based on provided indices
    for (int i=0; i<res_len; i++) {
        res[i] = w + res_indices[i]; // Use each index from res_indices
    }
}

// Method to run the MPC
int CasadiMPC::solve() {
    // Call the Casadi function
    int flag = casadi_fun(arg, res, iw, w_end_addr_ptr, mem);
    if (flag) {
        return flag; // Handle the error
    }

    // Copy the optimal control result to the provided array
    for(int i=0; i<u_opt_len; i++)
    {
        u_opt[i] = u_opt_res_ptr[i];
    }
    
    // Logic to handle results and cleanup can go here
    return flag; // Return the flag
}

// Destructor to clean up allocated memory
CasadiMPC::~CasadiMPC() {
}