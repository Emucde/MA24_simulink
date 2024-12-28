#ifndef CASADIMPC_HPP
#define CASADIMPC_HPP

#include <vector>
#include <iostream>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

typedef int (*CasadiFunPtr_t)(const casadi_real **arg, casadi_real **res, casadi_int *iw, casadi_real *w, int mem);

class CasadiMPC
{
public:
    // Constructor that accepts parameters for configuration
    CasadiMPC(CasadiFunPtr_t casadi_fun,
              const casadi_real **arg,
              casadi_real **res,
              casadi_int *iw,
              casadi_real *w,
              const uint32_t *arg_indices,
              const uint32_t *res_indices,
              casadi_real *u_opt,
              const uint32_t arg_len,
              const uint32_t res_len,
              const uint32_t u_opt_len,
              const uint32_t w_end_addr_len,
              const uint32_t u_opt_addr_len);

    // Method to initialize and run the MPC
    int solve();

    // Destructor
    ~CasadiMPC();

private:
    CasadiFunPtr_t casadi_fun;   // Function pointer
    const casadi_real **arg;     // Pointer to arguments
    casadi_real **res;           // Pointer to results
    casadi_int *iw;              // Workspace integer
    casadi_real *w;              // Workspace real
    casadi_real *u_opt;          // Optimal control result
    const uint32_t arg_len;      // Length of arg
    const uint32_t res_len;      // Length of res
    const uint32_t u_opt_len;    // Length of u_opt
    casadi_real *w_end_addr_ptr; // End address for w
    casadi_real *u_opt_res_ptr;  // Pointer to optimal control result
    int mem;                     // Memory flag
};

#endif // CASADIMPC_HPP