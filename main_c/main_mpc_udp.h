// Header for main_MPC_udp.c

#ifndef MAIN_MPC_UDP_H
#define MAIN_MPC_UDP_H

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

#ifndef CASADI_MAX_NUM_THREADS
#define CASADI_MAX_NUM_THREADS 1
#endif

#define CASADI_PRINTF printf

// CASADI DEPENDENCIES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// TIMER DEPENDENCIES
#include <time.h>
#include <pthread.h>

// UDP DEPENDENCIES
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "../s_functions/ur5e_6dof/mpc_c_sourcefiles/MPC8.h"
#include "../s_functions/ur5e_6dof/mpc_c_sourcefiles/MPC8_adressdef.h"
#include "../s_functions/ur5e_6dof/mpc_c_sourcefiles/MPC8_param_weight.h"

typedef int (*CasadiFunPtr_t)(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);

void delay_ms(long ms);

void read_trajectory_block(FILE* file, long int data_start, casadi_real* data, const int* indices);

void read_file(FILE* file, long int data_start, casadi_real* data, const int data_len);

struct shared_data {
    casadi_real *u_opt_new;              // Buffer to store data
    size_t u_opt_new_size;        // Size of the buffer
    casadi_real *input_buffer;              // Buffer to store data
    size_t input_buffer_size;        // Size of the buffer
    int sockfd;                // Socket file descriptor
    struct sockaddr_in udp_c_send_addr; // Destination address for UDP
    int udp_c_send_addrlen;         // Length of the address
    struct sockaddr_in udp_c_receive_addr; // Destination address for UDP
    int udp_c_receive_addrlen;         // Length of the address
    pthread_mutex_t mutex;     // Mutex for thread synchronization
    int should_exit;           // Flag to signal thread termination
    unsigned long update_interval_ns; // Interval between data updates (in nanoseconds)
    unsigned long send_interval_ns;   // Interval between sends (in nanoseconds)
    const casadi_real** arg;
    casadi_real** res;
    casadi_int* iw;
    casadi_real* w;
    casadi_real* w_end_addr;
    struct timespec start_time; // Start time for runtime tracking
    FILE *traj_file;
    CasadiFunPtr_t casadi_fun;
};

void *update_data(void *arg);

void *send_data(void *arg);

void threaded_send(int sockfd, casadi_real *u_opt_new, size_t u_opt_new_size,
                   casadi_real *input_buffer, size_t input_buffer_size,
                   const struct sockaddr_in udp_c_send_addr,
                   const struct sockaddr_in udp_c_receive_addr,
                   unsigned long update_interval_ms, unsigned long send_interval_ms,
                   const casadi_real** arg, casadi_real** res,
                   casadi_int* iw, casadi_real* w, casadi_real* w_end_addr,
                   FILE *traj_file, CasadiFunPtr_t casadi_fun);

casadi_int main_MPC(const casadi_real** arg, casadi_real** res,
                    casadi_int* iw, casadi_real* w, casadi_real* w_end_addr, int mem,
                    CasadiFunPtr_t casadi_fun);
#endif // MAIN_MPC_UDP_H