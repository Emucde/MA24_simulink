#ifndef MPC8_PARAM_WEIGHT_H
#define MPC8_PARAM_WEIGHT_H

#include <stdint.h>

#ifndef Inf
#define Inf INFINITY
#endif

typedef struct {
    double Q_y[36];
    double Q_ykp1[36];
    double Q_yN[36];
    double R_q_pp[36];
    double x_min[12];
    double x_max[12];
    double u_min[6];
    double u_max[6];
} MPC8_ParamWeight_t;

const MPC8_ParamWeight_t MPC8_param_weight = {
    .Q_y = {
        /* 6x6 matrix values */
        100, 0, 0, 0, 0, 0, 
        0, 100, 0, 0, 0, 0, 
        0, 0, 100, 0, 0, 0, 
        0, 0, 0, 100, 0, 0, 
        0, 0, 0, 0, 100, 0, 
        0, 0, 0, 0, 0, 100, 
    },
    .Q_ykp1 = {
        /* 6x6 matrix values */
        100, 0, 0, 0, 0, 0, 
        0, 100, 0, 0, 0, 0, 
        0, 0, 100, 0, 0, 0, 
        0, 0, 0, 100, 0, 0, 
        0, 0, 0, 0, 100, 0, 
        0, 0, 0, 0, 0, 100, 
    },
    .Q_yN = {
        /* 6x6 matrix values */
        100000, 0, 0, 0, 0, 0, 
        0, 100000, 0, 0, 0, 0, 
        0, 0, 100000, 0, 0, 0, 
        0, 0, 0, 100000, 0, 0, 
        0, 0, 0, 0, 100000, 0, 
        0, 0, 0, 0, 0, 100000, 
    },
    .R_q_pp = {
        /* 6x6 matrix values */
        1e-10, 0, 0, 0, 0, 0, 
        0, 1e-10, 0, 0, 0, 0, 
        0, 0, 1e-10, 0, 0, 0, 
        0, 0, 0, 1e-10, 0, 0, 
        0, 0, 0, 0, 1e-10, 0, 
        0, 0, 0, 0, 0, 1e-10, 
    },
    .x_min = {
        /* [12 1] array values */
        -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, 
    },
    .x_max = {
        /* [12 1] array values */
        Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, Inf, 
    },
    .u_min = {
        /* [6 1] array values */
        -Inf, -Inf, -Inf, -Inf, -Inf, -Inf, 
    },
    .u_max = {
        /* [6 1] array values */
        Inf, Inf, Inf, Inf, Inf, Inf, 
    },
};


#endif