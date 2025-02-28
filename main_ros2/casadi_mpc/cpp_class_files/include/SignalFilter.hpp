#ifndef SIGNALFILTER_HPP
#define SIGNALFILTER_HPP

#include <Eigen/Dense>
#include "eigen_templates.hpp"

class SignalFilter {
public:   
    // Constructor to initialize the filter with the number of degrees of freedom (DOF) and initial state
    SignalFilter(int num_joints, double* state, std::string general_config_filename);
    
    // Method to initialize the filter state and coefficients
    void update_config();
    void reset_state(double* state);
    
    // Method to update the filter with measured values
    void run(double* state);

    // Method to retrieve the filtered output
    Eigen::VectorXd getFilteredOutput() const {
        return x_filt;
    }

    // Method to retrieve the filtered output pointer
    double* getFilteredOutputPtr() {
        return x_filt.data();
    }

private:
    int num_joints;                       // Number of joints
    std::string general_config_filename;  // Path to the general configuration file
    double a_q;                           // Coefficient for the position filter
    double b_q;                           // Coefficient for the position filter
    double a_dq;                          // Coefficient for the velocity filter
    double b_dq;                          // Coefficient for the velocity filter
    Eigen::VectorXd x_filt;               // Filtered output state
};

#endif // SIGNALFILTER_HPP