#ifndef SIGNALFILTER_HPP
#define SIGNALFILTER_HPP

#include <Eigen/Dense>

class SignalFilter {
public:   
    // Constructor to initialize the filter with the number of degrees of freedom (DOF) and initial state
    SignalFilter(int num_joints, double Ts, double* state, double omega_c_q, double omega_c_dq);
    
    // Method to initialize the filter state and coefficients
    void init(double* state, double omega_c_q, double omega_c_dq);
    
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
    double Ts;                            // Sample time duration for the filter
    double a_q;                           // Coefficient for the position filter
    double b_q;                           // Coefficient for the position filter
    double a_dq;                          // Coefficient for the velocity filter
    double b_dq;                          // Coefficient for the velocity filter
    Eigen::VectorXd x_filt;               // Filtered output state
    Eigen::Ref<Eigen::VectorXd> q_filt;   // Reference for joint angle filtered output
    Eigen::Ref<Eigen::VectorXd> q_p_filt; // Reference for joint velocity filtered output
};

#endif // SIGNALFILTER_HPP