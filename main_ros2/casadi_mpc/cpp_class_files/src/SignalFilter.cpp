#include "SignalFilter.hpp"

// Constructor to initialize the filter with the number of degrees of freedom (DOF) and initial state
SignalFilter::SignalFilter(int num_joints, double Ts, double *state, double omega_c_q, double omega_c_dq)
    : SignalFilter(num_joints, Ts, omega_c_q, omega_c_dq)
{
    init(state, omega_c_q, omega_c_dq);
}

SignalFilter::SignalFilter(int num_joints, double Ts, double omega_c_q, double omega_c_dq)
    : num_joints(num_joints),
      Ts(Ts),
      x_filt(Eigen::VectorXd::Zero(2 * num_joints)),
      q_filt(x_filt.head(num_joints)),
      q_p_filt(x_filt.tail(num_joints))
{
    update_filter_coeffs(omega_c_q, omega_c_dq);
}

// Method to initialize the filter state and coefficients
void SignalFilter::update_filter_coeffs(double omega_c_q, double omega_c_dq)
{
    // Calculate filter coefficients
    a_q = std::exp(-omega_c_q * Ts); // Coefficient for position filter
    b_q = 1 - a_q;                   // Coefficient for position filter

    a_dq = std::exp(-omega_c_dq * Ts); // Coefficient for velocity filter
    b_dq = 1 - a_dq;                   // Coefficient for velocity filter
}

void SignalFilter::init(double *state, double omega_c_q, double omega_c_dq)
{
    update_filter_coeffs(omega_c_q, omega_c_dq);
    Eigen::Map<Eigen::VectorXd> state_eig(state, 2 * num_joints);
    x_filt = state_eig; // Initialize the filtered output
}

// Method to update the filter with measured values
void SignalFilter::run(double *state)
{
    // Map the measured state
    Eigen::Map<Eigen::VectorXd> q_meas(state, num_joints);
    Eigen::Map<Eigen::VectorXd> q_p_meas(state + num_joints, num_joints);

    // Apply the low-pass filter to the measured values (linear system form)
    q_filt = a_q * q_filt + b_q * q_meas;
    q_p_filt = a_dq * q_p_filt + b_dq * q_p_meas;
}