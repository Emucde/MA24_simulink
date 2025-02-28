#include "SignalFilter.hpp"

// Constructor to initialize the filter with the number of degrees of freedom (DOF) and initial state
SignalFilter::SignalFilter(int num_joints, double* state, std::string general_config_filename)
    : num_joints(num_joints),
      general_config_filename(general_config_filename),
      x_filt(Eigen::Map<Eigen::VectorXd> (state, 2 * num_joints))
{
    update_config();
}

// Method to initialize the filter state and coefficients
void SignalFilter::update_config()
{
    nlohmann::json general_config = read_config<>(general_config_filename);

    double omega_c_q  = get_config_value<double>(general_config, "lowpass_filter_omega_c_q");
    double omega_c_dq = get_config_value<double>(general_config, "lowpass_filter_omega_c_dq");
    double Ts = get_config_value<double>(general_config, "dt");

    // Calculate filter coefficients
    a_q = std::exp(-omega_c_q * Ts); // Coefficient for position filter
    b_q = 1 - a_q;                   // Coefficient for position filter

    a_dq = std::exp(-omega_c_dq * Ts); // Coefficient for velocity filter
    b_dq = 1 - a_dq;                   // Coefficient for velocity filter
}

void SignalFilter::reset_state(double *state)
{
    // Set the initial filter state using the provided input state
    Eigen::Map<Eigen::VectorXd> state_eig(state, 2 * num_joints);
    x_filt << state_eig; // Initialize the filtered output
}

// Method to update the filter with measured values
void SignalFilter::run(double *state)
{
    // Map the measured state
    Eigen::Map<Eigen::VectorXd> q_meas(state, num_joints);
    Eigen::Map<Eigen::VectorXd> q_p_meas(state + num_joints, num_joints);

    // Apply the low-pass filter to the measured values (linear system form)
    x_filt.head(num_joints) = a_q * x_filt.head(num_joints) + b_q * q_meas;
    x_filt.tail(num_joints) = a_dq * x_filt.tail(num_joints) + b_dq * q_p_meas;
}