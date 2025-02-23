#include "CasadiMPC.hpp"
#include <cstring> // for memcpy
#include <iostream>
#include <fstream>
#include <vector>
#include <memory> // for std::make_unique
#include "eigen_templates.hpp"

#define USE_PATH_FOLLOWING_MPC
#define USE_REFERENCE_SYS_MPC

// #define DEBUG 1

// Constructor implementation
CasadiMPC::CasadiMPC(CasadiMPCType mpc,
                     robot_config_t &robot_config,
                     TrajectoryGenerator &trajectory_generator) : CommonBaseMPC(robot_config, trajectory_generator),
                                                                  mpc_name(casadi_mpctype_to_string(mpc)),
                                                                  mpc_config(get_MPC_config(mpc)),
                                                                  is_kinematic_mpc(mpc_config.kinematic_mpc == 1), is_planner_mpc(mpc_config.planner_mpc == 1),
                                                                  casadi_fun(mpc_config.casadi_fun),
                                                                  arg(mpc_config.arg), res(mpc_config.res), iw(mpc_config.iw), w(mpc_config.w), w_end(mpc_config.w_end),
                                                                  horizon_len(mpc_config.traj_data_per_horizon),
                                                                  mpc_traj_indices((Eigen::VectorXi(horizon_len) << ConstIntVectorMap(mpc_config.traj_indices, horizon_len)).finished()),
                                                                  traj_data_per_horizon(mpc_config.traj_data_per_horizon),
                                                                  mem(mpc_config.mem)
{
    // Check if the configuration is valid
    if (mpc_config.casadi_fun == nullptr)
    {
        throw std::runtime_error("CasadiMPC::CasadiMPC(): " + mpc_name + "is not a valid MPC name. Use one of the following: MPC01, MPC6, MPC7, MPC8, MPC9, MPC10, MPC11, MPC12, MPC13, MPC14");
    }

    // Initialize arg with pointers based on provided indices
    for (casadi_uint i = 0; i < mpc_config.arg_in_len; i++)
    {
        arg[i] = w + mpc_config.arg_indices[i]; // Use each index from arg_indices
    }

    // Initialize res with pointers based on provided indices
    for (casadi_uint i = 0; i < mpc_config.res_out_len; i++)
    {
        res[i] = w + mpc_config.res_indices[i]; // Use each index from res_indices
    }

    // Check trajectory pointer
    if (traj_data == nullptr)
    {
        throw std::runtime_error("CasadiMPC::CasadiMPC(): Trajectory data is not provided.");
    }
    traj_rows = traj_data->rows();
    traj_cols = traj_data->cols();

    // check trajectory lengths
    if (traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1] > traj_cols)
    {
        throw std::runtime_error("CasadiMPC::CasadiMPC(): \
                                  Trajectory data length + mpc_traj_indices[end] = " +
                                 std::to_string(traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1]) +
                                 " exceeds totalthrow length = " +
                                 std::to_string(traj_cols) +
                                 ". Please generate new.");
    }

    // Copy the parameter weights
    memcpy(mpc_config.in.param_weight.ptr, mpc_config.param_weight, mpc_config.param_weight_len * sizeof(casadi_real));

    // Check whether quaternions or rpy angles are used
    // in case of mpc_config.in.z_k.len == 0 or mpc_config.in.z_k.len == 13 a quaternion is used
    // only in case of mpc_config.in.z_k.len == 12 rpy angles are used for y_d, y_d_p, y_d_pp
    #ifdef USE_REFERENCE_SYS_MPC
    use_quat = (mpc_config.in.z_k.len != 12);
    #else
    use_quat = true;
    #endif

#ifdef DEBUG
    std::cout << "w: ";
    for (int i = 0; i < int(mpc_config.in.init_guess.len + mpc_config.in.reference_values.len + mpc_config.in.param_weight.len); i++)
    {
        std::cout << w[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "MPC object created." << std::endl;
#endif
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PUBLIC METHODS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////                                                                             /////
/////                ||||||   ||   ||  ||||    ||      ||    |||||                /////
/////                ||   ||  ||   ||  ||  ||  ||      ||  ||                     /////
/////                ||||||   ||   ||  ||||||  ||      ||  ||                     /////
/////                ||       ||   ||  ||  ||  ||      ||  ||                     /////
/////                ||        |||||   ||||    ||||||  ||    |||||                /////
/////                                                                             /////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

// Method to run the MPC
int CasadiMPC::solve(const casadi_real *const x_k_in)
{ // Copy the state to the MPC object
    set_references(x_k_in);

    // Call the Casadi function
    int flag = casadi_fun(arg, res, iw, w_end, mem);

    // Set initial guess and prev ref values
    if (!flag)
    {
        mpc_config.set_prev_to_out(w);
        // mpc_config.set_init_guess_out_to_in(w);
        predict_init_guess();
    }
    else
    {
        std::cerr << "Error in Casadi function call." << std::endl;
    }

    return flag;
}

int CasadiMPC::solve_planner()
{
    // Call the Casadi function
    int flag = casadi_fun(arg, res, iw, w_end, mem);

    // Read the next trajectory block
    if (!flag)
    {
        // read_trajectory_block();
        mpc_config.set_init_guess_out_to_in(w);

        // mpc planner: use x_k+1 as x_k for next iteration - not the measurement!
        set_references(mpc_config.out.x_out.ptr + nx_red);
    }
    else
    {
        std::cerr << "Error in Casadi function call." << std::endl;
    }

    return flag;
}

void CasadiMPC::update_mpc_weights(nlohmann::json param_weight)
{
    // iterate all keys in param_weight and update the weights
    for (auto &item : param_weight.items())
    {
        const MPCInput input_id = string_to_mpc_input(item.key());
        const mpc_input_entry_t *entry = get_mpc_input_entry(&mpc_config.in, static_cast<mpc_input_config_id_t>(input_id));
        if (entry != nullptr)
        {
            if (item.value().is_array())
            {
                std::vector<casadi_real> weight = item.value().get<std::vector<casadi_real>>();
                entry->set(w, weight.data());
            }
            else if (item.value().is_number())
            {
                casadi_real weight = item.value().get<casadi_real>();
                entry->set(w, &weight);

                if (item.key() == "N_step")
                {
                    N_step = weight;
                }

                if (item.key() == "dt")
                {
                    dt = weight;
                }
            }
            else
            {
                throw std::runtime_error("CasadiMPC::update_mpc_weights(): " + item.key() + " has an invalid type. Expected scalar or array.");
            }
        }
        else
        {
            throw std::runtime_error("CasadiMPC::update_mpc_weights(): " + item.key() + " is not a valid parameter name.");
        }
    }

    if(dt < robot_config.dt)
    {
        throw std::runtime_error("CasadiMPC::update_mpc_weights(): dt is too small. Expected dt > robot_config.dt.");
    }
    else
    {
        traj_step = static_cast<casadi_uint>(dt / robot_config.dt); // default dt is 1kHz
        if(N_step == 1)
        {
            mpc_traj_indices = traj_step * Eigen::VectorXi::LinSpaced(horizon_len, 0, horizon_len - 1);
        }
        else
        {
            Eigen::VectorXi temp_indices = traj_step * N_step * Eigen::VectorXi::LinSpaced(horizon_len-2, 1, horizon_len-2);
            mpc_traj_indices << 0, traj_step, temp_indices;
            // such that I have e.g. [0, 1, 5, 10, 15, 20] for N_step = 5 and N_MPC+1 = horizon_len = 6 and dt=1ms
        }
    }
}

// Init guess for optimal prediction: If the prediction is correct, then this approximation woud fit very well.
// it can be seen as an alternative to the standard method
void CasadiMPC::predict_init_guess()
{
    int u_rows = nq_red;
    int x_rows = nx_red;
    int u_cols = mpc_config.out.u_out.len / nq_red;
    int x_cols = mpc_config.out.x_out.len / nx_red;

    // ignore last coloumn
    Eigen::Map<Eigen::MatrixXd> u1(mpc_config.out.u_out.ptr, u_rows, u_cols - 1);
    Eigen::Map<Eigen::MatrixXd> x1(mpc_config.out.x_out.ptr, x_rows, x_cols - 1);

    // ignore first coloumn
    Eigen::Map<Eigen::MatrixXd> u2(mpc_config.out.u_out.ptr+nq_red, u_rows, u_cols - 1);
    Eigen::Map<Eigen::MatrixXd> x2(mpc_config.out.x_out.ptr+nx_red, x_rows, x_cols - 1);

    Eigen::MatrixXd du = u2 - u1;
    Eigen::MatrixXd dx = x2 - x1;

    Eigen::MatrixXd u_pred = u1 + du * dt;
    Eigen::MatrixXd x_pred = x1 + dx * dt;

    // write the predicted values to the init_guess
    memcpy(mpc_config.in.u.ptr, u_pred.data(), u_pred.size() * sizeof(double));
    memcpy(mpc_config.in.x.ptr, x_pred.data(), x_pred.size() * sizeof(double));

    // the last u and x cannot be predicted. Therefore, the last value is copied from the last prediction
    memcpy(mpc_config.in.u.ptr + u_pred.size(), mpc_config.out.u_out.ptr + u_pred.size(), nq_red * sizeof(double));
    memcpy(mpc_config.in.x.ptr + x_pred.size(), mpc_config.out.x_out.ptr + x_pred.size(), nx_red * sizeof(double));
}

// Method for switching the trajectory
void CasadiMPC::switch_traj(const Eigen::VectorXd &x_k)
{
    traj_data = trajectory_generator.get_traj_data();
    traj_rows = traj_data->rows();
    traj_cols = traj_data->cols();
    traj_select = trajectory_generator.get_traj_select();

    traj_data_real_len = trajectory_generator.get_traj_data_real_len();

    if (traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1] > traj_cols)
    {
        throw std::runtime_error("void CasadiMPC::switch_traj(const Eigen::MatrixXd* traj_data_new, \
                                  const Eigen::VectorXd &x_k, casadi_uint traj_data_real_len_new): \
                                  Trajectory data length + mpc_traj_indices[end] = " +
                                 std::to_string(traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1]) +
                                 " exceeds totalthrow length = " +
                                 std::to_string(traj_cols) +
                                 ". Please generate new.");
    }

    generate_trajectory_blocks();
    init_references_and_pointers();
    set_coldstart_init_guess(x_k.data());

    traj_count = 0;
}

void CasadiMPC::reset(const Eigen::VectorXd &x_k)
{
    traj_count = 0;
    set_coldstart_init_guess(x_k.data());
}

// Method to set the cold start initial guess (assuming x_k was already set)
void CasadiMPC::set_coldstart_init_guess(const casadi_real *const x_k_in)
{
    set_x_k(x_k_in);

    memset(mpc_config.in.init_guess.ptr, 0, mpc_config.in.init_guess.len * sizeof(double));

    // set all x input values to the current state
    set_row_vector(mpc_config.in.x.ptr, mpc_config.in.x_k.ptr, mpc_config.in.x_k.len, mpc_config.in.x.len);

    // set all x_prev reference values to the current state
    set_row_vector(mpc_config.in.x_prev.ptr, mpc_config.in.x_k.ptr, mpc_config.in.x_k.len, mpc_config.in.x_prev.len);
    #ifdef USE_REFERENCE_SYS_MPC
    if (mpc_config.in.z.len != 0)
    {
        const Eigen::VectorXi z_d_quat_rows = (Eigen::VectorXi(10) << trajectory_generator.p_d_rows, trajectory_generator.q_d_rows, trajectory_generator.p_d_p_rows).finished();                                     // Selecting p_d (0-2), q_d (9-12), p_d_p (3-5)
        const Eigen::VectorXi z_d_rpy_rows = (Eigen::VectorXi(12) << trajectory_generator.p_d_rows, trajectory_generator.phi_d_rows, trajectory_generator.p_d_p_rows, trajectory_generator.phi_d_p_rows).finished(); // Selecting p_d (0-2), Phi_d (19-21), p_d_p (3-5), Phi_d_p (22-24)
        const Eigen::VectorXi quat_rows = trajectory_generator.q_d_rows;
        const Eigen::VectorXi omega_rows = trajectory_generator.omega_d_rows;
        const Eigen::VectorXi omega_p_rows = trajectory_generator.omega_d_p_rows;
        const Eigen::VectorXi p_d_pp_rows = trajectory_generator.p_d_pp_rows;

        Eigen::MatrixXd z_d(mpc_config.in.z_k.len, horizon_len);
        Eigen::MatrixXd alpha_d(static_cast<int>(mpc_config.in.alpha.len / horizon_len), horizon_len);
        Eigen::MatrixXd z_k_d(mpc_config.in.z_k.len, 1);
        if (use_quat) // in case of MPC_v3_quat z_k = [p_d; p_d_p; quat_d; omega_d]
        {
            Eigen::MatrixXd quat_d = (*traj_data)(quat_rows, mpc_traj_indices.array() + 0);
            Eigen::MatrixXd omega_d = (*traj_data)(omega_rows, mpc_traj_indices.array() + 0);
            Eigen::MatrixXd omega_d_p = (*traj_data)(omega_p_rows, mpc_traj_indices.array() + 0);

            Eigen::MatrixXd quat_d_p(4, horizon_len);
            Eigen::MatrixXd quat_d_pp(4, horizon_len);

            for (uint32_t i = 0; i < horizon_len; i++)
            {
                quat_d_p.col(i) = d_dt_quat(quat_d.col(i), omega_d.col(i));
                quat_d_pp.col(i) = d_dt2_quat(quat_d.col(i), omega_d.col(i), omega_d_p.col(i));
            }

            z_d << (*traj_data)(z_d_quat_rows, mpc_traj_indices.array() + 0), quat_d_p;
            alpha_d << (*traj_data)(p_d_pp_rows, mpc_traj_indices.array() + 0), quat_d_pp;
        }
        else // In case of MPC_v3_rpy z_k = [p_d; phi_d; p_d_p; phi_d_p]
        {
            z_d << (*traj_data)(z_d_rpy_rows, mpc_traj_indices.array() + 0);
            alpha_d << (*traj_data)(y_d_pp_rpy_rows, mpc_traj_indices.array() + 0);
        }
        z_k_d = z_d.col(0);

        mpc_config.in.z_k.set(w, z_k_d.data());
        mpc_config.in.z.set(w, z_d.data());
        mpc_config.in.z_prev.set(w, z_d.data());
        mpc_config.in.alpha.set(w, alpha_d.data());
        mpc_config.in.alpha_prev.set(w, alpha_d.data());
    }
    #endif

    #ifdef USE_PATH_FOLLOWING_MPC
    if (mpc_config.in.traj_select.len != 0)
    {
        double traj_select_double = static_cast<double>(traj_select);
        Eigen::VectorXd init_guess_theta = Eigen::VectorXd(mpc_traj_indices.cast<double>()) * dt;

        mpc_config.in.t_k.set(w, init_guess_theta.data());
        mpc_config.in.traj_select.set(w, &traj_select_double);
        mpc_config.in.theta.set(w, init_guess_theta.data());
        mpc_config.in.theta_prev.set(w, init_guess_theta.data());
    }
    #endif
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// PUBLIC GETTER METHODS //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

casadi_real* CasadiMPC::get_param_ptr(std::string key)
{
    const MPCInput input_id = string_to_mpc_input(key);
    const mpc_input_entry_t *entry = get_mpc_input_entry(&mpc_config.in, static_cast<mpc_input_config_id_t>(input_id));
    if (entry != nullptr)
    {
        return entry->ptr;
    }
    else
    {
        throw std::runtime_error("CasadiMPC::get_param(): " + key + " is not a valid parameter name.");
    }
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PUBLIC SETTER METHODS ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void CasadiMPC::set_param(std::string key, const casadi_real *param_data)
{
    const MPCInput input_id = string_to_mpc_input(key);
    const mpc_input_entry_t *entry = get_mpc_input_entry(&mpc_config.in, static_cast<mpc_input_config_id_t>(input_id));
    if (entry != nullptr)
    {
        entry->set(w, param_data);
    }
    else
    {
        throw std::runtime_error("CasadiMPC::set_param(): " + key + " is not a valid parameter name.");
    }
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PRIVATE METHODS ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////                                                                             /////
/////          ||||||   ||||||   ||  ||    ||   ||||    ||||||||  ||||||          /////
/////          ||   ||  ||   ||  ||  ||    ||  ||   ||     ||     ||              /////
/////          ||||||   ||||||   ||   ||  ||   |||||||     ||     ||||||          /////
/////          ||       || ||    ||    ||||    ||   ||     ||     ||              /////
/////          ||       ||   ||  ||     ||     ||   ||     ||     ||||||          /////
/////                                                                             /////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

/*
Ideen:
Ziel: Pointer von Referenzen erstellen dazu Pointer von Funktionen. In Schleife schreiben.
Dazu muss erkannt werden, welche Referenze zu welchen Daten gehört.
Also könnte man
- Enums welche angeben was was ist
- Array aus Pointern auf die Referenzen
- Array aus Pointern auf die Daten
- Array aus Pointern auf die Funktionen

Damit erstellt man ein Struct für die Setter der Referenzen und ein Struct für die Setter der Parameter.
*/

// Method to get the reference function pointer list
void CasadiMPC::init_references_and_pointers()
{
    // Temporary vectors to hold pointers and set functions
    std::vector<const double **> temp_mpc_data(static_cast<int>(MPCInput::COUNT), nullptr);
    std::vector<CasadiSetPtr_t> temp_mpc_set_funcs(static_cast<int>(MPCInput::COUNT), nullptr);

    size_t count = 0; // Counter for valid entries

    for (int i = 0; i < static_cast<int>(MPCInput::COUNT); ++i)
    {
        MPCInput mpc_input = static_cast<MPCInput>(i);

        switch (mpc_input)
        {
        case MPCInput::x_k:
            if (mpc_config.in.x_k.len != 0)
            {
                temp_mpc_data[count] = &x_k_ptr;                   // Store pointer
                temp_mpc_set_funcs[count] = mpc_config.in.x_k.set; // Store set function
                count++;                                           // Increment count
            }
            break;
        #ifdef USE_PATH_FOLLOWING_MPC
        case MPCInput::t_k:
            if (mpc_config.in.t_k.len != 0)
            {
                temp_mpc_data[count] = &t_k_ptr;
                temp_mpc_set_funcs[count] = mpc_config.in.t_k.set;
                count++;
            }
            break;
        #endif
        case MPCInput::y_d:
            if (mpc_config.in.y_d.len != 0)
            {
                temp_mpc_data[count] = &y_d_ptr;
                temp_mpc_set_funcs[count] = mpc_config.in.y_d.set;
                count++;
            }
            break;
        #ifdef USE_REFERENCE_SYS_MPC
        case MPCInput::y_d_p:
            if (mpc_config.in.y_d_p.len != 0)
            {
                temp_mpc_data[count] = &y_d_p_ptr;
                temp_mpc_set_funcs[count] = mpc_config.in.y_d_p.set;
                count++;
            }
            break;
        case MPCInput::y_d_pp:
            if (mpc_config.in.y_d_pp.len != 0)
            {
                temp_mpc_data[count] = &y_d_pp_ptr;
                temp_mpc_set_funcs[count] = mpc_config.in.y_d_pp.set;
                count++;
            }
            break;
        #endif
        default:
            break;
        }
    }

    // Resize mpc_data and mpc_set_funcs to the count of valid entries
    mpc_data.resize(count);
    mpc_set_funcs.resize(count);

    // Copy the valid entries from the temp vector to mpc_data and mpc_set_funcs
    for (size_t i = 0; i < count; ++i)
    {
        mpc_data[i] = temp_mpc_data[i];
        mpc_set_funcs[i] = temp_mpc_set_funcs[i];
    }
}

void CasadiMPC::generate_trajectory_blocks()
{
    Eigen::MatrixXd y_d;
    Eigen::MatrixXd y_d_p(6, horizon_len);
    Eigen::MatrixXd y_d_pp(6, horizon_len);
    Eigen::VectorXd t_d(horizon_len);

    Eigen::VectorXi y_d_rows;
    Eigen::VectorXi y_d_p_rows;
    Eigen::VectorXi y_d_pp_rows;

    if (use_quat)
    {
        y_d.resize(7, horizon_len);
        y_d_rows = y_d_quat_rows;
        y_d_p_rows = y_d_p_quat_rows;
        y_d_pp_rows = y_d_pp_quat_rows;
    }
    else
    {
        y_d.resize(6, horizon_len);
        y_d_rows = y_d_rpy_rows;
        y_d_p_rows = y_d_p_rpy_rows;
        y_d_pp_rows = y_d_pp_rpy_rows;
    }

    y_d_blocks.resize(traj_cols);
    y_d_p_blocks.resize(traj_cols);
    y_d_pp_blocks.resize(traj_cols);
    t_d_arr.resize(traj_cols);

    y_d_blocks_data.resize(traj_cols);
    y_d_p_blocks_data.resize(traj_cols);
    y_d_pp_blocks_data.resize(traj_cols);

    for (uint32_t i = 0; i < traj_cols; i++)
    {
        y_d << (*traj_data)(y_d_rows, mpc_traj_indices.array() + i);
        y_d_p << (*traj_data)(y_d_p_rows, mpc_traj_indices.array() + i);
        y_d_pp << (*traj_data)(y_d_pp_rows, mpc_traj_indices.array() + i);

        y_d_blocks[i] = y_d;
        y_d_p_blocks[i] = y_d_p;
        y_d_pp_blocks[i] = y_d_pp;
        t_d_arr[i] = i * dt;

        y_d_blocks_data[i] = y_d_blocks[i].data();
        y_d_p_blocks_data[i] = y_d_p_blocks[i].data();
        y_d_pp_blocks_data[i] = y_d_pp_blocks[i].data();
    }
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// PRIVATE GETTER METHODS /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// PRIVATE SETTER METHODS /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

// Method for setting all rows of matrix_data (rows x cols) to the (rows x 1) vector row_data. (length = rows * cols)
void CasadiMPC::set_row_vector(casadi_real *matrix_data, casadi_real *row_data, casadi_uint rows, casadi_uint length)
{
    casadi_uint cols = length / rows;
    for (casadi_uint i = 0; i < cols; i++)
    {
        memcpy(matrix_data + i * rows, row_data, rows * sizeof(casadi_real));
    }
}

void CasadiMPC::set_references(const casadi_real *const x_k_in)
{
    if (traj_count < traj_data_real_len - 1)
    {
        x_k_ptr = x_k_in;
        t_k_ptr = &t_d_arr[traj_count];
        y_d_ptr = y_d_blocks_data[traj_count];
        y_d_p_ptr = y_d_p_blocks_data[traj_count];
        y_d_pp_ptr = y_d_pp_blocks_data[traj_count];

        for (casadi_uint i = 0; i < mpc_data.size(); i++)
        {
            mpc_set_funcs[i](w, *mpc_data[i]);
        }
        traj_count += traj_step;
    }
}

// Destructor to clean up allocated memory
CasadiMPC::~CasadiMPC()
{
}