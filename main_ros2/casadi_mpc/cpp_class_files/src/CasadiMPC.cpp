#include "CasadiMPC.hpp"
#include <cstring> // for memcpy
#include <iostream>
#include <fstream>
#include <vector>
#include <memory> // for std::make_unique
#include "eigen_templates.hpp"

// #define DEBUG 1

mpc_config_t invalid_config(const std::string &mpc_name)
{
    throw std::runtime_error("CasadiMPC.cpp: \"" + mpc_name + "\" is not a valid MPC name. Use one of the following: MPC01, MPC6, MPC7, MPC8, MPC9, MPC10, MPC11, MPC12, MPC13, MPC14");
    return {};
}

// Constructor implementation
CasadiMPC::CasadiMPC(MPCType mpc,
                     robot_config_t &robot_config,
                     const Eigen::MatrixXd *traj_data,
                     const casadi_uint traj_data_real_len) : mpc_name(mpctype_to_string(mpc)),
                                                             mpc_config(get_MPC_config(mpc)),
                                                             robot_config(robot_config),
                                                             traj_data(traj_data),
                                                             traj_data_real_len(traj_data_real_len),
                                                             is_kinematic_mpc(mpc_config.kinematic_mpc == 1),
                                                             nq(robot_config.nq), nx(robot_config.nx), nq_red(robot_config.nq_red), nx_red(robot_config.nx_red),
                                                             casadi_fun(mpc_config.casadi_fun),
                                                             arg(mpc_config.arg), res(mpc_config.res), iw(mpc_config.iw), w(mpc_config.w), w_end(mpc_config.w_end),
                                                             horizon_len(mpc_config.traj_data_per_horizon),
                                                             mpc_traj_indices(ConstIntVectorMap(mpc_config.traj_indices, horizon_len)),
                                                             traj_data_per_horizon(mpc_config.traj_data_per_horizon),
                                                             traj_count(0), traj_select(1),
                                                             mem(mpc_config.mem), dt(robot_config.dt)
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

    // init the reference values vector
    generate_trajectory_blocks();
    init_references_and_pointers();

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
int CasadiMPC::solve(casadi_real *x_k_in)
{
    // Copy the state to the MPC object

    set_references(x_k_in);

#ifdef DEBUG
    std::cout << "int CasadiMPC::solve()\nw:\n";
    for (int i = 0; i < int(mpc_config.in.init_guess.len + mpc_config.in.reference_values.len + mpc_config.in.param_weight.len); i++)
    {
        std::cout << w[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "MPC object created." << std::endl
              << std::endl;
#endif

    // Call the Casadi function
    int flag = casadi_fun(arg, res, iw, w_end, mem);

    // Set initial guess and prev ref values
    if (!flag)
    {
        mpc_config.set_prev_to_out(w);
        mpc_config.set_init_guess_out_to_in(w);
    }
    else
    {
        std::cerr << "Error in Casadi function call." << std::endl;
    }

    return flag;
}

// int CasadiMPC::solve(casadi_real *x_k_in)
// {
//     // Copy the initial state
//     memcpy(x_k, x_k_in, nx_red * sizeof(casadi_real));

//     return CasadiMPC::solve();
// }

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
        // memcpy(x_k, w + mpc_config.out.x_out.ptr + nx_red, nx_red * sizeof(casadi_real));
#ifdef DEBUG
        if (traj_count % 100 == 0)
        {
            double *x_k = mpc_config.in.x_k.ptr;
            double *u_opt = mpc_config.out.u_opt.ptr;
            std::cout << "x_k: " << x_k[0] << " " << x_k[1] << " " << x_k[2] << " " << x_k[3] << " " << x_k[4] << " " << x_k[5] << " " << x_k[6] << std::endl;

            for (int i = 0; i < 24; i++)
            {
                std::cout << u_opt[i] << " ";
            }
            std::cout << std::endl;
        }
#endif
    }
    else
    {
        std::cerr << "Error in Casadi function call." << std::endl;
    }

    return flag;
}

// Method for switching the trajectory
void CasadiMPC::switch_traj(const Eigen::MatrixXd *traj_data_new, const casadi_real *const x_k_ptr, casadi_uint traj_data_real_len_new)
{
    traj_data = traj_data_new;
    traj_rows = traj_data->rows();
    traj_cols = traj_data->cols();

    traj_data_real_len = traj_data_real_len_new;

    if (traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1] > traj_cols)
    {
        throw std::runtime_error("void CasadiMPC::switch_traj(const Eigen::MatrixXd* traj_data_new, \
                                  const casadi_real *const x_k_ptr, casadi_uint traj_data_real_len_new): \
                                  Trajectory data length + mpc_traj_indices[end] = " +
                                 std::to_string(traj_data_real_len + mpc_traj_indices[traj_data_per_horizon - 1]) +
                                 " exceeds totalthrow length = " +
                                 std::to_string(traj_cols) +
                                 ". Please generate new.");
    }

    generate_trajectory_blocks();
    init_references_and_pointers();
    set_coldstart_init_guess(x_k_ptr);

    traj_count = 0;
}

void CasadiMPC::reset()
{
    traj_count = 0;
    set_coldstart_init_guess(x_k_ptr);
}

// Method to set the cold start initial guess (assuming x_k was already set)
void CasadiMPC::set_coldstart_init_guess(const casadi_real *const x_k_ptr)
{
    set_x_k(x_k_ptr);

    memset(mpc_config.in.init_guess.ptr, 0, mpc_config.in.init_guess.len * sizeof(double));

    // set all x input values to the current state
    set_row_vector(mpc_config.in.x.ptr, mpc_config.in.x_k.ptr, nx_red, mpc_config.in.x.len);

    // set all x_prev reference values to the current state
    set_row_vector(mpc_config.in.x_prev.ptr, mpc_config.in.x_k.ptr, nx_red, mpc_config.in.x_prev.len);
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// PUBLIC GETTER METHODS //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PUBLIC SETTER METHODS ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

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
    std::vector<double**> temp_mpc_data(static_cast<int>(MPCInput::COUNT), nullptr);
    std::vector<CasadiIOPtr_t> temp_mpc_set_funcs(static_cast<int>(MPCInput::COUNT), nullptr);

    size_t count = 0; // Counter for valid entries

    for (int i = 0; i < static_cast<int>(MPCInput::COUNT); ++i)
    {
        MPCInput mpc_input = static_cast<MPCInput>(i);

        switch (mpc_input)
        {
        case MPCInput::x_k:
            if (mpc_config.in.x_k.len != 0)
            {
                temp_mpc_data[count] = &x_k_ptr; // Store pointer
                temp_mpc_set_funcs[count] = mpc_config.in.x_k.set; // Store set function
                count++; // Increment count
            }
            break;
        case MPCInput::t_k:
            if (mpc_config.in.t_k.len != 0)
            {
                temp_mpc_data[count] = &t_k_ptr;
                temp_mpc_set_funcs[count] = mpc_config.in.t_k.set;
                count++;
            }
            break;
        case MPCInput::y_d:
            if (mpc_config.in.y_d.len != 0)
            {
                temp_mpc_data[count] = &y_d_ptr;
                temp_mpc_set_funcs[count] = mpc_config.in.y_d.set;
                count++;
            }
            break;
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
    Eigen::MatrixXd y_d(7, horizon_len);
    Eigen::MatrixXd y_d_p(6, horizon_len);
    Eigen::MatrixXd y_d_pp(6, horizon_len);

    y_d_blocks.resize(traj_cols);
    y_d_p_blocks.resize(traj_cols);
    y_d_pp_blocks.resize(traj_cols);

    y_d_blocks_data.resize(traj_cols);
    y_d_p_blocks_data.resize(traj_cols);
    y_d_pp_blocks_data.resize(traj_cols);

    for( uint32_t i = 0; i < traj_cols; i++)
    {
        y_d << (*traj_data)(y_d_rows, mpc_traj_indices.array() + i);
        y_d_p << (*traj_data)(y_d_p_rows, mpc_traj_indices.array() + i);
        y_d_pp << (*traj_data)(y_d_pp_rows, mpc_traj_indices.array() + i);
        y_d_blocks[i] = y_d;
        y_d_p_blocks[i] = y_d_p;
        y_d_pp_blocks[i] = y_d_pp;

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

void CasadiMPC::set_references(casadi_real *x_k_in)
{
    if (traj_count < traj_data_real_len - 1)
    {
        x_k_ptr = x_k_in;
        t_k = dt*traj_count;
        y_d_ptr = y_d_blocks_data[traj_count];
        y_d_p_ptr = y_d_p_blocks_data[traj_count];
        y_d_pp_ptr = y_d_pp_blocks_data[traj_count];

        for (casadi_uint i = 0; i < mpc_data.size(); i++)
        {
            mpc_set_funcs[i](w, *mpc_data[i]);
        }
        traj_count++;
    }
}

// Destructor to clean up allocated memory
CasadiMPC::~CasadiMPC()
{
}