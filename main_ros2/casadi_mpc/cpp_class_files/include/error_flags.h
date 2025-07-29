#ifndef ERROR_FLAGS_H
#define ERROR_FLAGS_H

/*
Error flags for the MPC system.
These flags are used to indicate various error states that can occur during the operation of the MPC.
Each flag corresponds to a specific type of error, allowing for easier debugging and error handling.
*/
enum class ErrorFlag {
    NO_ERROR = 0,           // 0: no error
    JUMP_DETECTED = 1,      // 1: jump in torque detected
    NAN_DETECTED = 2,       // 2: NaN in torque detected
    CASADI_ERROR = 3,        // 3: error in Casadi function call
    CROCODDYL_ERROR = 4     // 4: error in Crocoddyl function call
};

#endif // ERROR_FLAGS_H