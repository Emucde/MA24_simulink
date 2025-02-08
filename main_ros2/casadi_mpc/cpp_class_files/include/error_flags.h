#ifndef ERROR_FLAGS_H
#define ERROR_FLAGS_H

enum class ErrorFlag {
    NO_ERROR = 0,           // 0: no error
    JUMP_DETECTED = 1,      // 1: jump in torque detected
    NAN_DETECTED = 2,       // 2: NaN in torque detected
    CASADI_ERROR = 3        // 3: error in Casadi function call
};

#endif // ERROR_FLAGS_H