#ifndef CASADICONTROLLER_TYPES_HPP
#define CASADICONTROLLER_TYPES_HPP

enum class MPCType
{
    MPC01,
    MPC6,
    MPC7,
    MPC8,
    MPC9,
    MPC10,
    MPC11,
    MPC12,
    MPC13,
    MPC14,
    INVALID, // Consider using INVALID for out-of-bounds
    COUNT    // This can denote the number of valid enum values
};

enum ErrorFlag {
    NO_ERROR = 0,           // 0: no error
    JUMP_DETECTED = 1,      // 1: jump in torque detected
    NAN_DETECTED = 2,       // 2: NaN in torque detected
    CASADI_ERROR = 3        // 3: error in Casadi function call
};

#endif // CASADICONTROLLER_TYPES_HPP