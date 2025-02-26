#include <iostream>
#include <fstream>
#include <vector>
#include <cstring> // for memcpy

#include "SharedMemoryController.hpp"

#include "include/FullSystemTorqueMapper.hpp"
#include "include/CasadiMPC.hpp"
#include "include/CasadiController.hpp"
#include "mpc_config_types.h"
#include "param_robot.h"
#include "casadi_types.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include "include/eigen_templates.hpp"
#include "include/TicToc.hpp"
#include "include/SignalFilter.hpp"
#include "include/SharedMemory.hpp"
#include "param_robot.h"
#include "trajectory_settings.hpp"
#include "WorkspaceController.hpp"
#include "CrocoddylController.hpp"
#include "CrocoddylMPCType.hpp"
#include "CasadiEKF.hpp"
#include "CommonBaseController.hpp"
#include "json.hpp"
#include <random>
#include <cmath>

#define CUSTOM_LIST 1
#define USE_SHARED_MEMORY 1

int main()
{
    SharedMemoryController shared_memory_controller(MASTERDIR, true);
    shared_memory_controller.run_simulation();
}