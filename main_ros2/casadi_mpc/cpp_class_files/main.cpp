/*
This file contains the main entry point for the CasADi MPC application for offline simulation or shared memory mode.
It initializes the SharedMemoryController and runs either the simulation or shared memory mode based on command line
arguments.
*/

#include <iostream>
#include "SharedMemoryController.hpp"
#include "SharedMemory.hpp"
#include <semaphore.h>
#include <Eigen/Dense>

#define USE_CUSTOM_LIST true
#define USE_SHARED_MEMORY 1
#define RUN_SIMULATION 1

int main(int argc, char *argv[])
{
    bool run_simulation = false;
    if (argc > 1 && std::string(argv[1]) == "run")
    {
        run_simulation = true;
    }

    SharedMemoryController shared_memory_controller(MASTERDIR, USE_CUSTOM_LIST);
    if (run_simulation)
    {
        std::cout << "Running simulation mode" << std::endl;
        shared_memory_controller.run_simulation();
    }
    else
    {
        std::cout << "Running shared memory mode" << std::endl;
        while(true)
        {
            shared_memory_controller.run_shm_mode();
        }
    }
}
