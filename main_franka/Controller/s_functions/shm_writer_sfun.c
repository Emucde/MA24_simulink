#define S_FUNCTION_NAME  shm_writer_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#define SHM_DATA_SIZE 14
#define SHM_DATA_SIZE_BYTES (SHM_DATA_SIZE * sizeof(double))

#define SHM_VALID_FLAG_SIZE 1
#define SHM_VALID_FLAG_SIZE_BYTES (SHM_VALID_FLAG_SIZE * sizeof(double))

#define SHM_START_FLAG_SIZE 1
#define SHM_START_FLAG_SIZE_BYTES (SHM_START_FLAG_SIZE * sizeof(double))

#define SHM_RESET_FLAG_SIZE 1
#define SHM_RESET_FLAG_SIZE_BYTES (SHM_RESET_FLAG_SIZE * sizeof(double))

#define SHM_STOP_FLAG_SIZE 1
#define SHM_STOP_FLAG_SIZE_BYTES (SHM_STOP_FLAG_SIZE * sizeof(double))

#define INPUT_NUM 5

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, INPUT_NUM); // Two parameters: first: shared memory name of data, second: shared memory name of valid flag
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }
    if (!ssSetNumInputPorts(S, INPUT_NUM)) return;  // INPUT_NUM inputs: data and valid flag
    int shm_size[INPUT_NUM] = {SHM_DATA_SIZE, SHM_VALID_FLAG_SIZE, SHM_START_FLAG_SIZE, SHM_RESET_FLAG_SIZE, SHM_STOP_FLAG_SIZE};
    for (int i = 0; i < INPUT_NUM; i++) {
        ssSetInputPortWidth(S, i, shm_size[i]);
        ssSetInputPortDataType(S, i, SS_DOUBLE);
        ssSetInputPortDirectFeedThrough(S, i, 1);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }
    
    if (!ssSetNumOutputPorts(S, 0)) return;
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, INPUT_NUM*2);   // For INPUT_NUM*2 Shared Memory Pointers and two File Descriptors
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    int shm_size_bytes[INPUT_NUM] = {SHM_DATA_SIZE_BYTES, SHM_VALID_FLAG_SIZE_BYTES, SHM_START_FLAG_SIZE_BYTES, SHM_RESET_FLAG_SIZE_BYTES, SHM_STOP_FLAG_SIZE_BYTES};
    for (int i = 0; i < INPUT_NUM; i++) {
        char_T shm_name[256];
        mxGetString(ssGetSFcnParam(S, i), shm_name, sizeof(shm_name));
        ssPrintf("Attempting to open shared memory %d: %s\n", i, shm_name);

        int fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if (fd == -1) {
            ssPrintf("Failed to open shared memory %d: %s\n", i, strerror(errno));
            ssSetErrorStatus(S, "Failed to open shared memory");
            return;
        }
        
        if (ftruncate(fd, shm_size_bytes[i]) == -1) {
            ssPrintf("Failed to resize shared memory %d: %s\n", i, strerror(errno));
            ssSetErrorStatus(S, "Failed to resize shared memory");
            close(fd);
            return;
        }
        
        double *shared_data = (double*)mmap(0, shm_size_bytes[i], PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (shared_data == MAP_FAILED) {
            ssPrintf("Failed to map shared memory %d: %s\n", i, strerror(errno));
            ssSetErrorStatus(S, "Failed to map shared memory");
            close(fd);
            return;
        }
        
        memset(shared_data, 0, shm_size_bytes[i]);
        
        ssPrintf("Shared memory %d created and mapped successfully\n", i);
        ssSetPWorkValue(S, i*2, shared_data);
        ssSetPWorkValue(S, i*2+1, (void*)(intptr_t)fd);
    }
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    //ssPrintf("mdlOutputs called\n");
    
    if (ssGetNumInputPorts(S) == 0) {
        ssPrintf("Error: No input ports defined\n");
        return;
    }
    
    //ssPrintf("Number of input ports: %d\n", ssGetNumInputPorts(S));
    //ssPrintf("Input port width: %d\n", ssGetInputPortWidth(S, 0));
    
    int shm_size_bytes[INPUT_NUM] = {SHM_DATA_SIZE_BYTES, SHM_VALID_FLAG_SIZE_BYTES, SHM_START_FLAG_SIZE_BYTES, SHM_RESET_FLAG_SIZE_BYTES, SHM_STOP_FLAG_SIZE_BYTES};
    for (int i = 0; i < INPUT_NUM; i++) {
        double *u = (double*)ssGetInputPortSignal(S, i);
        double *shared_data = (double*)ssGetPWorkValue(S, i*2);
        
        if (!u) {
            ssPrintf("Error: Input signal pointer is null for input %d\n", i);
            return;
        }
        
        if (!shared_data) {
            ssPrintf("Error: Shared memory pointer is null for input %d\n", i);
            return;
        }
    
    //ssPrintf("Input signal values: %f, %f, %f\n", u[0], u[1], u[2]);
    
        memcpy(shared_data, u, shm_size_bytes[i]);
    }
    
    //ssPrintf("Shared memory values: %f, %f, %f\n", shared_data[0], shared_data[1], shared_data[2]);
}

static void mdlTerminate(SimStruct *S)
{
    for (int i = 0; i < INPUT_NUM; i++) {
        double *shared_data = (double*)ssGetPWorkValue(S, i*2);
        int fd = (int)(intptr_t)ssGetPWorkValue(S, i*2+1);
        
        if (shared_data) {
            munmap(shared_data, SHM_DATA_SIZE_BYTES);
        }
        if (fd != -1) {
            close(fd);
        }
    }
}

#ifdef  MATLAB_MEX_FILE    
#include "simulink.c"      
#else
#include "cg_sfun.h"       
#endif