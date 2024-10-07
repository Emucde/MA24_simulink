#define S_FUNCTION_NAME  shm_writer_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#define SHM_SIZE 800  // Größe für 100 double-Werte

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 1);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 100);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S, 0)) return;
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 2);
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
    char_T shm_name[256];
    mxGetString(ssGetSFcnParam(S, 0), shm_name, sizeof(shm_name));
    ssPrintf("Attempting to open shared memory: %s\n", shm_name);

    int fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
        ssPrintf("Failed to open shared memory: %s\n", strerror(errno));
        ssSetErrorStatus(S, "Failed to open shared memory");
        return;
    }
    
    if (ftruncate(fd, SHM_SIZE) == -1) {
        ssPrintf("Failed to resize shared memory: %s\n", strerror(errno));
        ssSetErrorStatus(S, "Failed to resize shared memory");
        close(fd);
        return;
    }
    
    double *shared_data = (double*)mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED) {
        ssPrintf("Failed to map shared memory: %s\n", strerror(errno));
        ssSetErrorStatus(S, "Failed to map shared memory");
        close(fd);
        return;
    }
    
    // Initialisiere den Shared Memory mit Nullen
    memset(shared_data, 0, SHM_SIZE);
    
    ssPrintf("Shared memory created and mapped successfully\n");
    ssSetPWorkValue(S, 0, shared_data);
    ssSetPWorkValue(S, 1, (void*)(intptr_t)fd);
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
    
    double *u = (double*)ssGetInputPortSignal(S, 0);
    double *shared_data = (double*)ssGetPWorkValue(S, 0);
    
    if (!u) {
        ssPrintf("Error: Input signal pointer is null\n");
        return;
    }
    
    if (!shared_data) {
        ssPrintf("Error: Shared memory pointer is null\n");
        return;
    }
    
    //ssPrintf("Input signal values: %f, %f, %f\n", u[0], u[1], u[2]);
    
    memcpy(shared_data, u, 100 * sizeof(double));
    
    //ssPrintf("Shared memory values: %f, %f, %f\n", shared_data[0], shared_data[1], shared_data[2]);
}

static void mdlTerminate(SimStruct *S)
{
    double *shared_data = (double*)ssGetPWorkValue(S, 0);
    int fd = (int)(intptr_t)ssGetPWorkValue(S, 1);
    
    if (shared_data) {
        munmap(shared_data, SHM_SIZE);
    }
    if (fd != -1) {
        close(fd);
    }
}

#ifdef  MATLAB_MEX_FILE    
#include "simulink.c"      
#else
#include "cg_sfun.h"       
#endif