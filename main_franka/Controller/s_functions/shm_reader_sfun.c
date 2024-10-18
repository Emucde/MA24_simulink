#define S_FUNCTION_NAME  shm_reader_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define SHM_DATA_SIZE 6
#define SHM_DATA_SIZE_BYTES (SHM_DATA_SIZE * sizeof(double))

#define SHM_VALID_FLAG_SIZE 1
#define SHM_VALID_FLAG_SIZE_BYTES (SHM_VALID_FLAG_SIZE * sizeof(SS_DOUBLE))

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 2);  // Two parameters: first: shared memory name of data, second: shared memory name of valid flag
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 0)) return;
    
    if (!ssSetNumOutputPorts(S, 2)) return;
    ssSetOutputPortWidth(S, 0, SHM_DATA_SIZE);  // 250 double values for first output
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);

    ssSetOutputPortWidth(S, 1, SHM_VALID_FLAG_SIZE);  // 250 double values for second output
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 4);  // For two Shared Memory Pointers and two File Descriptors
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
    int shm_size_bytes[2] = {SHM_DATA_SIZE_BYTES, SHM_VALID_FLAG_SIZE_BYTES};
    char shm_name[256];
    // Open two shared memory and map it to the process
    for (int i = 0; i < 2; i++) {
        // char_T *shm_name = mxArrayToString(ssGetSFcnParam(S, i));
        // shm_name = (char*) shm_name_list[i];
        mxGetString(ssGetSFcnParam(S, i), shm_name, sizeof(shm_name));
        int fd = shm_open(shm_name, O_RDONLY, 0666);
        if (fd == -1) {
            ssSetErrorStatus(S, "Failed to open shared memory");
            return;
        }
        
        double *shared_data = (double*)mmap(0, shm_size_bytes[i], PROT_READ, MAP_SHARED, fd, 0);
        if (shared_data == MAP_FAILED) {
            ssSetErrorStatus(S, "Failed to map shared memory");
            close(fd);
            return;
        }
        
        ssSetPWorkValue(S, i*2, shared_data);
        ssSetPWorkValue(S, i*2+1, (void*)(intptr_t)fd);
        // mxFree(shm_name); // this commands causes a crash in realtime environment
    }
}

// int firstrun_flag = 1;

static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Copy the shared memory data to the output
    double *y = 0;
    double *shared_data = 0;
    int shm_size[2] = {SHM_DATA_SIZE, SHM_VALID_FLAG_SIZE};
    int i,j;

    for (i = 0; i < 2; i++) {
        y = (double*)ssGetOutputPortSignal(S, i);
        shared_data = (double*)ssGetPWorkValue(S, i*2);
        
        for (j = 0; j < shm_size[i]; j++) {
            y[j] = shared_data[j];
        }
    }
}

static void mdlTerminate(SimStruct *S)
{
    int shm_size_bytes[2] = {SHM_DATA_SIZE_BYTES, SHM_VALID_FLAG_SIZE_BYTES};
    // Unmap and close the shared memory
    for (int i = 0; i < 2; i++) {
        double *shared_data = (double*)ssGetPWorkValue(S, i*2);
        int fd = (int)(intptr_t)ssGetPWorkValue(S, i*2+1);
        
        if (shared_data) {
            munmap(shared_data, shm_size_bytes[i]);
        }
        if (fd != -1) {
            close(fd);
        }
    }
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
