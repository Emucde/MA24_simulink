#define S_FUNCTION_NAME  shm_reader_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define SHM_SIZE 2000  // Größe in Bytes, anpassen nach Bedarf

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 1);  // Ein Parameter: Name des Shared Memory
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 0)) return;
    
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 250);  // 250 double-Werte
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 2);  // Für Shared Memory Pointer und Dateideskriptor
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
    char_T *shm_name = mxArrayToString(ssGetSFcnParam(S, 0));
    int fd = shm_open(shm_name, O_RDONLY, 0666);
    if (fd == -1) {
        ssSetErrorStatus(S, "Failed to open shared memory");
        return;
    }
    
    double *shared_data = (double*)mmap(0, SHM_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED) {
        ssSetErrorStatus(S, "Failed to map shared memory");
        close(fd);
        return;
    }
    
    ssSetPWorkValue(S, 0, shared_data);
    ssSetPWorkValue(S, 1, (void*)(intptr_t)fd);
    mxFree(shm_name);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    double *y = ssGetOutputPortSignal(S, 0);
    double *shared_data = (double*)ssGetPWorkValue(S, 0);
    
    for (int i = 0; i < 250; i++) {
        y[i] = shared_data[i];
    }
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

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
