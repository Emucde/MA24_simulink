###########################################################################
## Makefile generated for component 'realtime_simu_franka_fr3'. 
## 
## Makefile     : realtime_simu_franka_fr3.mk
## Generated on : Tue Oct 29 16:56:04 2024
## Final product: $(RELATIVE_PATH_TO_ANCHOR)/realtime_simu_franka_fr3
## Product type : executable
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile

PRODUCT_NAME              = realtime_simu_franka_fr3
MAKEFILE                  = realtime_simu_franka_fr3.mk
MATLAB_ROOT               = /usr/local/MATLAB/R2022b
MATLAB_BIN                = /usr/local/MATLAB/R2022b/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/glnxa64
START_DIR                 = /home/rslstudent/Students/Emanuel/MA24_simulink
SOLVER                    = 
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
TGT_FCN_LIB               = ISO_C
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 0
RELATIVE_PATH_TO_ANCHOR   = ..
C_STANDARD_OPTS           = -fwrapv
CPP_STANDARD_OPTS         = -fwrapv

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Franka Simulink | gmake (64-bit linux)
# Supported Version(s):    4.4.x
# ToolchainInfo Version:   2022b
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# C_STANDARD_OPTS
# CPP_STANDARD_OPTS
# ANSI_OPTS
# CPP_ANSI_OPTS

#-----------
# MACROS
#-----------

WARN_FLAGS                 = -Wall -W -Wwrite-strings -Winline -Wstrict-prototypes -Wnested-externs -Wpointer-arith -Wcast-align
WARN_FLAGS_MAX             = $(WARN_FLAGS) -Wcast-qual -Wshadow
CPP_WARN_FLAGS             = -Wall -W -Wwrite-strings -Winline -Wpointer-arith -Wcast-align
CPP_WARN_FLAGS_MAX         = $(CPP_WARN_FLAGS) -Wcast-qual -Wshadow
MW_BIN_DIR                 = $(MATLAB_ROOT)/bin/glnxa64
CPP_17_OPTS                = -std=c++17
FRANKA_INCLUDE_DIRECTORIES = -I/home/rslstudent/Students/Emanuel/MA24_simulink/main_franka/franka_matlab_v0.3.1/libfranka/include -I/home/rslstudent/Students/Emanuel/MA24_simulink/main_franka/franka_matlab_v0.3.1/libfranka/examples -I/home/rslstudent/Students/Emanuel/MA24_simulink/main_franka/franka_matlab_v0.3.1/libfranka/common/include -I/home/rslstudent/Students/Emanuel/MA24_simulink/main_franka/franka_matlab_v0.3.1/libfranka/src -I/home/rslstudent/Students/Emanuel/MA24_simulink/main_franka/franka_matlab_v0.3.1/libfranka/build -I/home/rslstudent/Students/Emanuel/MA24_simulink/main_franka/franka_matlab_v0.3.1/franka_simulink_library/src -I/home/rslstudent/Students/Emanuel/MA24_simulink/main_franka/franka_matlab_v0.3.1/franka_simulink_library/blocks -I/usr/local/MATLAB/R2022b/toolbox/coder/rtiostream/src -I/usr/local/MATLAB/R2022b/toolbox/coder/rtiostream/src/utils -I/usr/include/eigen3

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = /home/rslstudent/Students/Emanuel/MA24_simulink/main_franka/franka_matlab_v0.3.1/libfranka/build/examples/libexamples_common.a /home/rslstudent/Students/Emanuel/MA24_simulink/main_franka/franka_matlab_v0.3.1/libfranka/build/libfranka.so -lPocoNet -lPocoFoundation -lPocoUtil -lmx -lmex -lmat -lm -lstdc++ -lpthread -lrt -L/home/rslstudent/Students/Emanuel/casadi-3.6.6-linux64-matlab2018b -I/home/rslstudent/Students/Emanuel/casadi-3.6.6-linux64-matlab2018b/include -lcasadi

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: GNU C Compiler
CC = gcc

# Linker: GNU Linker
LD = gcc

# C++ Compiler: GNU C++ Compiler
CPP = g++

# C++ Linker: GNU C++ Linker
CPP_LD = gcc

# Archiver: GNU Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = %MATLAB%/bin/glnxa64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @rm -f
ECHO                = @echo
MV                  = @mv
RUN                 =

#----------------------------------------
# "Faster Builds" Build Configuration
#----------------------------------------

MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =



#---------------------------
# Model-Specific Options
#---------------------------

CFLAGS = -c -fPIC -L/home/rslstudent/Students/Emanuel/casadi-3.6.6-linux64-matlab2018b -I/home/rslstudent/Students/Emanuel/casadi-3.6.6-linux64-matlab2018b/include -lcasadi $(ANSI_OPTS) $(FRANKA_INCLUDE_DIRECTORIES) -O3 -march=native -fno-loop-optimize -g

LDFLAGS = -Wl,-rpath,"$(MW_BIN_DIR)",-L"$(MW_BIN_DIR)" -g

SHAREDLIB_LDFLAGS = -shared -Wl,-rpath,"$(MW_BIN_DIR)",-L"$(MW_BIN_DIR)" -g

CPPFLAGS = -c -fPIC $(CPP_17_OPTS) $(FRANKA_INCLUDE_DIRECTORIES) -O3 -march=native -fno-loop-optimize -g

CPP_LDFLAGS = -Wl,-rpath,"$(MW_BIN_DIR)",-L"$(MW_BIN_DIR)" -g

CPP_SHAREDLIB_LDFLAGS = -shared -Wl,-rpath,"$(MW_BIN_DIR)",-L"$(MW_BIN_DIR)" -g

ARFLAGS = ruvs

DOWNLOAD_FLAGS = 

EXECUTE_FLAGS = 

MAKE_FLAGS = -f $(MAKEFILE)

###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(RELATIVE_PATH_TO_ANCHOR)/realtime_simu_franka_fr3
PRODUCT_TYPE = "executable"
BUILD_TYPE = "Top-Level Standalone Executable"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I$(MATLAB_ROOT)/toolbox/eml/externalDependency/timefun -I$(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/simulink/include -I$(MATLAB_ROOT)/rtw/c/src -I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/common -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/src -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/transport/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/transport/src -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/platform/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/platform/default -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src -I$(MATLAB_ROOT)/toolbox/coder/rtiostream/src

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_BUILD_ARGS = -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DEXT_MODE=1 -DMAT_FILE=1 -DONESTEPFCN=1 -DTERMFCN=1 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=0
DEFINES_CUSTOM = 
DEFINES_OPTS = -DXCP_DAQ_SUPPORT -DXCP_CALIBRATION_SUPPORT -DXCP_TIMESTAMP_SUPPORT -DXCP_TIMESTAMP_BASED_ON_SIMULATION_TIME -DXCP_SET_MTA_SUPPORT -DEXTMODE_XCP_TRIGGER_SUPPORT -DINTERNAL_XCP_MEM_BLOCK_1_SIZE=128 -DINTERNAL_XCP_MEM_BLOCK_1_NUMBER=1 -DINTERNAL_XCP_MEM_BLOCK_2_SIZE=144 -DINTERNAL_XCP_MEM_BLOCK_2_NUMBER=4 -DINTERNAL_XCP_MEM_BLOCK_3_SIZE=384 -DINTERNAL_XCP_MEM_BLOCK_3_NUMBER=4 -DINTERNAL_XCP_MEM_RESERVED_POOLS_TOTAL_SIZE=42430 -DINTERNAL_XCP_MEM_RESERVED_POOLS_NUMBER=5 -DXCP_MEM_DAQ_RESERVED_POOL_BLOCKS_NUMBER=3 -DXCP_MEM_DAQ_RESERVED_POOLS_NUMBER=1 -DXCP_MIN_EVENT_NO_RESERVED_POOL=2 -DXCP_EXTMODE_RUN_BACKGROUND_FLUSH -DEXTMODE_STATIC -DEXTMODE_STATIC_SIZE=1000000 -DON_TARGET_WAIT_FOR_START=1 -DTID01EQ=1
DEFINES_STANDARD = -DMODEL=realtime_simu_franka_fr3 -DNUMST=2 -DNCSTATES=0 -DHAVESTDIO -DRT -DUSE_RTMODEL

DEFINES = $(DEFINES_BUILD_ARGS) $(DEFINES_CUSTOM) $(DEFINES_OPTS) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(MATLAB_ROOT)/toolbox/eml/externalDependency/timefun/coder_posix_time.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_work.c $(MATLAB_ROOT)/rtw/c/src/rt_matrx.c $(MATLAB_ROOT)/rtw/c/src/rt_printf.c $(MATLAB_ROOT)/rtw/c/src/rt_logging.c $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/realtime_simu_franka_fr3.cpp $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/realtime_simu_franka_fr3_data.cpp $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/rtGetInf.cpp $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/rtGetNaN.cpp $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/rt_nonfinite.cpp $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_common.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_classic_trigger.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/src/xcp.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/src/xcp_standard.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/src/xcp_daq.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/src/xcp_calibration.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/transport/src/xcp_fifo.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/transport/src/xcp_transport.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/platform/default/xcp_mem_default.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/platform/default/xcp_drv_rtiostream.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/common/xcp_utils.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/transport/src/xcp_frame_tcp.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_param_default_tcp.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/platform/default/xcp_platform_default.c $(MATLAB_ROOT)/toolbox/coder/rtiostream/src/rtiostreamtcpip/rtiostream_tcpip.c $(START_DIR)/s_functions/fr3_no_hand_6dof/s_function_opti_MPC8.c $(START_DIR)/main_franka/Controller/s_functions/s_function_opti_robot_model_bus_fun.c $(START_DIR)/main_franka/Controller/s_functions/shm_reader_sfun.c $(START_DIR)/main_franka/Controller/s_functions/shm_writer_sfun.c $(START_DIR)/main_franka/franka_matlab_v0.3.1/franka_simulink_library/src/sizes_codegen.cpp $(START_DIR)/main_franka/franka_matlab_v0.3.1/franka_simulink_library/src/gripper_api.cpp $(START_DIR)/main_franka/franka_matlab_v0.3.1/franka_simulink_library/src/robot_api.cpp $(START_DIR)/main_franka/franka_matlab_v0.3.1/franka_simulink_library/src/control_modes.cpp $(START_DIR)/s_functions/fr3_no_hand_6dof/MPC8.c $(START_DIR)/main_franka/Controller/s_functions/robot_model_bus_fun.c

MAIN_SRC = $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/rt_main.cpp

ALL_SRCS = $(SRCS) $(MAIN_SRC)

###########################################################################
## OBJECTS
###########################################################################

OBJS = coder_posix_time.o xcp_ext_work.o rt_matrx.o rt_printf.o rt_logging.o realtime_simu_franka_fr3.o realtime_simu_franka_fr3_data.o rtGetInf.o rtGetNaN.o rt_nonfinite.o xcp_ext_common.o xcp_ext_classic_trigger.o xcp.o xcp_standard.o xcp_daq.o xcp_calibration.o xcp_fifo.o xcp_transport.o xcp_mem_default.o xcp_drv_rtiostream.o xcp_utils.o xcp_frame_tcp.o xcp_ext_param_default_tcp.o xcp_platform_default.o rtiostream_tcpip.o s_function_opti_MPC8.o s_function_opti_robot_model_bus_fun.o shm_reader_sfun.o shm_writer_sfun.o sizes_codegen.o gripper_api.o robot_api.o control_modes.o MPC8.o robot_model_bus_fun.o

MAIN_OBJ = rt_main.o

ALL_OBJS = $(OBJS) $(MAIN_OBJ)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build buildobj clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


buildobj : prebuild $(OBJS) $(PREBUILT_OBJS)
	@echo "### Successfully generated all binary outputs."


prebuild : 


download : $(PRODUCT)


execute : download
	@echo "### Invoking postbuild tool "Execute" ..."
	$(EXECUTE) $(EXECUTE_FLAGS)
	@echo "### Done invoking postbuild tool."


###########################################################################
## FINAL TARGET
###########################################################################

#-------------------------------------------
# Create a standalone executable            
#-------------------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS) $(MAIN_OBJ)
	@echo "### Creating standalone executable "$(PRODUCT)" ..."
	$(CPP_LD) $(CPP_LDFLAGS) -o $(PRODUCT) $(OBJS) $(MAIN_OBJ) $(SYSTEM_LIBS) $(TOOLCHAIN_LIBS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/s_functions/fr3_no_hand_6dof/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(START_DIR)/s_functions/fr3_no_hand_6dof/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/main_franka/Controller/s_functions/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(START_DIR)/main_franka/Controller/s_functions/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/main_franka/franka_matlab_v0.3.1/franka_simulink_library/blocks/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(START_DIR)/main_franka/franka_matlab_v0.3.1/franka_simulink_library/blocks/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/src/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/transport/src/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/transport/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/common/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/common/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/platform/default/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/platform/default/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/rtiostream/src/rtiostreamtcpip/%.c
	$(CC) $(CFLAGS) -o"$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/rtiostream/src/rtiostreamtcpip/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


coder_posix_time.o : $(MATLAB_ROOT)/toolbox/eml/externalDependency/timefun/coder_posix_time.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_ext_work.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_work.c
	$(CC) $(CFLAGS) -o"$@" "$<"


rt_matrx.o : $(MATLAB_ROOT)/rtw/c/src/rt_matrx.c
	$(CC) $(CFLAGS) -o"$@" "$<"


rt_printf.o : $(MATLAB_ROOT)/rtw/c/src/rt_printf.c
	$(CC) $(CFLAGS) -o"$@" "$<"


rt_logging.o : $(MATLAB_ROOT)/rtw/c/src/rt_logging.c
	$(CC) $(CFLAGS) -o"$@" "$<"


realtime_simu_franka_fr3.o : $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/realtime_simu_franka_fr3.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


realtime_simu_franka_fr3_data.o : $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/realtime_simu_franka_fr3_data.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetInf.o : $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/rtGetInf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetNaN.o : $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/rtGetNaN.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rt_main.o : $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/rt_main.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rt_nonfinite.o : $(START_DIR)/realtime_simu_franka_fr3_franka_emika_rtw/rt_nonfinite.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xcp_ext_common.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_common.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_ext_classic_trigger.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_classic_trigger.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/src/xcp.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_standard.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/src/xcp_standard.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_daq.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/src/xcp_daq.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_calibration.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/protocol/src/xcp_calibration.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_fifo.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/transport/src/xcp_fifo.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_transport.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/transport/src/xcp_transport.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_mem_default.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/platform/default/xcp_mem_default.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_drv_rtiostream.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/platform/default/xcp_drv_rtiostream.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_utils.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/common/xcp_utils.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_frame_tcp.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/transport/src/xcp_frame_tcp.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_ext_param_default_tcp.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_param_default_tcp.c
	$(CC) $(CFLAGS) -o"$@" "$<"


xcp_platform_default.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/slave/platform/default/xcp_platform_default.c
	$(CC) $(CFLAGS) -o"$@" "$<"


rtiostream_tcpip.o : $(MATLAB_ROOT)/toolbox/coder/rtiostream/src/rtiostreamtcpip/rtiostream_tcpip.c
	$(CC) $(CFLAGS) -o"$@" "$<"


s_function_opti_MPC8.o : $(START_DIR)/s_functions/fr3_no_hand_6dof/s_function_opti_MPC8.c
	$(CC) $(CFLAGS) -o"$@" "$<"


s_function_opti_robot_model_bus_fun.o : $(START_DIR)/main_franka/Controller/s_functions/s_function_opti_robot_model_bus_fun.c
	$(CC) $(CFLAGS) -o"$@" "$<"


shm_reader_sfun.o : $(START_DIR)/main_franka/Controller/s_functions/shm_reader_sfun.c
	$(CC) $(CFLAGS) -o"$@" "$<"


shm_writer_sfun.o : $(START_DIR)/main_franka/Controller/s_functions/shm_writer_sfun.c
	$(CC) $(CFLAGS) -o"$@" "$<"


sizes_codegen.o : $(START_DIR)/main_franka/franka_matlab_v0.3.1/franka_simulink_library/src/sizes_codegen.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


gripper_api.o : $(START_DIR)/main_franka/franka_matlab_v0.3.1/franka_simulink_library/src/gripper_api.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


robot_api.o : $(START_DIR)/main_franka/franka_matlab_v0.3.1/franka_simulink_library/src/robot_api.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


control_modes.o : $(START_DIR)/main_franka/franka_matlab_v0.3.1/franka_simulink_library/src/control_modes.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MPC8.o : $(START_DIR)/s_functions/fr3_no_hand_6dof/MPC8.c
	$(CC) $(CFLAGS) -o"$@" "$<"


robot_model_bus_fun.o : $(START_DIR)/main_franka/Controller/s_functions/robot_model_bus_fun.c
	$(CC) $(CFLAGS) -o"$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."


