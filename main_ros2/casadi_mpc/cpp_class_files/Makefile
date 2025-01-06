# Default build type
BUILD_TYPE ?= debug

# Set paths
MASTERDIR = $(masterdir)
COLCON_INCLUDE = $(_colcon_cd_root)/include
COLCON_LIB = $(_colcon_cd_root)/lib
EIGEN_PATH = $(eigen_path)
CASADI_PATH = $(casadi_path)
MPC_INCLUDE = $(MASTERDIR)/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles
UTILS_INCLUDE = $(MASTERDIR)/main_ros2/casadi_mpc/cpp_class_files/include/
MPC_LIBS = $(MASTERDIR)/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles

# Define directories
RELEASE_DIR = bin/release
DEBUG_DIR = bin/debug
OUT_DIR = $(if $(filter release,$(BUILD_TYPE)),$(RELEASE_DIR),$(DEBUG_DIR))

# Set compiler and flags
CXX = /usr/bin/g++-11 $(shell pkg-config --cflags pinocchio)
CXXFLAGS = -fdiagnostics-color=always -Wall -Wextra -std=c++11
LDFLAGS = -Wl,-rpath,$(MPC_LIBS) -L$(CASADI_PATH) -L$(MPC_LIBS)
LIBS = -lMPC01_param -lMPC6_param -lMPC7_param -lMPC8 -lMPC9_param -lMPC10_param -lMPC11_param -lMPC12_param -lMPC13_param -lMPC14_param -lMPC01 -lMPC6 -lMPC7 -lMPC8 -lMPC9 -lMPC10 -lMPC11 -lMPC12 -lMPC13 -lMPC14 -lpinocchio_utils -lCasadiMPC -lcasadi $(shell pkg-config --libs pinocchio)

# Include directories
INCLUDES = -I$(COLCON_INCLUDE) -I$(EIGEN_PATH) -I$(CASADI_PATH) -I$(MPC_INCLUDE) -I$(UTILS_INCLUDE)

# Optimization and debug flags
ifeq ($(BUILD_TYPE), release)
	OPT_FLAGS = -O3
	DEBUG_FLAGS =
else
	OPT_FLAGS = -O0
	DEBUG_FLAGS = -g
endif

CXXFLAGS += $(OPT_FLAGS) $(DEBUG_FLAGS) $(INCLUDES)

# Source files
SRC_CPP = $(wildcard $(MASTERDIR)/main_ros2/casadi_mpc/cpp_class_files/*.cpp)
SRC_C = $(wildcard $(MASTERDIR)/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC*_param.c)

# Automatically convert source files to object files
OBJ_CPP = $(SRC_CPP:.cpp=.o)
OBJ_C = $(SRC_C:.c=.o) # wird nicht mehr verwendet da ich libs habe

# Output binary
TARGET = $(OUT_DIR)/main

# Default target
.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJ_CPP)
	@echo "Building in $(BUILD_TYPE) mode..."
	@mkdir -p $(OUT_DIR)  # Ensure output directory exists
	@echo "$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)"
	@if /usr/bin/time -f "\nBuild Statistics:\n\
	User time: %U seconds\n\
	System time: %S seconds\n\
	Elapsed time: %e seconds\n\
	CPU usage: %P\n\
	Max memory usage: %M KB" \
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS); then \
		echo "\033[32mBuild completed successfully!\033[0m"; \
	else \
		echo "\033[31mBuild failed.\033[0m"; \
		exit 1; \
	fi

# Rules for building object files
$(OUT_DIR)/%.o: $(MASTERDIR)/main_ros2/casadi_mpc/cpp_class_files/%.cpp
	@mkdir -p $(OUT_DIR)  # Ensure output directory exists
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -c $< -o $@

# $(OUT_DIR)/%.o: $(MASTERDIR)/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/%.c
# 	@mkdir -p $(OUT_DIR)  # Ensure output directory exists
# 	$(CXX) $(CXXFLAGS) $(LDFLAGS) -c $< -o $@

# Clean up build artifacts
clean:
	@echo "Cleaning up..."
	@rm -rf $(RELEASE_DIR) $(DEBUG_DIR)
