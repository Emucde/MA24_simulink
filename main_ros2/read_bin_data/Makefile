# Default build type
BUILD_TYPE ?= debug

# Set paths
MASTERDIR = $(masterdir)
COLCON_INCLUDE = $(colcon_cd_root)/include
EIGEN_PATH = $(eigen_path)
CASADI_PATH = $(casadi_path)

# Define directories
RELEASE_DIR = bin/release
DEBUG_DIR = bin/debug
OUT_DIR = $(if $(filter release,$(BUILD_TYPE)),$(RELEASE_DIR),$(DEBUG_DIR))

# Set compiler and flags
CXX = /usr/bin/g++-11 $(shell pkg-config --cflags pinocchio)
CXXFLAGS = -fdiagnostics-color=always
LDFLAGS = -Wl,-rpath=$(MASTERDIR)/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles -L $(MASTERDIR)/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles -L $(CASADI_PATH)
LIBS = -lMPC01 -lMPC6 -lMPC7 -lMPC8 -lMPC9 -lMPC10 -lMPC11 -lMPC12 -lMPC13 -lMPC14 -lcasadi $(shell pkg-config --libs pinocchio)

# Include directories
INCLUDES = -I$(COLCON_INCLUDE) -I$(EIGEN_PATH) -I$(CASADI_PATH) -I$(MASTERDIR)/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles

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
SRC_CPP = $(wildcard $(MASTERDIR)/main_ros2/read_bin_data/*.cpp)
SRC_C = $(wildcard $(MASTERDIR)/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/MPC*_param.c)

# Automatically convert source files to object files
OBJ_CPP = $(SRC_CPP:.cpp=.o)
OBJ_C = $(SRC_C:.c=.o)

# Output binary
TARGET = $(OUT_DIR)/main

# Default target
.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJ_CPP) $(OBJ_C)
	@echo "Building in $(BUILD_TYPE) mode..."
	@mkdir -p $(OUT_DIR)  # Ensure output directory exists
	@if /usr/bin/time -f "\nBuild Statistics:\n\
	User time: %U seconds\n\
	System time: %S seconds\n\
	Elapsed time: %e seconds\n\
	CPU usage: %P\n\
	Max memory usage: %M KB" \
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS); then \
		echo "\033[32mBuild completed successfully!\033[0m"; \
	else \
		echo "\033[31mBuild failed.\033[0m"; \
		exit 1; \
	fi

# Rules for building object files
$(OUT_DIR)/%.o: $(MASTERDIR)/main_ros2/read_bin_data/%.cpp
	@mkdir -p $(OUT_DIR)  # Ensure output directory exists
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -c $< -o $@

$(OUT_DIR)/%.o: $(MASTERDIR)/s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles/%.c
	@mkdir -p $(OUT_DIR)  # Ensure output directory exists
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -c $< -o $@

# Clean up build artifacts
clean:
	@echo "Cleaning up..."
	@rm -rf $(RELEASE_DIR) $(DEBUG_DIR)
