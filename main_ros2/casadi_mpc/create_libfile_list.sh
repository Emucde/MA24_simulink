#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Set the output CMakeLists.txt file
CMAKE_FILE="$SCRIPT_DIR/../franka_ros2_ws/src/franka_example_controllers/LibraryFiles.cmake"

find_and_sort_files() {
    find "$1" -type f \( -name "*.so" \) -print0 | 
    xargs -0 stat --format='%Y %n' | 
    sort -rn | 
    cut -d' ' -f2- | 
    xargs realpath
}

# Find all .c and .cpp files
UTILS_LIBRARIES=$(find_and_sort_files "$SCRIPT_DIR/../../s_functions/fr3_no_hand_6dof/utils_libraries")
MPC_LIBRARIES=$(find_and_sort_files "$SCRIPT_DIR/../../s_functions/fr3_no_hand_6dof/mpc_libraries")

# Create a temporary file for the new CMakeLists.txt content
TMP_FILE=$(mktemp)

# Write the CMakeLists.txt content

echo "set(MPC_LIBRARIES" >> "$TMP_FILE"
# Add each source file to the SOURCES variable
for file in $MPC_LIBRARIES; do
    echo "    ${file#./}" >> "$TMP_FILE"
done
echo ")" >> "$TMP_FILE"

echo "set(UTILS_LIBRARIES" >> "$TMP_FILE"
# Add each source file to the SOURCES variable
for file in $UTILS_LIBRARIES; do
    echo "    ${file#./}" >> "$TMP_FILE"
done
echo ")" >> "$TMP_FILE"

# Replace the old CMakeLists.txt with the new one
mv "$TMP_FILE" "$CMAKE_FILE"

echo "SourceFiles.cmake updated with current library files."

