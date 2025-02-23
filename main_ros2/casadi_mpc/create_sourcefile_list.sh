#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Set the output CMakeLists.txt file
CMAKE_FILE="$SCRIPT_DIR/cpp_class_files/SourceFiles.cmake"

find_and_sort_files() {
    find "$1" -type f \( -name "*.c" -o -name "*.cpp" \) -print0 | 
    xargs -0 du -b | 
    sort -rn | 
    cut -f2- | 
    xargs realpath
}

# Find all .c and .cpp files
UTILS_SOURCES=$(find_and_sort_files "$SCRIPT_DIR/cpp_class_files/src")
MPC_SOURCES=$(find_and_sort_files "$SCRIPT_DIR/../../s_functions/fr3_no_hand_6dof/mpc_c_sourcefiles")

# Create a temporary file for the new CMakeLists.txt content
TMP_FILE=$(mktemp)

# Write the CMakeLists.txt content

echo "set(UTILS_SOURCES" >> "$TMP_FILE"
# Add each source file to the SOURCES variable
for file in $UTILS_SOURCES; do
    echo "    ${file#./}" >> "$TMP_FILE"
done
echo ")" >> "$TMP_FILE"

echo "set(MPC_SOURCES" >> "$TMP_FILE"
for file in $MPC_SOURCES; do
    echo "    ${file#./}" >> "$TMP_FILE"
done
echo ")" >> "$TMP_FILE"

# Replace the old CMakeLists.txt with the new one
mv "$TMP_FILE" "$CMAKE_FILE"

echo "SourceFiles.cmake updated with current source files."

