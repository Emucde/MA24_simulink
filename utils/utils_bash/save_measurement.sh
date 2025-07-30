#!/bin/bash
# This script copies configuration files to a specified directory.

# Help function
function display_help() {
    echo "Usage: $0 OUTPUT_DIR MEASUREMENT_NAME"
    echo
    echo "This script copies configuration files to a specified directory."
    echo
    echo "Arguments:"
    echo "  OUTPUT_DIR        The base directory where files will be copied."
    echo "  MEASUREMENT_NAME  The name of the measurement folder."
    echo
    echo "Options:"
    echo "  -h, --help        Display this help message."
    exit 0
}

# Check if help is requested
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    display_help
fi

# Validate input arguments
if [[ $# -ne 2 ]]; then
    echo "Error: Invalid number of arguments."
    display_help
fi

OUTPUT_DIR="$1"
MEASUREMENT_NAME="$2"

# Check if OUTPUT_DIR exists and is writable
if [[ ! -d "$OUTPUT_DIR" || ! -w "$OUTPUT_DIR" ]]; then
    echo "Error: OUTPUT_DIR does not exist or is not writable."
    exit 1
fi

TARGET_DIR="$OUTPUT_DIR/$MEASUREMENT_NAME"

# Overwrite confirmation if the directory already exists
if [[ -d "$TARGET_DIR" ]]; then
    read -p "Directory '$TARGET_DIR' already exists. Do you want to overwrite it? (y/n): " confirm
    if [[ "$confirm" != "y" ]]; then
        echo "Operation aborted."
        exit 0
    fi
fi

mkdir -p "$TARGET_DIR"
cp "$masterdir/config_settings/"* "$TARGET_DIR"
cp -R "$masterdir/main_ros2/nodejs_ros2_gui/public/"* "$TARGET_DIR"
cp "$masterdir/main_ros2/franka_ros2_ws/src/franka_bringup/config/controllers.yaml" "$TARGET_DIR"

echo "Files successfully copied to '$TARGET_DIR'."
