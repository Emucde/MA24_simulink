#!/bin/bash

# Default path to MuJoCo simulate binary
DEFAULT_MUJOCO_PATH="/media/daten/Anwendungen/mujoco-2.3.7/bin/simulate"

# Calculate the absolute directory of this script
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Use the absolute directory to define XML directories
FR3_XML_DIR="${SCRIPT_DIR}/fr3_mujoco"
UR5E_XML_DIR="${SCRIPT_DIR}/ur5e_mujoco"

# Help message
function show_help() {
  echo "Usage: $0 [-p MUJOCO_PATH] <base_model> <configuration>"
  echo
  echo "Arguments:"
  echo "  -p MUJOCO_PATH     Optional: Path to MuJoCo simulate binary (default: $DEFAULT_MUJOCO_PATH)"
  echo "  <base_model>       Base model name (e.g., 'fr3' or 'ur5e')"
  echo "  <configuration>    Configuration of the model:"
  echo "                     - For 'fr3': '7dof', '6dof', '6dof_no_hand'"
  echo "                     - For 'ur5e': '6dof'"
  echo
  echo "Examples:"
  echo "  $0 fr3 7dof"
  echo "  $0 ur5e 6dof"
  echo "  $0 -p /custom/path/to/simulate fr3 6dof"
}

# Validate inputs
function validate_inputs() {
  local base_model="$1"
  local configuration="$2"

  # Define valid configurations for each model
  local valid_fr3_configurations=("7dof" "6dof" "6dof_no_hand")
  local valid_ur5e_configurations=("6dof")

  # Check if base model is valid
  if [[ "$base_model" != "fr3" && "$base_model" != "ur5e" ]]; then
    echo "Error: Invalid base model '${base_model}'. Supported models are: 'fr3', 'ur5e'."
    show_help
    exit 1
  fi

  # Check if configuration is valid for the specified base model
  if [[ "$base_model" == "fr3" && ! " ${valid_fr3_configurations[@]} " =~ " ${configuration} " ]]; then
    echo "Error: Invalid configuration '${configuration}' for base model 'fr3'."
    echo "Valid configurations for 'fr3' are: ${valid_fr3_configurations[*]}."
    show_help
    exit 1
  elif [[ "$base_model" == "ur5e" && ! " ${valid_ur5e_configurations[@]} " =~ " ${configuration} " ]]; then
    echo "Error: Invalid configuration '${configuration}' for base model 'ur5e'."
    echo "Valid configurations for 'ur5e' are: ${valid_ur5e_configurations[*]}."
    show_help
    exit 1
  fi

  # Determine XML directory and file based on base model and configuration
  if [[ "$base_model" == "fr3" ]]; then
    XML_FILE="${FR3_XML_DIR}/${base_model}_${configuration}.xml"
  elif [[ "$base_model" == "ur5e" ]]; then
    XML_FILE="${UR5E_XML_DIR}/${base_model}_${configuration}.xml"
  fi

  # Check if the XML file exists in the specified directory
  if [[ ! -f "$XML_FILE" ]]; then
    echo "Error: XML file '${XML_FILE}' not found."
    exit 1
  fi
}

# Main logic
function main() {
  # Default values
  local mujoco_path="$DEFAULT_MUJOCO_PATH"

  # Parse options using getopts
  while getopts ":p:h" opt; do
    case $opt in
      p)
        mujoco_path="$OPTARG"
        ;;
      h)
        show_help
        exit 0
        ;;
      \?)
        echo "Error: Invalid option -$OPTARG"
        show_help
        exit 1
        ;;
      :)
        echo "Error: Option -$OPTARG requires an argument."
        show_help
        exit 1
        ;;
    esac
  done

  # Shift positional arguments after options are processed
  shift $((OPTIND -1))

  # Validate arguments count after options are parsed
  if [[ $# -ne 2 ]]; then
    echo "Error: Invalid number of arguments."
    show_help
    exit 1
  fi

  # Parse positional arguments (base_model and configuration)
  local base_model="$1"
  local configuration="$2"

  # Validate inputs and file existence in respective directories (fr3_mujoco or ur5e_mujoco)
  validate_inputs "$base_model" "$configuration"

  # Run simulation with the determined XML file path and MuJoCo binary path
  echo "Running simulation with ${XML_FILE} using MuJoCo binary at ${mujoco_path}..."
  
  "${mujoco_path}" "${XML_FILE}"
}

# Entry point of the script
main "$@"

