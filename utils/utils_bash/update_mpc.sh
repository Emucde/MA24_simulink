#!/bin/bash

# Send the MATLAB command to commands.txt
echo "mpc_casadi_main;" > commands.txt

# Wait for the file to be deleted
while [ -f commands.txt ]; do
    sleep 1
done

# Print completion message
echo "fin"