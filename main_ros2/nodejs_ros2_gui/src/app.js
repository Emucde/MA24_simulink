/*
This program creates a WebSocket server that communicates with a ROS 2 system.
It allows clients to connect and send commands to control various robotic controllers,
switch trajectories, and manage services. The server also handles homing operations
and provides real-time updates on the status of the controllers.
- It includes error handling for ROS 2 connection issues and can restart the ROS 2 node
if necessary.
- It uses the 'ws' library for WebSocket communication, 'express' for serving static files
and HTML pages, and 'child_process' to execute shell commands for managing ROS 2 processes
and services.
- It also includes a function to search for trajectory names in a specified MATLAB file.
- The server listens on port 8080 for WebSocket connections and serves a static HTML page
at the root URL.
- It also listens on port 8081 for data logger WebSocket connections.
The server handles various commands such as starting, stopping, resetting controllers,
switching trajectories, and performing homing operations.
- It broadcasts messages to all connected clients and handles incoming messages
from clients to perform the requested actions.
- It also includes a function to kill processes matching a specific pattern.
- It uses a JSON configuration file to manage available Casadi MPCs and workspace controllers.
- It handles unhandled promise rejections and signals for graceful shutdown.
*/

// Import necessary modules
const http = require('http');
const WebSocket = require('ws');
const express = require('express');
const path = require('path');
const { exec } = require('child_process');
const kill_ros = 'kill $(pgrep -f "launch.py")';
const { checkROSConnection, restartNode, startService, resetService, stopService, switchTrajectory, switch_control, switch_casadi_mpc, switch_workspace_controller, deactivate_control, activate_control, get_controller_info, objectToString, set_enable_single_joint_homing, set_trajectory_selection_homing } = require('./ros_services');
const { searchTrajectoryNames } = require('./search_m_file');

// Express setup to serve HTML files
const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });
const wss_data_logger = new WebSocket.Server({ port: 8081 });
const clients = new Set();
const clients_data_logger = new Set();

// Global variables
var controller_names = null;
var controller_active_states = null;
var active_controller_idx = null;
var active_controller_name = null;
var enable_control = false;
var ros_running = false;
var init_ros_running = false;
var traj_path = path.join(__dirname, '..', '..', '..', 'utils', 'matlab_init_general', 'param_traj_fr3_no_hand_6dof.m');
var general_config_path = path.join(__dirname, '..', '..', '..', 'config_settings', 'general_settings.json');
var general_config = require(general_config_path);
var available_casadi_mpcs = general_config['available_casadi_mpc'];
var available_workspace_controller = general_config['available_classic_controller'];

app.use(express.static(path.join(__dirname, '..', 'public')));

app.get('/', function (req, res) {
    res.sendFile(__dirname + '/index.html');
});

/**
 * Function to kill processes matching a specific pattern.
 * @param {string} pattern - The pattern to search for (e.g., "launch.py").
 * @param {function} callback - Optional callback to handle the result.
 */
function kill_process(pattern, callback) {
    const command = `pgrep -f "${pattern}" | xargs -r kill`;

    exec(command, (error, stdout, stderr) => {
        if (error) {
            console.error(`Error executing command: ${error.message}`);
            if (callback) callback(error, null);
            return;
        }
        if (stderr) {
            console.error(`Error output: ${stderr}`);
            if (callback) callback(new Error(stderr), null);
            return;
        }
        console.log(`Command output: ${stdout}`);
        if (callback) callback(null, stdout);
    });
}

/** * Function to restart the ROS 2 node.
 * @returns {Promise} - A promise that resolves when the node is restarted.
 */
async function init_ros() {
    try {
        const result = await get_controller_info();
        controller_names = result.controller_names;
        controller_active_states = result.controller_active_states;
        active_controller_idx = result.active_controller_idx;
        console.log("Loaded controllers: ", controller_names);

        if (active_controller_idx === -1) {
            console.log("No active controller found. Setting 'move_to_start_example_controller' as active.");
            const result_activate = await activate_control('move_to_start_example_controller');
            await check(result_activate, 'activate_control', {});
        }

        active_controller_name = controller_names[active_controller_idx];
        enable_control = ['mpc_crocoddyl_controller', 'mpc_casadi_controller'].includes(active_controller_name);
        console.log("Currently active controllers: ", active_controller_name);
        ros_running = true;
        init_ros_running = false;
        broadcast(JSON.stringify({ status: 'success', result: { name: 'ros2_alive', status: 'ROS2 connected!' } }));
    } catch (error) {
        console.log("Error while getting loaded controllers. ROS2 was not started. Retrying in 5 seconds.");
        setTimeout(init_ros, 5000);
    }
}

// Check ROS connection every 5 seconds
async function check_ros_connection() {
    setInterval(async function () {
        var is_alive = await checkROSConnection();
        if(!is_alive && !init_ros_running)
        {
            broadcast(JSON.stringify({ status: 'error', error: 'ROS2 not connected!' }));
            init_ros();
            init_ros_running = true;
        }
    }, 5000);
}

// Get controller info and update global variables
async function update() {
    var result = await get_controller_info()
    if(result.ok)
    {
        ({ controller_names, controller_active_states, active_controller_idx, active_controller_name } = result);
        result.status = 'Controller info updated, active controller: ' + controller_names[active_controller_idx];
    }
    else
    {
        result.status = 'Error while updating controller info';
    }
    return result;
}

/*

* Function to check the result of a command.
* @param {Object} result - The result object from the command.
* @param {string} checked_command - The command that was checked.
* @param {Object} data - Additional data related to the command.
* @returns {Object} - An object containing the status and name of the command.
*/
async function check(result, checked_command, data)
{
    var status = '[' + data.command + ']: ';
    var name;
    if(result.ok == null)
        result.ok = true;

    if(result.status == null)
        status = '';
    else
        status = result.status;

    switch(checked_command)
    {
        case 'switch_control':
            if (result.ok) {
                result = await update(); // Update controller info
                status += ' Controller switched to ' + active_controller_name + ' ';
                console.log('Controller switched to ' + active_controller_name);
            }
            else
                status += 'Error while switching controller';
            name = 'ros_service';
            break;
        case 'disable_control':
            if (result.ok) {
                status += ' All Controller disabled';
                console.log('All Controller disabled');
            }
            else
                status += 'Error while switching controller';
            name = 'ros_service';
            break;
        case 'switch_casadi_mpc':
            if (result.ok) {
                status += ' Switched to Casadi ' + available_casadi_mpcs[data.mpc_type];
            }
            else
                status += ' Error while switching Casadi MPC';
            name = 'ros_service';
            break;
        case 'switch_workspace_controller':
            if (result.ok) {
                status += ' Switched to ' + available_workspace_controller[data.workspace_controller_type] + ' Workspace Controller';
            }
            else
                status += 'Error while switching Casadi MPC';
            name = 'ros_service';
            break;
        case 'switch_trajectory':
            if(result.ok )
            {
                if(result.status != null && !result.status.includes('No active controller'))
                {
                    status += ' Trajectory ' + data.traj_num + ' on ' + active_controller_name + ' selected ';
                }
            }
            else
                status += 'Error while switching trajectory: ' + result.status;
            name = 'ros_service';
            break;
        case 'start_service':
            if(result.ok)
            {
                if(result.status != null && !result.status.includes('No active controller'))
                {
                    status += active_controller_name + ' started ';
                }
            }
            else
                status += ' Error while starting controller: ' + result.status;
            name = 'ros_service';
            break;
        case 'reset_service':
            if(result.ok)
            {
                if(result.status != null && !result.status.includes('No active controller'))
                {
                    status += active_controller_name + ' reset';
                }
            }
            else
                status += ' Error while resetting controller: ' + result.status;
            name = 'ros_service';
            break;
        case 'stop_service':
            if(result.ok )
            {
                if(result.status != null && !result.status.includes('No active controller'))
                    result.status = active_controller_name + ' stopped';
            }
            else
                status += ' Error while stopping controller: ' + result.status;
            name = 'ros_service';
            break;
        case 'update':
            name = 'ros_service';
            break;
        default:
            name = result.name;
            break;
    }
    console.log(status);
    return { ok: result.ok, status: status, name: name };
}

// broadcasting data
function broadcast(message) {
    clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(message);
        }
    });
}

// broadcasting data logger
function broadcast_data_logger(message) {
    clients_data_logger.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(message);
        }
    });
}

/** * Function to perform homing operation.
 * @param {WebSocket} ws - The WebSocket connection to send messages to the client.
 * @param {Object} data - The data object containing command and trajectory number.
 * @returns {Object} - An object containing the status of the homing operation.
 */
async function performHoming(ws, data) {
    let log_check;

    // Set single joint homing
    if (data.command === 'home') {
        result = await set_enable_single_joint_homing('move_to_start_example_controller', false);
    } else {
        result = await set_enable_single_joint_homing('move_to_start_example_controller', true);
    }
    log_check = await check(result, 'ros_service', data);

    // Set trajectory_selection
    result = await set_trajectory_selection_homing('move_to_start_example_controller', data.traj_num);
    log_check = await check(result, 'ros_service', data);

    // Switch controller
    var old_controller_name = active_controller_name;
    result = await switch_control(old_controller_name, 'move_to_start_example_controller');
    log_check = await check(result, 'switch_control', data);

    if (log_check.ok) {
        result = await update(); // Update controller info
        log_check = await check(result, 'update', data);

        if (log_check.ok) {
            log_check.status += '. Homing in progress';

            // Set timeout for homing process
            setTimeout(async function () {
                result = await switch_control(active_controller_name, old_controller_name);
                log_check = await check(result, 'switch_control', data);
                log_check.status += '. Homing done';

                if (result.ok) {
                    result = await update(); // Update controller info
                    log_check = await check(result, 'update', data);

                    if (log_check.ok) {
                        ws.send(JSON.stringify({ status: 'success', result: { name: 'ros_service', status: 'Homing done' } }));
                    } else {
                        ws.send(JSON.stringify({ status: 'error', error: { name: 'ros_service', status: log_check.status } }));
                    }
                }
            }, data.delay);
        }
    }

    return log_check;
}

/**
 * Main function to initialize the WebSocket server and handle connections.
 * It sets up the WebSocket server, handles incoming messages, and manages client connections.
 * It also listens for connections on the data logger WebSocket server.
 * The function handles various commands from clients, such as starting, stopping, and resetting controllers,
 * switching trajectories, and performing homing operations.
 * It broadcasts messages to all connected clients and handles errors gracefully.
 * It also checks the ROS connection status and restarts the ROS node if necessary.
 * The function listens for unhandled promise rejections and signals for graceful shutdown.
 */
async function main() {
    wss_data_logger.on('connection', (ws) => {
        clients_data_logger.add(ws);
        console.log('WebSocket connection (data logger ' + (clients_data_logger.size) + ') established\n');
        ws.on('close', () => {
            clients_data_logger.delete(ws);
        });

        ws.on('message', async (message) => {
            broadcast_data_logger(message);
        });
    });


    wss.on('listening', (ws) => {
        check_ros_connection();
    });

    wss.on('connection', (ws) => {
        clients.add(ws);

        console.log('WebSocket connection (client ' + (clients.size) + ') established\n');

        ws.on('close', () => {
            clients.delete(ws);
        });

        ws.on('message', async (message) => {
            const data = JSON.parse(message);
            var log_check = { ok: false, status: '' };
        
            try
            {
                // no ros services:
                switch (data.command)
                {
                    case 'open_brakes':
                        broadcast('open_brakes');
                        log_check = { ok: true, status: 'brakes_service: open_brakes', name: 'open_brakes' };
                        break;
                    case 'close_brakes':
                        broadcast('close_brakes');
                        log_check = { ok: true, status: 'brakes_service: close_brakes', name: 'close_brakes' };
                        break;
                }

                let result = await update(); // Update controller info
                log_check = await check(result, 'update', data)
                if(log_check.ok)
                {
                    switch (data.command)
                    {
                        case 'start':
                            // Switch Controller
                            var old_controller_name = active_controller_name;
                            result = await deactivate_control(old_controller_name);

                            // result = await switch_control(old_controller_name, data.controller_name);
                            log_check = await check(result, 'disable_control', data)
                            if (!log_check.ok)
                                break;

                            result = await activate_control(data.controller_name);
                            log_check = await check(result, 'switch_control', data)
                            if (!log_check.ok)
                                break;

                            if(active_controller_name === 'mpc_casadi_controller')
                            {
                                result = await switch_casadi_mpc(data.mpc_type);
                                log_check = await check(result, 'switch_casadi_mpc', data)
                                if (!log_check.ok)
                                    break;
                            }

                            if(active_controller_name === 'conventional_workspace_controller')
                            {
                                result = await switch_workspace_controller(data.workspace_controller_type);
                                log_check = await check(result, 'switch_workspace_controller', data)
                                if (!log_check.ok)
                                    break;
                            }

                            if(active_controller_name !== 'gravity_compensation_example_controller' && active_controller_name !== 'move_to_start_example_controller')
                            {
                                // Switch Trajectory
                                result = await switchTrajectory(active_controller_name, data.traj_num);
                                log_check = await check(result, 'switch_trajectory', data)
                                if (!log_check.ok)
                                    break;

                                // start controller
                                result = await startService(active_controller_name);
                                log_check = await check(result, 'start_service', data)
                            }
                            break;
                        case 'reset':
                            // Switch Controller
                            var old_controller_name = active_controller_name;
                            result = await switch_control(old_controller_name, data.controller_name);
                            log_check = await check(result, 'switch_control', data)
                            if (!log_check.ok)
                                break;
                            result = await resetService(active_controller_name);
                            log_check = await check(result, 'reset_service', data)
                            break;
                        case 'stop':
                            // Switch Controller
                            var old_controller_name = active_controller_name;
                            result = await switch_control(old_controller_name, data.controller_name);
                            log_check = await check(result, 'switch_control', data)
                            if (!log_check.ok)
                                break;
                            result = await stopService(active_controller_name);
                            log_check = await check(result, 'stop_service', data)
                            break;
                        case 'home_q_fixed':
                        case 'home':
                            performHoming(ws, data);
                            break;
                        default:
                            break;
                    }
                }               

                if(log_check.ok)
                    ws.send(JSON.stringify({ status: 'success', result: { name: log_check.name, status: log_check.status } }));
                else
                    ws.send(JSON.stringify({ status: 'error', error: { name: log_check.name, status: log_check.status } }));
            } catch (error)
            {
                console.error('An error occurred:', error.message);
                additional_string = "";
                if(error.message.includes('Received undefined'))
                    {
                        new Promise((resolve, reject) => {
                            kill_process('launch.py', (error, result) => {
                                if (error) {
                                    reject(error);
                                } else {
                                    resolve(result);
                                }
                            });
                        })
                        .then(() => {
                            console.log('Successfully killed processes');
                            restartNode()
                            .then(() => {
                            console.log('Node restarted successfully');
                            // Perform any necessary actions after restart
                            })
                            .catch(error => {
                            console.error('Failed to restart node:', error);
                            // Handle restart failure
                            });
                            additional_string=": ROS2 service node restarted";
                        })
                        .catch((error) => {
                            console.error('Failed to kill processes:', error.message);
                        });
                    }
                ws.send(JSON.stringify({ status: 'error', error: error.message + additional_string }));
            }
        });

        // Search for trajectory names
        searchTrajectoryNames(traj_path)
            .then(trajectoryNames => {
                console.log('Found Trajectory Names:', trajectoryNames);
                ws.send(JSON.stringify({ status: 'success', result: { name: 'traj_names', trajectories: trajectoryNames } }));
            })
            .catch(err => {
                console.error(err);
            });

        // Send available Casadi MPCs
        ws.send(JSON.stringify({ status: 'success', result: { name: 'casadi_mpcs', mpcs: available_casadi_mpcs } }));
    });

    const terminator = (signal) => {
        console.log(`Received ${signal} - terminating app`);
        process.exit(1);
    };

    ['SIGHUP', 'SIGINT', 'SIGQUIT', 'SIGILL', 'SIGTRAP', 'SIGABRT',
        'SIGBUS', 'SIGFPE', 'SIGUSR1', 'SIGSEGV', 'SIGUSR2', 'SIGTERM'
    ].forEach((signal) => {
        process.on(signal, () => terminator(signal));
    });

    server.listen(8080, () => {
        console.log('Server listening on http://localhost:8080 for ros commands');
    });
}

main().catch(error => {
    console.error("An error occurred in the main function:", error);
    process.exit(1);
});

// Handle unhandled promise rejections
// This will catch any unhandled promise rejections in the application
// and log the error, then attempt to restart the ROS node if necessary.
process.on('unhandledRejection', (reason, promise) => {
    console.error('Unhandled Rejection at:', promise, 'reason:', reason);

    if(reason.message.includes('Received undefined'))
    {
        new Promise((resolve, reject) => {
            kill_process('launch.py', (error, result) => {
                if (error) {
                    reject(error);
                } else {
                    resolve(result);
                }
            });
        })
        .then(() => {
            console.log('Successfully killed processes');
            restartNode()
            .then(() => {
            console.log('Node restarted successfully');
            // Perform any necessary actions after restart
            })
            .catch(error => {
            console.error('Failed to restart node:', error);
            // Handle restart failure
            });
            additional_string=": ROS2 service node restarted";
        })
        .catch((error) => {
            console.error('Failed to kill processes:', error.message);
        });
    }
    // Optional: Exit with failure code
    // process.exit(1);
});

// Handle graceful shutdown on SIGTERM
// This will allow the application to perform cleanup operations before exiting.
process.on('SIGTERM', () => {
    console.log('Received SIGTERM. Shutting down gracefully.');
    // Perform cleanup operations here
    process.exit(0);
    });