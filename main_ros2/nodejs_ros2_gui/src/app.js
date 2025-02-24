const http = require('http');
const WebSocket = require('ws');
const express = require('express');
const path = require('path');
const { checkROSConnection, restartNode, startService, resetService, stopService, switchTrajectory, switch_control, switch_casadi_mpc, switch_workspace_controller, activate_control, get_controller_info, objectToString } = require('./ros_services');
const { searchTrajectoryNames } = require('./search_m_file');

// Express setup to serve HTML files
const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });
const wss_data_logger = new WebSocket.Server({ port: 8081 });
const clients = new Set();
const clients_data_logger = new Set();

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

async function init_ros() {
    try {
        await get_controller_info()
            .then((result) => {
                controller_names = result.controller_names;
                controller_active_states = result.controller_active_states;
                active_controller_idx = result.active_controller_idx
                console.log("Loaded controllers: ", controller_names);
                if (active_controller_idx === -1) {
                    console.log("No active controller found. Setting 'move_to_start_example_controller' as active.");
                    result_activate = activate_control('move_to_start_example_controller');
                    check_result_activate(result_activate)
                }
                active_controller_name = controller_names[active_controller_idx];
                enable_control = (active_controller_name === 'mpc_pinocchio_controller' || active_controller_name === 'mpc_casadi_controller');
                console.log("Currently active controllers: ", active_controller_name);
                ros_running = true;
                init_ros_running = false;
                broadcast(JSON.stringify({ status: 'success', result: { name: 'ros2_alive', status: 'ROS2 connected!' } }));
            })
            .catch((error) => {
                console.log("Error while getting loaded controllers. It seems that ROS2 crashed.");
                throw new Error('It seems that ROS2 crashed');
            });
    }
    catch (error) {
        console.log("Error while getting loaded controllers. ROS2 was not started. Retrying in 5 seconds.");
        setTimeout(init_ros, 5000);
    }
}

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

function broadcast_data_logger(message) {
    clients_data_logger.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(message);
        }
    });
}

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
                            result = await switch_control(old_controller_name, data.controller_name);
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

                            // Switch Trajectory
                            result = await switchTrajectory(active_controller_name, data.traj_num);
                            log_check = await check(result, 'switch_trajectory', data)
                            if (!log_check.ok)
                                break;

                            // start controller
                            result = await startService(active_controller_name);
                            log_check = await check(result, 'start_service', data)
                            break;
                        case 'reset':
                            result = await resetService(active_controller_name);
                            log_check = await check(result, 'reset_service', data)
                            break;
                        case 'stop':
                            result = await stopService(active_controller_name);
                            log_check = await check(result, 'stop_service', data)
                            break;
                        case 'home':
                            var old_controller_name = active_controller_name;
                            result = await switch_control(old_controller_name, 'move_to_start_example_controller');
                            log_check = await check(result, 'switch_control', data)
                            if (log_check.ok) {
                                result = await update(); // Update controller info
                                log_check = await check(result, 'update', data)
                                if (log_check.ok) {
                                    log_check.status += '. Homing in progress';
                                    setTimeout(async function () {
                                        result = await switch_control(active_controller_name, old_controller_name);
                                        log_check = await check(result, 'switch_control', data)
                                        log_check.status += '. Homing done';
                                        if (result.ok)
                                        {
                                            result = await update(); // Update controller info
                                            log_check = await check(result, 'update', data)
                                            if (log_check.ok)
                                                ws.send(JSON.stringify({ status: 'success', result: { name: 'ros_service', status: 'Homing done' } }));
                                            else
                                                ws.send(JSON.stringify({ status: 'error', error: { name: 'ros_service', status: log_check.status } }));
                                        }
                                    }, data.delay);
                                }
                            }
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