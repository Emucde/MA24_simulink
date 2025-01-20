const http = require('http');
const WebSocket = require('ws');
const express = require('express');
const path = require('path');
const { startService, resetService, stopService, switchTrajectory, switch_control, switch_casadi_mpc, activate_control, get_controller_info, objectToString } = require('./ros_services');
const { searchTrajectoryNames } = require('./search_m_file');

// Express setup to serve HTML files
const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });
const clients = new Set();

var controller_names = null;
var controller_active_states = null;
var active_controller_idx = null;
var active_controller_name = null;
var enable_control = false;
var ros_running = false;
var traj_path = path.join(__dirname, '..', '..', '..', 'utils', 'matlab_init_general', 'param_traj_fr3_no_hand_6dof.m');

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
            })
            .catch((error) => {
                console.log("Error while getting loaded controllers. It seems that ROS2 crashed.");
                throw new Error('It seems that ROS2 crashed');
            });
    }
    catch (error) {
        console.log("Error while getting loaded controllers. ROS2 was not started. Retrying in 2 seconds.");
        setTimeout(init_ros, 2000);
    }
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

async function main() {
    try {
        init_ros(); // Initialize ROS2
    }
    catch (error) {
        console.log("Error while getting loaded controllers. It seems that ROS2 crashed.'");
    }

    wss.on('connection', (ws) => {
        clients.add(ws);

        console.log('WebSocket connection (client ' + (clients.size) + ') established\n');

        ws.on('close', () => {
            clients.delete(ws);
        });

        ws.on('message', async (message) => {
            // ws.send(JSON.stringify({ status: 'success', result: "test" }));
            const data = JSON.parse(message);
            let result;

            try {
                switch (data.command) {
                    case 'start':
                        var extra_status = '';
                        if(data.controller_name !== active_controller_name)
                        {
                            result = await switch_control(active_controller_name, data.controller_name);
                            result.name = 'ros_service';
                            if(result.ok)
                            {
                                result = await update(); // Update controller info
                                extra_status = 'Controller switched to ' + active_controller_name + ' and ';
                            }
                            else
                                break;
                        }
                        result = await startService(active_controller_name);
                        result.name = 'ros_service';
                        if(result.status != null && !result.status.includes('No active controller'))
                        {
                            result.status = extra_status + active_controller_name + ' started';
                        }
                        break;
                    case 'reset':
                        result = await resetService(active_controller_name);
                        if(result.status != null && !result.status.includes('No active controller'))
                        {
                            result.status = active_controller_name + ' reset';
                        }
                        result.name = 'ros_service';
                        break;
                    case 'stop':
                        result = await stopService(active_controller_name);
                        result.name = 'ros_service';
                        if(result.status != null && !result.status.includes('No active controller'))
                            result.status = active_controller_name + ' stopped';
                        break;
                    case 'trajectory_selection':
                        result = await switchTrajectory(active_controller_name, data.traj_select);
                        if(result.status != null && !result.status.includes('No active controller'))
                        {
                            result.status = 'Trajectory ' + data.traj_select + ' on ' + active_controller_name + ' selected';
                        }
                        result.name = 'ros_service';
                        break;
                    case 'switch_controller':
                        var old_controller_name = active_controller_name;
                        result = await switch_control(old_controller_name, data.controller_name);
                        if (result.ok) {
                            result = await update(); // Update controller info
                            result.status = 'Controller switched to ' + data.controller_name;
                        }
                        break;
                    case 'home':
                        var old_controller_name = active_controller_name;
                        result = await switch_control(old_controller_name, 'move_to_start_example_controller');
                        if (result.ok) {
                            result = await update(); // Update controller info
                            result.status = 'Homing in progress';
                            result.name = 'ros_service';
                            setTimeout(async function () {
                                result = await switch_control(active_controller_name, old_controller_name);
                                if (result.ok)
                                {
                                    result = await update(); // Update controller info
                                    ws.send(JSON.stringify({ status: 'success', result: { name: 'ros_service', status: 'Homing done' } }));
                                }
                            }, data.delay);
                        }
                        else
                            break;
                        if (result.status != null && result.status.includes('Already in')) {
                            result.status = result.status + '. Homing done';
                        }
                        result.name = 'ros_service';
                        break;
                    case 'update':
                        result = await update(); // Update controller info
                        result.name = 'ros_service';
                    case 'casadi_mpc_switch':
                        // switch internal mpc in casadi_controller
                        var mpc_type = data.mpc_type;
                        result = await switch_casadi_mpc(mpc_type);
                        if (result.ok) {
                            switch(mpc_type) {
                                case 0: result.status = 'Switched to Casadi MPC01'; break;
                                case 1: result.status = 'Switched to Casadi MPC6'; break;
                                case 2: result.status = 'Switched to Casadi MPC7'; break;
                                case 3: result.status = 'Switched to Casadi MPC8'; break;
                                case 4: result.status = 'Switched to Casadi MPC9'; break;
                                case 5: result.status = 'Switched to Casadi MPC10'; break;
                                case 6: result.status = 'Switched to Casadi MPC11'; break;
                                case 7: result.status = 'Switched to Casadi MPC12'; break;
                                case 8: result.status = 'Switched to Casadi MPC13'; break;
                                case 9: result.status = 'Switched to Casadi MPC14'; break;
                                default: result.status = 'Switched to Casadi MPC01'; break;
                            }
                        }
                        break;
                    case 'open_brakes':
                        broadcast('open_brakes');
                        result = { status: 'brakes_service: open_brakes', name: 'open_brakes' };
                        break;
                    case 'close_brakes':
                        broadcast('close_brakes');
                        result = { status: 'brakes_service: close_brakes', name: 'close_brakes' };
                        break;
                    default:
                        result = { status: 'error', error: 'Unknown command ' + data.command };
                        break;
                }

                if(result.status == null)
                    result.status = result;
                console.log(result.status);

                ws.send(JSON.stringify({ status: 'success', result: { name: result.name, status: result.status } }));
            } catch (error) {
                console.log(error);
                ws.send(JSON.stringify({ status: 'error', error: error.message }));
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
    });

    function broadcast(message) {
        clients.forEach(client => {
            if (client.readyState === WebSocket.OPEN) {
                client.send(message);
            }
        });
    }

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
        console.log('Server listening on http://localhost:8080');
    });
}

main().catch(error => {
    console.error("An error occurred in the main function:", error);
    process.exit(1);
});