const http = require('http');
const WebSocket = require('ws');
const express = require('express');
const path = require('path');
const { startService, resetService, stopService, switchTrajectory, switch_control, activate_control, get_controller_info, objectToString } = require('./ros_services');
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

                console.log("Loaded controllers: ", controller_names);
                active_controller_idx = controller_active_states.findIndex(state => state === 'active');
                if (active_controller_idx === -1) {
                    console.log("No active controller found. Setting 'move_to_start_example_controller' as active.");
                    activate_control('move_to_start_example_controller');
                    init_ros();
                    return;
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
            var status = 'Service request timed out', name = 'ros_service';
            let result;

            if(data.controller_name != undefined)
            {
                if (data.controller_name !== active_controller_name) {
                    result = await switch_control(active_controller_name, data.controller_name);
                    if (result.ok) {
                        active_controller_name = data.controller_name;
                        status = 'Controller switched to ' + data.controller_name;
                    }
                    else {
                        init_ros();
                        status = objectToString(result);
                    }
                }
                enable_control = (active_controller_name === 'mpc_pinocchio_controller' || active_controller_name === 'mpc_casadi_controller');
            }
            try {
                if (enable_control) {
                    switch (data.command) {
                        case 'start':
                            result = await startService(active_controller_name);
                            status = result.status;
                            break;
                        case 'reset':
                            result = await resetService(active_controller_name);
                            status = result.status;
                            break;
                        case 'stop':
                            result = await stopService(active_controller_name);
                            status = result.status;
                            break;
                        case 'trajectory_selection':
                            result = await switchTrajectory(active_controller_name, data.traj_select);
                            status = result.status;
                            break;
                    }
                }
                switch (data.command) {
                    case 'home':
                        var old_controller_name = active_controller_name;
                        active_controller_name = 'move_to_start_example_controller';
                        result = await switch_control(old_controller_name, active_controller_name);

                        var status = 'Homing in progress';
                        if (result.ok) {
                            setTimeout(async function () {
                                try {
                                    result = await switch_to_active_control(active_controller_name, old_controller_name);
                                    active_controller_name = old_controller_name;
                                    result.status = 'Homing done';
                                } catch (error) {
                                    console.log("Error while homeing. It seems that ROS2 crashed.'");
                                    result = { status: 'Error while homeing. Service timed out. It seems that ROS2 crashed.' };
                                    status = result.status;
                                }
                                ws.send(JSON.stringify({ status: 'success', result: { name: 'ros_service', status: result.status } }));
                            }, data.delay);
                        }
                        else if (result.status.includes('Already in')) {
                            status = result.status + '. Homing done';
                        }
                        else {
                            if (result.error instanceof Error) {
                                console.error(result.error);
                            } else {
                                console.log(result);
                            }
                        }
                        break;
                    case 'open_brakes':
                        broadcast('open_brakes');
                        status = 'open_brakes';
                        name = 'brakes_service';
                        break;
                    case 'close_brakes':
                        broadcast('close_brakes');
                        status = 'close_brakes';
                        name = 'brakes_service';
                        break;
                    default:
                        if (!enable_control)
                        {
                            status = 'The active controller is not a MPC controller';
                            name = 'error';
                        }
                }

                console.log(status);

                ws.send(JSON.stringify({ status: 'success', result: { name: name, status: status } }));
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