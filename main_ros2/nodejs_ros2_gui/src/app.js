const http = require('http');
const WebSocket = require('ws');
const express = require('express');
const path = require('path');
const { startService, resetService, stopService, switchTrajectory, load_and_configure_controller, switch_to_home_control, switch_to_mpc_control } = require('./ros_services');
const { searchTrajectoryNames } = require('./search_m_file');

// Express setup to serve HTML files
const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });
const clients = new Set();

var traj_path = path.join(__dirname, '..', '..', '..', 'utils', 'matlab_init_general', 'param_traj_fr3_no_hand_6dof.m');

app.use(express.static(path.join(__dirname, '..', 'public')));

app.get('/', function(req, res) {
   res.sendFile(__dirname + '/index.html');
});

load_and_configure_controller('move_to_start_example_controller');

wss.on('connection', (ws) => {
    console.log('WebSocket connection established');

    clients.add(ws);
  
    ws.on('close', () => {
      clients.delete(ws);
    });

    ws.on('message', async (message) => {
        // ws.send(JSON.stringify({ status: 'success', result: "test" }));
        const data = JSON.parse(message);
        var status = 'none', name = 'ros_service';
        try {
            let result;
            switch (data.command) {
                case 'start':
                    result = await startService();
                    status = result.status;
                    console.log("end2");
                    break;
                case 'reset':
                    result = await resetService();
                    status = result.status;
                    break;
                case 'stop':
                    result = await stopService();
                    status = result.status;
                    break;
                case 'trajectory_selection':
                    result = await switchTrajectory(data.traj_select);
                    status = result.status;
                    break;
                case 'home':
                    try{
                        result = await switch_to_home_control();
                        console.log(result);
                        status='homing in progress';
                        if(result.ok)
                        {
                            setTimeout(async function(){
                                result = await switch_to_mpc_control();
                                result.status='homing done';
                                console.log(result);
                                ws.send(JSON.stringify({ status: 'success', result: { name: 'ros_service', status: result.status } }));
                            }, data.delay);
                        }
                        else
                        {
                            status='Error: move_to_start_example_controller was not loaded. restarting.';
                            throw new Error('It seems that move_to_start_example_controller was not loaded')
                        }
                    } catch (error) {
                        console.log("trying to readding move_to_start_example_controller");
                        load_and_configure_controller('move_to_start_example_controller');
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
                    status = 'Unknown command';
                    name = 'error';
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

