const http = require('http');
const WebSocket = require('ws');
const express = require('express');
const path = require('path');
const { startService, resetService, stopService, switchTrajectory } = require('./ros_services');
const { searchTrajectoryNames } = require('./search_m_file');

// Express setup to serve HTML files
const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

var traj_path = path.join(__dirname, '..', '..', '..', 'utils', 'matlab_init_general', 'param_traj_fr3_no_hand_6dof.m');

app.use(express.static(path.join(__dirname, '..', 'public')));

app.get('/', function(req, res) {
   res.sendFile(__dirname + '/index.html');
});

wss.on('connection', (ws) => {
    console.log('WebSocket connection established');

    ws.on('message', async (message) => {
        // ws.send(JSON.stringify({ status: 'success', result: "test" }));
        const data = JSON.parse(message);
        try {
            let result;
            switch (data.command) {
                case 'start':
                    result = await startService();
                    console.log("end2");
                    break;
                case 'reset':
                    result = await resetService();
                    break;
                case 'stop':
                    result = await stopService();
                    break;
                case 'trajectory_selection':
                    result = await switchTrajectory(data.traj_select);
                    break;
                default:
                    result = 'Unknown command';
            }

            console.log(result);

            ws.send(JSON.stringify({ status: 'success', result: { name: 'ros_service', status: result.status } }));
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

