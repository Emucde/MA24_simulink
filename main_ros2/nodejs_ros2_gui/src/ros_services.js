const rclnodejs = require('rclnodejs');

rclnodejs.init().then(() => {
    console.log('ROS 2 Node initialized');
});
const node = new rclnodejs.Node('web_ros_bridge');
node.spin();

// Service Call Handlers
async function startService() {
    console.log("start");
    const client = node.createClient('mpc_interfaces/srv/SimpleCommand', '/start_mpc_service');
    return callService(client, {})
}

async function resetService() {
    const client = node.createClient('mpc_interfaces/srv/SimpleCommand', 'reset_mpc_service');
    return callService(client, {});
}

async function stopService() {
    const client = node.createClient('mpc_interfaces/srv/SimpleCommand', 'stop_mpc_service');
    return callService(client, {});
}

async function switchTrajectory(trajSelect) {
    const client = node.createClient('mpc_interfaces/srv/TrajectoryCommand', 'traj_switch_service');
    return callService(client, { traj_select: trajSelect });
}

function callService(client, request) {
    return new Promise((resolve, reject) => {
        // First, check if the service is available
        client.waitForService(1000).then(() => {
            // Create a timeout for the sendRequest
            const timeout = setTimeout(() => {
                reject(new Error('Service request timed out'));
            }, 1000); // Set timeout duration (e.g., 1 second)

            client.sendRequest(request, (response) => {
                clearTimeout(timeout); // Clear the timeout if response is received
                if (response) {
                    resolve(response); // Resolve with the entire response object
                } else {
                    reject(new Error('No response received from service'));
                }
            });
        }).catch((err) => {
            // Handle case where service is not available
            reject(new Error('Service not available: ' + err.message));
        });
    });
}

module.exports = { startService, resetService, stopService, switchTrajectory };