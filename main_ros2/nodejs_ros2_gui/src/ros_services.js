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

// ros2 service call /stop_mpc_service mpc_interfaces/srv/SimpleCommand "{}"
async function stopService() {
    const client = node.createClient('mpc_interfaces/srv/SimpleCommand', 'stop_mpc_service');
    return callService(client, {});
}

//ros2 service call /traj_switch_service mpc_interfaces/srv/TrajectoryCommand "{traj_select: 1}"
async function switchTrajectory(trajSelect) {
    const client = node.createClient('mpc_interfaces/srv/TrajectoryCommand', 'traj_switch_service');
    return callService(client, { traj_select: trajSelect });
}

// ros2 service call /controller_manager/load_controller controller_manager_msgs/srv/LoadController "{name: 'move_to_start_example_controller'}"
// ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: 'move_to_start_example_controller'}"
async function load_and_configure_controller_intern(controller_name) {
    var clients = [node.createClient('controller_manager_msgs/srv/LoadController',      'controller_manager/load_controller'),
                   node.createClient('controller_manager_msgs/srv/ConfigureController', 'controller_manager/configure_controller')];
    var requests = [{ name: controller_name }, { name: controller_name }];
    var status_strings = ['load ' + controller_name, 'configure ' + controller_name];

    return callMultipleServices(clients, requests, status_strings);
}

async function load_and_configure_controller(controller_name, home_robot=null, ws=null, data=null) {
    return load_and_configure_controller_intern(controller_name)
        .then((result) => {
            console.log('\nService responses:', result);
            if (home_robot) {
                return home_robot(ws, data);
            }
            else
            {
                return { result: 'Success', status: 'Controller loaded and configured' };
            }
        })
        .catch((error) => {
            console.error('\nError occurred:', error.message);
            console.error('Did you forget to start ROS2?\n');
            return { result: 'Error occurred', status: error.message };
        });
}

// ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{deactivate_controllers: ['move_to_start_example_controller'], activate_controllers: ['mpc_pinocchio_controller'], strictness: 2}"
async function switch_to_home_control() {
    const client = node.createClient('controller_manager_msgs/srv/SwitchController', 'controller_manager/switch_controller');
    return callService(client, { activate_controllers: ['move_to_start_example_controller'], deactivate_controllers: ['mpc_pinocchio_controller'], strictness: 2 });
}

async function switch_to_mpc_control() {
    const client = node.createClient('controller_manager_msgs/srv/SwitchController', 'controller_manager/switch_controller');
    return callService(client, { deactivate_controllers: ['move_to_start_example_controller'], activate_controllers: ['mpc_pinocchio_controller'], strictness: 2 });
}

function objectToString(obj) {
    const entries = Object.entries(obj);
    return `{ ${entries.map(([key, value]) => `${key}: ${value}`).join(', ')} }`;
}


function callService(client, request) {
    return new Promise((resolve, reject) => {
        // First, check if the service is available
        client.waitForService(1000).then(() => {
            // Create a timeout for the sendRequest
            const timeout = setTimeout(() => {
                reject(new Error('Error: Service request timed out'));
            }, 1000); // Set timeout duration (e.g., 1 second)

            client.sendRequest(request, (response) => {
                clearTimeout(timeout); // Clear the timeout if response is received
                if (response) {
                    resolve(response); // Resolve with the entire response object
                } else {
                    reject(new Error('Error: No response received from service'));
                }
            });
        }).catch((err) => {
            // Handle case where service is not available
            reject(new Error('Error: Service not available: ' + err.message));
        });
    });
}

async function callMultipleServices(clients, requests, status_strings) {
    if (clients.length !== requests.length) {
        throw new Error('Error: The number of clients and requests must match.');
    }

    let results = "";

    for (let i = 0; i < clients.length; i++) {
        const client = clients[i];
        const request = requests[i];
        const status_string = status_strings[i];

        // Wait for the service to be available
        try {
            await client.waitForService(1000);
        } catch (err) {
            throw new Error(`Error: ${status_string}: Service ${i + 1} not available: ${err.message}`);
        }

        try {
            const response = await new Promise((resolve, reject) => {
                const timeout = setTimeout(() => {
                    reject(new Error(`Error: ${status_string}: Service ${i + 1} request timed out`));
                }, 1000); // Increase timeout to 2 seconds for better reliability

                client.sendRequest(request, (response) => {
                    clearTimeout(timeout);
                    if (response) {
                        resolve(response);
                    } else {
                        reject(new Error(`Error: ${status_string}: No response received from service ${i + 1}`));
                    }
                });
            });

            results += status_string + objectToString(response)
            console.log(`Error: ${status_string}: Response from service ${i + 1}:`, response);
        } catch (err) {
            throw err; // or you can handle it gracefully and continue
        }
    }

    console.log("All responses combined:");
    console.log(results); // or however you want to process results

    return results; // Return the array of responses
}

module.exports = { startService, resetService, stopService, switchTrajectory, load_and_configure_controller, switch_to_home_control, switch_to_mpc_control };