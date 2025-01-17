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

// ros2 service call /controller_manager/load_controller controller_manager_msgs/srv/LoadController "{name: 'move_to_start_example_controller'}"
async function load_and_configure_controller_intern(controller_name) {
    const client1 = node.createClient('controller_manager_msgs/srv/LoadController', 'controller_manager/load_controller');
    const request1 = {name: controller_name};
    const client2 = node.createClient('controller_manager_msgs/srv/ConfigureController', 'controller_manager/configure_controller');
    const request2 = {name: controller_name};
    return callTwoServices(client1, request1, client2, request2);
}

async function load_and_configure_controller(controller_name) {
    load_and_configure_controller_intern(controller_name)
        .then((result) => {
            console.log('Service responses:', result);
        })
        .catch((error) => {
            console.error('Error occurred:', error.message);
            console.error('Did you forget to start ROS2?:', error.message);
        });
}

// ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{deactivate_controllers: ['move_to_start_example_controller'], activate_controllers: ['mpc_pinocchio_controller'], strictness: 2}"
async function switch_to_home_control() {
    const client = node.createClient('controller_manager_msgs/srv/SwitchController', 'controller_manager/switch_controller');
    return callService(client, {activate_controllers: ['move_to_start_example_controller'], deactivate_controllers: ['mpc_pinocchio_controller'], strictness: 2});
}

async function switch_to_mpc_control() {
    const client = node.createClient('controller_manager_msgs/srv/SwitchController', 'controller_manager/switch_controller');
    return callService(client, {deactivate_controllers: ['move_to_start_example_controller'], activate_controllers: ['mpc_pinocchio_controller'], strictness: 2});
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

function callTwoServices(client1, request1, client2, request2) {
    return new Promise((resolve, reject) => {
        // First, check if the service is available
        client1.waitForService(1000).then(() => {
            // Create a timeout for the sendRequest
            const timeout1 = setTimeout(() => {
                reject(new Error('Service1 request timed out'));
            }, 1000); // Set timeout duration (e.g., 1 second)
            client1.sendRequest(request1, (response1) => {
                clearTimeout(timeout1); // Clear the timeout if response is received
                if (response1) {
                    client2.waitForService(1000).then(() => {
                        const timeout2 = setTimeout(() => {
                            reject(new Error('Service2 request timed out'));
                        }, 1000); // Set timeout duration (e.g., 1 second)
                        client2.sendRequest(request2, (response2) => {
                            clearTimeout(timeout2); // Clear the timeout if response is received
                            if (response2) {
                                resolve(response1 + response2); // Resolve with the entire response object
                            } else {
                                reject(new Error('No response received from service2'));
                            }
                        });
                    }).catch((err) => {
                        // Handle case where service is not available
                        reject(new Error('Service2 not available: ' + err.message));
                    });
                } else {
                    reject(new Error('No response received from service1'));
                }
            });
        }).catch((err) => {
            // Handle case where service is not available
            reject(new Error('Service1 not available: ' + err.message));
        });
    });
}

module.exports = { startService, resetService, stopService, switchTrajectory, load_and_configure_controller, switch_to_home_control, switch_to_mpc_control };