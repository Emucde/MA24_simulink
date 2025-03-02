const rclnodejs = require('rclnodejs');

let node;

async function initializeNode() {
    if (!rclnodejs._initialized) {
        await rclnodejs.init();
    }
    console.log('ROS 2 Node initialized');
    node = new rclnodejs.Node('web_ros_bridge');
    node.spin();
    return node;
}

async function restartNode() {
    if (node) {
      console.log('Shutting down existing node...');
      node.destroy();
    }
    
    if (rclnodejs.shutdown) {
      console.log('Shutting down rclnodejs...');
      rclnodejs.shutdown();
    }
    
    await rclnodejs.init();
    console.log('ROS 2 Node initialized');
    node = new rclnodejs.Node('web_ros_bridge');
    node.spin();
    return node;
  }
  

// Initial node start
initializeNode().catch(error => {
    console.error('Error initializing ROS 2 Node:', error);
});

async function checkROSConnection() {
    try {
        // Get the list of available topics
        const nodeNames = node.getNodeNames();

        if(nodeNames.includes('controller_manager'))
            return true;
        else
            return false;
    } catch (error) {
        console.error('ROS connection is not available:', error.message);
        return false;
    }
  }

// Service Call Handlers
async function startService(active_controller_name) {
    console.log("start");
    const client = node.createClient('mpc_interfaces/srv/SimpleCommand', '/'+active_controller_name+'/start_mpc_service');
    return callService(client, {})
}


async function resetService(active_controller_name) {
    const client = node.createClient('mpc_interfaces/srv/SimpleCommand', '/'+active_controller_name+'/reset_mpc_service');
    return callService(client, {});
}
// ros2 service call /stop_mpc_service mpc_interfaces/srv/SimpleCommand "{}"
async function stopService(active_controller_name) {
    const client = node.createClient('mpc_interfaces/srv/SimpleCommand', '/'+active_controller_name+'/stop_mpc_service');
    return callService(client, {});
}

//ros2 service call /traj_switch_service mpc_interfaces/srv/TrajectoryCommand "{traj_select: 1}"
async function switchTrajectory(active_controller_name, trajSelect) {
    const client = node.createClient('mpc_interfaces/srv/TrajectoryCommand', '/'+active_controller_name+'/traj_switch_service');
    return callService(client, { traj_select: trajSelect });
}

// ros2 service call /controller_manager/load_controller controller_manager_msgs/srv/LoadController "{name: 'move_to_start_example_controller'}"
// ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: 'move_to_start_example_controller'}"
async function load_and_configure_controller_intern(controller_name) {
    var clients = [node.createClient('controller_manager_msgs/srv/LoadController', 'controller_manager/load_controller'),
    node.createClient('controller_manager_msgs/srv/ConfigureController', 'controller_manager/configure_controller')];
    var requests = [{ name: controller_name }, { name: controller_name }];
    var status_strings = ['load ' + controller_name, 'configure ' + controller_name];

    return callMultipleServices(clients, requests, status_strings);
}

async function unloadController(controllerName) {
    const client = node.createClient(
        'controller_manager_msgs/srv/UnloadController',
        '/controller_manager/unload_controller'
    );
    return callService(client, { name: controllerName });
}

// ros2 param set /move_to_start_example_controller enable_single_joint_homing false
async function set_enable_single_joint_homing(active_controller_name, enable) {
    const client = node.createClient('rcl_interfaces/srv/SetParameters', '/' + active_controller_name + '/set_parameters');
    const request = {
        parameters: [{
            name: 'enable_single_joint_homing',
            value: {
                type: 1,  // 1 corresponds to PARAMETER_BOOL
                bool_value: enable
            }
        }]
    };

    try {
        const response = await callService(client, request);
        console.log('Parameter set response:', response);
        return response;
    } catch (error) {
        console.error('Error setting parameter:', error.message);
        throw error;
    }
}


// async function load_and_configure_controller(controller_name, home_robot = null, ws = null, data = null) {
//     return load_and_configure_controller_intern(controller_name)
//         .then((result) => {
//             console.log('\nService responses:', result);
//             if (home_robot) {
//                 return home_robot(ws, data);
//             }
//             else {
//                 return { result: 'Success', status: 'Controller loaded and configured' };
//             }
//         })
//         .catch((error) => {
//             console.error('\nError occurred:', error.message);
//             console.error('Did you forget to start ROS2?\n');
//             return { result: 'Error occurred', status: error.message };
//         });
// }

// ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{deactivate_controllers: ['move_to_start_example_controller'], activate_controllers: ['mpc_pinocchio_controller'], strictness: 2}"
async function switch_control(old_control_name, new_control_name) {
    if(old_control_name === new_control_name) {
        return { ok : true, status: 'Already in ' + new_control_name };
    }
    const client = node.createClient('controller_manager_msgs/srv/SwitchController', 'controller_manager/switch_controller');
    return callService(client, { deactivate_controllers: [old_control_name], activate_controllers: [new_control_name], strictness: 2 });
}

async function switch_casadi_mpc(mpc_type) {
    const client = node.createClient('mpc_interfaces/srv/CasadiMPCTypeCommand', '/mpc_casadi_controller/mpc_switch_service');
    return callService(client, { mpc_type: BigInt(mpc_type) });
}

async function switch_workspace_controller(workspace_controller_type) {
    const client = node.createClient('mpc_interfaces/srv/WorkspaceControllerTypeCommand', '/conventional_workspace_controller/workspace_controller_switch_service');
    return callService(client, { workspace_controller_type: BigInt(workspace_controller_type) });
}

async function activate_control(new_control_name) {
    const client = node.createClient('controller_manager_msgs/srv/SwitchController', 'controller_manager/switch_controller');
    return callService(client, { activate_controllers: [new_control_name], strictness: 2 });
}

async function deactivate_control(new_control_name) {
    const client = node.createClient('controller_manager_msgs/srv/SwitchController', 'controller_manager/switch_controller');
    return callService(client, { deactivate_controllers: [new_control_name], strictness: 2 });
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

async function get_controller_info() {
    const client = node.createClient(
        'controller_manager_msgs/srv/ListControllers',
        '/controller_manager/list_controllers'
    );
    const response = await callService(client, {});

    var controller_names = response.controller.map(controller => controller.name);
    var controller_active_states = response.controller.map(controller => controller.state);
    var active_controller_idx = controller_active_states.findIndex(state => state === 'active');
    var active_controller_name = controller_names[active_controller_idx];
    var name = controller_names[active_controller_idx];

    // if (active_controller_name === undefined) {
    //     active_controller_name = controller_names[0];
    //     active_controller_idx = 0;
    //     const switchClient = node.createClient(
    //         'controller_manager_msgs/srv/SwitchController',
    //         '/controller_manager/switch_controller'
    //     );

    //     const deactivateRequest = {
    //         activate_controllers: [],
    //         deactivate_controllers: [active_controller_name],
    //         strictness: 2,
    //         start_asap: false,
    //         timeout: { sec: 0, nanosec: 0 }
    //     };

    //     try {
    //         const deactivateResponse = await callService(switchClient, deactivateRequest);
    //         console.log('Deactivate response:', deactivateResponse);
    //     } catch (error) {
    //         console.error('Error deactivating controller:', error.message);
    //         throw error;
    //     }

    //     const unloadClient = node.createClient(
    //         'controller_manager_msgs/srv/UnloadController',
    //         '/controller_manager/unload_controller'
    //     );

    //     try {
    //         const unloadResponse = await callService(unloadClient, { name: active_controller_name });
    //         console.log('Unload response:', unloadResponse);
    //     } catch (error) {
    //         console.error('Error unloading controller:', error.message);
    //         throw error;
    //     }

    //     const loadClient = node.createClient(
    //         'controller_manager_msgs/srv/LoadController',
    //         '/controller_manager/load_controller'
    //     );

    //     try {
    //         const loadResponse = await callService(loadClient, { name: active_controller_name });
    //         console.log('Load response:', loadResponse);
    //     } catch (error) {
    //         console.error('Error loading controller:', error.message);
    //         throw error;
    //     }

    //     const configureClient = node.createClient(
    //         'controller_manager_msgs/srv/ConfigureController',
    //         '/controller_manager/configure_controller'
    //     );

    //     try {
    //         const configureResponse = await callService(configureClient, { name: active_controller_name });
    //         console.log('Configure response:', configureResponse);
    //     } catch (error) {
    //         console.error('Error configuring controller:', error.message);
    //         throw error;
    //     }

    //     const activateRequest = {
    //         activate_controllers: [active_controller_name],
    //         deactivate_controllers: [],
    //         strictness: 2,
    //         start_asap: false,
    //         timeout: { sec: 0, nanosec: 0 }
    //     };

    //     try {
    //         const activateResponse = await callService(switchClient, activateRequest);
    //         console.log('Activate response:', activateResponse);
    //     } catch (error) {
    //         console.error('Error activating controller:', error.message);
    //         throw error;
    //     }
    // }

    return { controller_names, controller_active_states, active_controller_idx, active_controller_name, ok: true, name: name };
}

module.exports = { checkROSConnection, restartNode, startService, resetService, stopService, switchTrajectory, switch_control, switch_casadi_mpc, switch_workspace_controller, deactivate_control, activate_control, get_controller_info, objectToString, set_enable_single_joint_homing};