# main_python/node_data_logger.py
# This script is responsible for logging data from shared memory and sending it to a WebSocket server
# It connects to the WebSocket server at ws://localhost:8081 and sends data every 10 iterations
# It uses shared memory to read control data and a semaphore to synchronize access
# The script runs indefinitely, retrying every 5 seconds in case of connection errors

# It is used for the online plot in the web interface to visualize the torque data from the robot

import asyncio
import websockets
import posix_ipc
from multiprocessing import shared_memory
import numpy as np
import sys, os
sys.path.append(os.path.dirname(os.path.abspath('./utils_python')))
from utils_python.utils import initialize_shared_memory
import time

print("Starting data logging...")

async def start_data_logging():
    i = 0
    shm_objects, shm_data = initialize_shared_memory()
    data_logger_semaphore = posix_ipc.Semaphore("/data_logger_semaphore")
    read_control_data = shm_data["read_control_data"]
    while True:
        try:
            async with websockets.connect("ws://localhost:8081") as websocket:
                while True:
                    data_logger_semaphore.acquire()

                    if np.mod(i, 10) == 0:
                        data_to_send = read_control_data.astype(np.float32).tobytes()
                        await websocket.send(data_to_send)
                    i=i+1
                    if i >= 1000:
                        i = 0
        except asyncio.CancelledError:
            print("Data logging interrupted. Exiting...")
        except Exception as e:
            print("Error in data logging: ", e)
            print("Retrying in 5 second...")
        time.sleep(5)

# Run the data logging coroutine
asyncio.run(start_data_logging())