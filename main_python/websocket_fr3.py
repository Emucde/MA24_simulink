# websocket_fr3.py
# This script implements a WebSocket server that handles multiple clients.
# It listens for incoming messages from clients and broadcasts them to all connected clients.

# It is used to update the html plot data in the web interface if the MPC is restarted or the simulation is reset.

import asyncio
import websockets

connected_clients = set()

async def handler(websocket):
    connected_clients.add(websocket)
    print("Client connected")
    try:
        async for message in websocket:  # Warten auf Nachrichten vom Client
            print(f"Nachricht empfangen: {message}")
            await broadcast(message)  # Nachricht an alle Clients senden
    finally:
        connected_clients.remove(websocket)
        print("Client disconnected")

async def broadcast(message):
    print("Broadcasting message")
    if connected_clients:  # Nur senden, wenn es verbundene Clients gibt
        await asyncio.gather(
            *[client.send(message) for client in connected_clients],
            return_exceptions=True
        )

async def websocket_server():
    try:
        port = 8765
        server = await websockets.serve(handler, "localhost", port)
    finally:
        await broadcast('reload')
        await server.wait_closed()

try:
    asyncio.run(websocket_server())
except KeyboardInterrupt:
    print("Server interrupted. Exiting...")