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
    port = 8765
    server = await websockets.serve(handler, "localhost", port)
    await broadcast('reload')
    await server.wait_closed()

asyncio.run(websocket_server())
