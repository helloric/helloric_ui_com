import uuid
from fastapi import WebSocket


class WebSocketConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []
        # connections with names
        self.named_connections: dict = {}

    def add(self, websocket: WebSocket, name: str = None):
        if not name:
            name = str(uuid.uuid4())
        self.named_connections[name] = websocket
        return name

    def disconnect(self, name: str):
        """Client (agent or user) disconnected."""
        if name in self.named_connections:
            del self.named_connections[name]

    async def send_bytes(self, name: str, message: bytes):
        if name in self.named_connections:
            await self.named_connections[name].send_bytes(message)
        else:
            self.logger.error('😖 Connection %s does not exist', name)

    async def broadcast_bytes(self, message: bytes):
        """respond to all agents."""
        for connection in self.named_connections.values():
            await connection.send_bytes(message)

    async def send_json(self, name: str, message: dict):
        if name in self.named_connections:
            await self.named_connections[name].send_json(message)
        else:
            print('Connection %s does not exist', name)

    async def broadcast_json(self, message: dict):
        """respond to all agents."""
        for connection in self.named_connections.values():
            await connection.send_json(message)