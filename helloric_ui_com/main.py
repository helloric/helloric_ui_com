import rclpy
import uuid
import asyncio
from fastapi import FastAPI, WebSocket
from ros_com import init_node
from uvicorn import Config, Server


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

    async def send_json(self, name: str, message: dict):
        if name in self.named_connections:
            await self.named_connections[name].send_json(message)
        else:
            print('Connection %s does not exist', name)

    async def broadcast_json(self, message: dict):
        """respond to all agents."""
        for connection in self.named_connections.values():
            await connection.send_json(message)



class HelloRICMgr(WebSocketConnectionManager):
    def __init__(self):
        self.emotion = -1
        self.last_emotion = -1
        self.speaking = False
        self.last_speaking = False
        self.ros_node = None
        super().__init__()

    async def main_loop(self):
        while True:
            if self.ros_node:
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)
            # TODO: forward data to Client if its new
            if self.emotion != self.last_emotion:
                await self.broadcast_json({'emotion': self.emotion})
                self.last_emotion = self.emotion
            if self.speaking != self.last_speaking:
                await self.broadcast_json({'speaking': self.speaking})
                self.last_speaking = self.speaking
            await asyncio.sleep(0.01)

    async def new_client(self, user_id):
        """a new user connected - send initial data."""
        await self.send_json(user_id, {
            'emotion': self.emotion,
            'speaking': self.speaking
        })


def init_websocket():
    app = FastAPI()
    mgr = HelloRICMgr()
    node = init_node(mgr)
    mgr.ros_node = node

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        user_id = mgr.add(websocket)
        try:
            # broadcast states from the 
            await mgr.new_client(user_id)
            while True:
                data = await websocket.receive_json()
                # Future: data from the UI
        except Exception as e:
            print('Error: %s', e)
        finally:
            mgr.disconnect(user_id)
    return mgr, app


def init_fastapi(logger, loop):
    logger.info('⚡ Provide WebSocket interface using fastapi')
    mgr, app = init_websocket()
    config = Config(app=app, loop=loop, host='0.0.0.0', port=7000)
    server = Server(config)
    return mgr, server, app
