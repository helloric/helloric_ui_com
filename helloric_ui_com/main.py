import rospy
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
            rospy.logerr('Connection %s does not exist', name)

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
        super().__init__()

    async def main_loop(self):
        while True:
            # TODO: forward data to Client if its new
            if self.emotion != self.last_emotion:
                self.broadcast_json({'emotion': self.emotion})
                self.last_emotion = self.emotion
            if self.speaking != self.last_speaking:
                self.broadcast_json({'speaking': self.speaking})
                self.last_speaking = self.speaking
            await asyncio.sleep(0.01)


def init_websocket():
    app = FastAPI()
    mgr = HelloRICMgr()
    node = init_node(mgr)

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        user_id = mgr.add(websocket)
        try:
            while True:
                data = await websocket.receive_json()
                # Future: data from the UI
        except Exception as e:
            rospy.logerr('Error: %s', e)
        finally:
            mgr.disconnect(user_id)
    return mgr, app


def init_fastapi(logger, loop):
    logger.info('⚡ Provide WebSocket interface using fastapi')
    mgr, app = init_websocket()
    config = Config(app=app, loop=loop, host='0.0.0.0', port=7000)
    server = Server(config)
    return mgr, server, app
