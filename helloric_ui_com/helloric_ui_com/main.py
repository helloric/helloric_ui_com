import asyncio

# Websocket imports
from fastapi import FastAPI, WebSocket
from starlette.websockets import WebSocketDisconnect
from uvicorn import Config, Server

# ROS imports
import rclpy
from std_msgs.msg import String

from .ros_com import init_node
from .utils import WebSocketConnectionManager


class HelloRICMgr(WebSocketConnectionManager):
    def __init__(self):
        # TODO: create a generic "message forwarding"
        #       that gets data from any ros topic service and
        #       forwards it to the websocket, ideally with
        #       user management.
        #       but also receives data from the websocket and
        #       forwards it to ROS

        self.new_queue = False
        self.queue = []
        self.release_mic = False

        self.ros_node = None
        super().__init__()

    async def main_loop(self):
        while rclpy.ok():
            if self.ros_node:
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)
            # forward data to Client if its new
            await self.update_data()
            await asyncio.sleep(0.01)

    async def update_data(self):
        if self.new_queue:
            await self.broadcast_json({
                'messages': self.queue,
                'release_mic': self.release_mic})
            self.new_queue = False
            self.queue = []

    async def new_client(self, user_id):
        """a new user connected - send initial data."""
        await self.send_json(user_id, {
            'emotion': 0,
            'speaking': False
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
            # send initial emotion and speaking state to UI
            await mgr.new_client(user_id)
            while True:
                data = await websocket.receive_json()
                print('received new message')
                if data.get('audio_data') is not None:
                    print('received audio data')
                    node.audio.publish(String(data=data.get('audio_data')))
                # Future: data from the UI
        except WebSocketDisconnect as wsd:
            print('Websocket disconnected - Error:', wsd)
        finally:
            mgr.disconnect(user_id)
    return mgr, app


def init_fastapi(logger, loop, host='0.0.0.0', port=7000):
    logger.info('⚡ Provide WebSocket interface using fastapi')
    mgr, app = init_websocket()
    config = Config(app=app, loop=loop, host=host, port=port)
    server = Server(config)
    return mgr, server, app
