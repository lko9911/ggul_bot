import rclpy
from rclpy.node import Node
import asyncio
import websockets

class WebSocketROSNode(Node):
    def __init__(self):
        super().__init__('websocket_ros_publisher')
        self.get_logger().info("ROS 2 WebSocket Publisher Node Started")

    async def echo(self, websocket):
        async for message in websocket:
            self.get_logger().info(f"Received message: {message}")
            await websocket.send(f"Echo: {message}")

async def main_async():
    rclpy.init()
    node = WebSocketROSNode()
    start_server = websockets.serve(node.echo, "0.0.0.0", 8765)
    async with start_server:
        await asyncio.Future()  # run forever

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()
