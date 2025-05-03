import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import asyncio
import websockets
import json

class WebSocketToROSPublisher(Node):
    def __init__(self):
        super().__init__('websocket_ros_publisher')
        self.publisher_ = self.create_publisher(Point, 'ik_goal', 10)
        self.get_logger().info("✅ WebSocket ROS Publisher Node Started")

    async def echo_and_publish(self, websocket):
        async for message in websocket:
            self.get_logger().info(f"📩 Received: {message}")
            try:
                data = json.loads(message)
                msg = Point()
                msg.x = data['X']
                msg.y = data['Y']
                msg.z = data['Z']
                self.publisher_.publish(msg)
                await websocket.send("✅ Received and published")
            except Exception as e:
                self.get_logger().error(f"❌ Error: {e}")
                await websocket.send("❌ Failed to parse message")

async def main_async():
    rclpy.init()
    node = WebSocketToROSPublisher()
    start_server = websockets.serve(node.echo_and_publish, "0.0.0.0", 8765)
    async with start_server:
        await asyncio.Future()  # run forever

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()
