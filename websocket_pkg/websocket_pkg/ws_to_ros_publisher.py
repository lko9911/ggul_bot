import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets

class WebSocketROSNode(Node):
    def __init__(self):
        super().__init__('websocket_ros_publisher')
        self.publisher_ = self.create_publisher(String, 'websocket_topic', 10)
        self.get_logger().info("✅ ROS 2 WebSocket Publisher Node Started")

    async def echo(self, websocket):
        async for message in websocket:
            self.get_logger().info(f"🌐 WebSocket 수신 메시지: {message}")
            
            # ROS2 토픽으로 publish
            msg = String()
            msg.data = message
            self.publisher_.publish(msg)
            self.get_logger().info(f"📤 ROS2 토픽 발행: {message}")

            await websocket.send(f"서버에서 수신 완료: {message}")

async def main_async():
    rclpy.init()
    node = WebSocketROSNode()

    # asyncio task를 rclpy와 함께 유지
    loop = asyncio.get_running_loop()
    loop.create_task(websockets.serve(node.echo, "0.0.0.0", 8765))

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()
