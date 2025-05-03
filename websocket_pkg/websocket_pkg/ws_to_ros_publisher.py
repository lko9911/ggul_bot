import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import asyncio
import websockets
import json

class WebSocketToROSPublisher(Node):
    def __init__(self):
        super().__init__('websocket_ros_publisher')
        self.publisher_ = self.create_publisher(Pose, '/target_pose', 10)
        self.get_logger().info("✅ WebSocket ROS Publisher Node Started")

    async def echo_and_publish(self, websocket):
        async for message in websocket:
            self.get_logger().info(f"📩 Received: {message}")
            try:
                data = json.loads(message)

                # Pose 메시지 생성
                msg = Pose()
                msg.position.x = data['X']
                msg.position.y = data['Y']
                msg.position.z = data['Z']
                
                # 방향 정보는 기본값(회전 없음)
                msg.orientation.x = 0.0
                msg.orientation.y = 0.0
                msg.orientation.z = 0.0
                msg.orientation.w = 1.0  # 회전하지 않음을 의미 (단위 쿼터니언)

                # Pose 메시지 발행
                self.publisher_.publish(msg)
                self.get_logger().info(f"📤 Published Pose: {msg}")
                await websocket.send("✅ Received and published Pose to /target_pose")

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

if __name__ == '__main__':
    main()
