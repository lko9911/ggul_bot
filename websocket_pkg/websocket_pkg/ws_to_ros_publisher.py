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
        # 웹소켓 메시지를 저장할 변수 초기화
        self.received_message = None

    async def echo_and_publish(self, websocket):
        async for message in websocket:
            self.get_logger().info(f"📩 Received: {message}")
            try:
                # 메시지를 파싱하여 Point 메시지로 변환
                data = json.loads(message)
                msg = Point()
                msg.x = data['X']
                msg.y = data['Y']
                msg.z = data['Z']
                
                # 변환된 메시지를 변수에 저장
                self.received_message = msg
                self.get_logger().info(f"📤 Published: {msg}")

                # ROS 토픽으로 퍼블리시
                self.publisher_.publish(msg)
                
                # 웹소켓 클라이언트에 응답
                await websocket.send("✅ Received and published")
            except Exception as e:
                self.get_logger().error(f"❌ Error: {e}")
                await websocket.send("❌ Failed to parse message")

    def get_received_message(self):
        """받은 메시지를 반환하는 함수"""
        return self.received_message

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
