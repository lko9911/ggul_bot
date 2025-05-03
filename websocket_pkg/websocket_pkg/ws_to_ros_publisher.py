import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import asyncio
import websockets
import json

class WebSocketToROSPublisher(Node):
    def __init__(self):
        super().__init__('websocket_ros_publisher')

        # WebSocket으로 받은 Pose 메시지 퍼블리셔
        self.pose_publisher = self.create_publisher(Pose, '/target_pose', 10)

        # JointState 구독자 추가
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info("✅ WebSocket ROS Publisher Node Started")

    async def echo_and_publish(self, websocket):
        async for message in websocket:
            self.get_logger().info(f"📩 Received: {message}")
            try:
                # JSON → Pose 메시지로 변환
                data = json.loads(message)
                pose_msg = Pose()
                pose_msg.position.x = data['X']
                pose_msg.position.y = data['Y']
                pose_msg.position.z = data['Z']
                # Orientation은 기본값(0)으로 둡니다.
                pose_msg.orientation.w = 1.0  # 단위 쿼터니언

                self.pose_publisher.publish(pose_msg)
                self.get_logger().info(f"📤 Published Pose: {pose_msg.position}")

                await websocket.send("✅ Pose published")
            except Exception as e:
                self.get_logger().error(f"❌ Error: {e}")
                await websocket.send("❌ Failed to parse message")

    def joint_state_callback(self, msg: JointState):
        self.get_logger().info(f"🔧 Received JointState with {len(msg.name)} joints")
        # 필요한 경우 데이터를 활용할 수 있도록 저장하거나 처리

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
