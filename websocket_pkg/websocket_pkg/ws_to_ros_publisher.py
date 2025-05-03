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
        self.pose_publisher = self.create_publisher(Pose, '/target_pose', 10)

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
                data = json.loads(message)
                pose_msg = Pose()
                pose_msg.position.x = data['X']
                pose_msg.position.y = data['Y']
                pose_msg.position.z = data['Z']
                pose_msg.orientation.w = 1.0

                self.pose_publisher.publish(pose_msg)
                self.get_logger().info(f"📤 Published Pose: {pose_msg.position}")

                await websocket.send("✅ Pose published")
            except Exception as e:
                self.get_logger().error(f"❌ Error: {e}")
                await websocket.send("❌ Failed to parse message")

    def joint_state_callback(self, msg: JointState):
        self.get_logger().info(f"🔧 Received JointState with {len(msg.name)} joints")

async def ros_spin_once(node):
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        await asyncio.sleep(0.01)

async def main_async():
    rclpy.init()
    node = WebSocketToROSPublisher()

    # 웹소켓 서버와 ROS spin을 함께 돌림
    server = await websockets.serve(node.echo_and_publish, "0.0.0.0", 8765)
    print("🌐 WebSocket server started on port 8765")

    await asyncio.gather(
        ros_spin_once(node),
        asyncio.Future()  # 웹소켓이 살아 있는 한 무한 대기
    )

if __name__ == '__main__':
    asyncio.run(main_async())
