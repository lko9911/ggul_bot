import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import asyncio
import websockets
import json

class IKWebSocketPublisher(Node):
    def __init__(self):
        super().__init__('ik_ws_publisher')
        self.subscription = self.create_subscription(
            JointState,  # JointState 메시지로 변경
            '/joint_states',  # 퍼블리시된 조인트 상태 토픽
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        joint_values = msg.position  # JointState의 position 필드에서 조인트 값 추출
        self.get_logger().info(f'Received Joint States: {joint_values}')

        # 웹소켓으로 전송
        asyncio.run(self.send_to_websocket(joint_values))

    async def send_to_websocket(self, joint_values):
        uri = "ws://192.168.150.149:8765"  # 웹소켓 서버 주소
        data = {
            "joint_values": joint_values
        }
        try:
            async with websockets.connect(uri) as websocket:
                await websocket.send(json.dumps(data))
        except Exception as e:
            self.get_logger().warn(f'WebSocket 전송 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IKWebSocketPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
