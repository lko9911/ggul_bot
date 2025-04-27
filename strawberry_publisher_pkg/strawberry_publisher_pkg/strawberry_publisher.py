import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from strawberry_publisher_pkg.Strawberry_Vision import detect_and_save
from strawberry_publisher_pkg.utils import load_detected_objects

class StrawberryPublisher(Node):
    def __init__(self):
        super().__init__('strawberry_publisher')
        self.publisher_ = self.create_publisher(String, 'strawberry_positions', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # 2초마다 실행
        self.yolo_path = "model/Strawberry_yolo.pt"
        self.npz_path = "stereo_calibration_result_test.npz"
        self.save_path = "detected_objects.json"

    def timer_callback(self):
        detect_and_save(model_path=self.yolo_path, npz_path=self.npz_path, save_path=self.save_path, time_interval=2000)
        data = load_detected_objects(self.save_path)

        if "detected_objects" in data:
            for obj in data["detected_objects"]:
                msg = String()
                msg.data = json.dumps({
                    "index": obj["index"],
                    "position": {
                        "x": obj["X"],
                        "y": obj["Y"],
                        "z": obj["Z"]
                    }
                })
                self.publisher_.publish(msg)
                self.get_logger().info(f'퍼블리시: {msg.data}')
        else:
            self.get_logger().info("탐지된 객체가 없습니다.")

def main(args=None):
    rclpy.init(args=args)
    node = StrawberryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
