import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2

from .ObjectDetector import ObjectDetector

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.publisher_ = self.create_publisher(String, 'detections', 10)
        self.cap = cv2.VideoCapture(0)  # Adjust if using USB camera index or file

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.detector = ObjectDetector("yolo11n.pt")  # Initialize ObjectDetector
        self.fixed_z = 50.0  # Fixed depth in cm, can be updated from other sources later
        self.get_logger().info('Camera opened. Starting detection...')

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            print(f"Frame shape: {frame.shape}")
            if not ret:
                self.get_logger().warn("Failed to read frame.")
                continue

            # === Use ObjectDetector to get formatted JSON results ===
            detection_json = self.detector.get_object_center(frame, z=self.fixed_z)
            print(f"Detection JSON: {detection_json}")

            # === Publish result ===
            msg = String()
            msg.data = detection_json

            self.publisher_.publish(msg)

    def update_depth(self, new_z):
        """Update the fixed depth value for coordinate conversion"""
        self.fixed_z = new_z
        self.get_logger().info(f'Updated depth to: {new_z} cm')


def main():
    rclpy.init()
    node = ObjectDetectionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()