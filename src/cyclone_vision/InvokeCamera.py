#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import Bool
import threading
import time

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Initialize camera objects as None
        self.frontCam = None
        self.bottomCam = None
        
        # Track camera states
        self.frontCam_active = False
        self.bottomCam_active = False
        
        self.subscription1 = self.create_subscription(
            Bool, 'camera1_command_topic', self.camera1_subscriber_callback, 10)
        self.subscription2 = self.create_subscription(
            Bool, 'camera2_command_topic', self.camera2_subscriber_callback, 10)
        
        self.frontCam_lock = threading.Lock()
        self.bottomCam_lock = threading.Lock()
        
        self.get_logger().info('Camera subscriber node initialized')
    
    def camera1_subscriber_callback(self, msg):
        self.get_logger().info(f'Camera1 command received: {msg.data}')
        
        with self.frontCam_lock:
            if msg.data is True and not self.frontCam_active:
                try:
                    self.frontCam = cv2.VideoCapture(0)  # TODO replace ID
                    if self.frontCam.isOpened():
                        self.frontCam_active = True
                        self.get_logger().info('Camera1 started capturing')
                        threading.Thread(target=self.capture_camera1, daemon=True).start()
                    else:
                        self.get_logger().error('Failed to open Camera1')
                        self.frontCam.release()
                        self.frontCam = None
                except Exception as e:
                    self.get_logger().error(f'Error starting Camera1: {str(e)}')
                    
            elif msg.data is False and self.frontCam_active:
                self.frontCam_active = False
                if self.frontCam is not None:
                    self.frontCam.release()
                    self.frontCam = None
                    self.get_logger().info('Camera1 stopped and released')
    
    def camera2_subscriber_callback(self, msg):
        self.get_logger().info(f'Camera2 command received: {msg.data}')
        
        with self.bottomCam_lock:
            if msg.data is True and not self.bottomCam_active:
                try:
                    self.bottomCam = cv2.VideoCapture(1)  # TODO replace ID
                    if self.bottomCam.isOpened():
                        self.bottomCam_active = True
                        self.get_logger().info('Camera2 started capturing')
                        threading.Thread(target=self.capture_camera2, daemon=True).start()
                    else:
                        self.get_logger().error('Failed to open Camera2')
                        self.bottomCam.release()
                        self.bottomCam = None
                except Exception as e:
                    self.get_logger().error(f'Error starting Camera2: {str(e)}')
                    
            elif msg.data is False and self.bottomCam_active:
                self.bottomCam_active = False
                if self.bottomCam is not None:
                    self.bottomCam.release()
                    self.bottomCam = None
                    self.get_logger().info('Camera2 stopped and released')
    
    def capture_camera1(self):
        """Continuous capture loop for camera1"""
        while self.frontCam_active and self.frontCam is not None:
            with self.frontCam_lock:
                if not self.frontCam_active:
                    break
                    
                ret, frame = self.frontCam.read()
                if ret:
                    pass # Process frame here 
                else:
                    self.get_logger().warning('Failed to read frame from Camera1')
                    time.sleep(0.1)  # Brief pause before retry
            
            time.sleep(0.033)  # ~30 FPS
    
    def capture_camera2(self):
        """Continuous capture loop for camera2"""
        while self.bottomCam_active and self.bottomCam is not None:
            with self.bottomCam_lock:
                if not self.bottomCam_active:
                    break
                    
                ret, frame = self.bottomCam.read()
                if ret:
                    pass 
                else:
                    self.get_logger().warning('Failed to read frame from Camera2')
                    time.sleep(0.1)
            
            time.sleep(0.033)
    
    def destroy_node(self):
        """Clean up when shutting down"""
        self.frontCam_active = False
        self.bottomCam_active = False
        
        if self.frontCam is not None:
            self.frontCam.release()
        if self.bottomCam is not None:
            self.bottomCam.release()
            
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    camera_subscriber = CameraSubscriber()
    
    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        camera_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()