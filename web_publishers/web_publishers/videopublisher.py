import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import base64

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'video_frame', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1 / 30.0, self.timer_callback) # 30hz video

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise IOError("Cannot open camera")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                return
            
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(buffer).tobytes()
            self.publisher_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.cap.release()
    video_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
