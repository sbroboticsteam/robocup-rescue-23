import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(String, 'video_frame', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise IOError("Cannot open camera")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode()
            self.publisher_.publish(String(data=jpg_as_text))
    
def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.cap.release()
    video_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
