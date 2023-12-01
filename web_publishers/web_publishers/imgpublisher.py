import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64

IMG_PATH = 'web_publishers/sample.jpg'

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(String, 'image_base64_topic', 10)
        self.bridge = CvBridge()
        time = 1
        self.timer = self.create_timer(time, self.timer_callback)
    
    def timer_callback(self):
        cv_image = cv2.imread(IMG_PATH)
        if cv_image is None:
            raise FileNotFoundError(f"Unable to load image from path: {IMG_PATH}")
        _, buffer = cv2.imencode('.jpg', cv_image)
        jpgAsText = base64.b64encode(buffer).decode()
        self.publisher_.publish(String(data=jpgAsText))
        self.get_logger().info('Publishing: "%s"' % IMG_PATH)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()