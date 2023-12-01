import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import string

class RandomStringPublisher(Node):

    def __init__(self):
        super().__init__('random_string_publisher')
        self.publisher_ = self.create_publisher(String, 'random_string_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        random_string = ''.join(random.choices(string.ascii_uppercase + string.digits, k=10))
        self.publisher_.publish(String(data=random_string))
        self.get_logger().info('Publishing: "%s"' % random_string)

def main(args=None):
    rclpy.init(args=args)
    random_string_publisher = RandomStringPublisher()
    rclpy.spin(random_string_publisher)
    random_string_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()