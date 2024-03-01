import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time

class LaserScanPublisher(Node):

    def __init__(self):
        super().__init__('laser_scan_publisher')
        self.publisher_=self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"  
        msg.angle_min = -math.pi / 2  
        msg.angle_max = math.pi / 2  
        msg.angle_increment = math.pi / 180  
        msg.time_increment = 0.0  
        msg.scan_time = 0.1  
        msg.range_min = 0.2  
        msg.range_max = 100.0  

        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [20.0] * num_readings  
        msg.intensities = [0.0] * num_readings 

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing LaserScan...')

def main(args=None):
    rclpy.init(args=args)
    laser_scan_publisher = LaserScanPublisher()
    rclpy.spin(laser_scan_publisher)
    laser_scan_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()