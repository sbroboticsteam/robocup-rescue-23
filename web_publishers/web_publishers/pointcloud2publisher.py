import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

# let's publish some LiDAR SLAM data!
class PointCloud2Publisher(Node):
    def __init__(self):
        super().__init__('pointcloud2_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        points = np.random.rand(10, 3) * 10

        header = self.create_msg_header()
        cloud = pc2.create_cloud_xyz32(header, points)
        self.publisher_.publish(cloud)
        self.get_logger().info('Publishing PointCloud2...')

    def create_msg_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        return header
    
def main(args=None):
    rclpy.init(args=args)
    pointcloud2_publisher = PointCloud2Publisher()
    rclpy.spin(pointcloud2_publisher)
    pointcloud2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()
