import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import random

class DiagnosticPublisher(Node):
    def __init__(self):
        super().__init__('diagnostic_publisher')
        self.publisher_ = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.level = random.choice([DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR])
        status.name = "robot_diagnostic"
        status.message = "Diagnostic message"
        status.hardware_id = "SBRT"

        status.values = [
            KeyValue(key="Battery", value=str(random.uniform(0, 100))),
            KeyValue(key="Voltage", value=str(random.uniform(20, 30))),
            KeyValue(key="Temperature", value=str(random.uniform(20, 40)))
        ]

        msg.status.append(status)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing diagnostic data...')

def main(args=None):
    rclpy.init(args=args)
    diagnostic_publisher = DiagnosticPublisher()
    rclpy.spin(diagnostic_publisher)
    diagnostic_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()