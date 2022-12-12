import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SelfDrive(Node):
    def __init__(self):
        super().__init__('self_drive')
        lidar_qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                       history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                       depth=1)
        vel_qos_profile = QoSProfile(depth=10)
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.subscribe_scan,
            lidar_qos_profile)
        self.pub_velo = self.create_publisher(Twist, '/cmd_vel', vel_qos_profile)
        self.step = 0
        self.module = 'start'

    def subscribe_scan(self, scan):
        twist = Twist()
        if self.module == 'start':
            twist.linear.x = 0.15
            twist.angular.z = 0.
            if scan.ranges[0] < 0.20 and scan.ranges[0] != 0:
                self.module = 'turn'

        if self.module == 'turn':
            twist.linear.x = 0.
            twist.angular.z = 0.4
            if scan.ranges[315]*0.9 < scan.ranges[270]*1.414 < scan.ranges[315]*1.1 and scan.ranges[0] > 0.25:
                self.module = 'wall_follow'

        if self.module == 'wall_follow':
            twist.linear.x = 0.15
            twist.angular.z = 0.

            if scan.ranges[305] < scan.ranges[235]:
                twist.linear.x = 0.15
                twist.angular.z = 0.4

            elif scan.ranges[235] < scan.ranges[305]:
                twist.linear.x = 0.15
                twist.angular.z = -0.4

            if scan.ranges[270] < 0.13:
                twist.linear.x = 0.15
                twist.angular.z = 0.2

            if scan.ranges[0] < 0.30 and scan.ranges[0] != 0:
                self.module = 'start'

            if scan.ranges[305] > scan.ranges[235] * 2:
                self.module = 'out_conner'

        self.pub_velo.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SelfDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
