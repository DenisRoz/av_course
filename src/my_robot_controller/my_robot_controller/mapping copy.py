import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class TurtlebotMappingNode(Node): 
    def __init__(self): 
        super().__init__('mapping_node') 
        self.laser_subscription = self.create_subscription(LaserScan, '/scan', self.robot_controller, 10) 
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10) 
        self.get_logger().info('Turtlebot Mapping Node has been started.') 


    def robot_controller(self, scan: LaserScan): 
        twist = Twist()

        def is_valid_range(value):
            return (value is not None
                    and not (value != value)  # NaN check
                    and value != float('inf')
                    and value != float('-inf')
                    and scan.range_min <= value <= scan.range_max)

        def region_min(ideal_angle_deg, window_deg=20):
            # Calculate indices from angles in scan message
            angle_count = len(scan.ranges)
            if angle_count == 0 or scan.angle_increment == 0:
                return scan.range_max if scan.range_max > 0 else 0.0

            center_idx = int((ideal_angle_deg - scan.angle_min) / scan.angle_increment)
            half_step = int((window_deg / 2.0) / (scan.angle_increment * 180.0 / 3.141592653589793))
            candidates = []
            for offset in range(-half_step, half_step + 1):
                idx = (center_idx + offset) % angle_count
                dist = scan.ranges[idx]
                if is_valid_range(dist):
                    candidates.append(dist)
            return min(candidates) if candidates else scan.range_max

        front = region_min(0.0, window_deg=20)
        left = region_min(90.0, window_deg=30)
        right = region_min(270.0, window_deg=30)

        # Smoother, adaptive obstacle avoidance behavior
        slow_down_threshold = 1.0
        stop_threshold = 0.45

        if front <= stop_threshold:
            twist.linear.x = 0.0
            twist.angular.z = 0.6 if right < left else -0.6
        elif front <= slow_down_threshold:
            twist.linear.x = 0.10
            twist.angular.z = 0.4 if right < left else -0.4
        else:
            twist.linear.x = min(0.4, 0.8 * (front / scan.range_max)) if scan.range_max > 0 else 0.2
            twist.angular.z = 0.0

        # Always keep minimal turning correction from sidedists to avoid deadlocks
        if abs(left - right) > 0.15 and front > 0.3:
            twist.angular.z += 0.1 if right < left else -0.1

        self.cmd_vel_publisher.publish(twist)

def main(args=None): 
    rclpy.init(args=args) 
    mapping_node = TurtlebotMappingNode() 
    rclpy.spin(mapping_node) 
    mapping_node.destroy_node() 
    rclpy.shutdown()
