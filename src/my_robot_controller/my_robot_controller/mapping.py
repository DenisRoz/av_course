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

        front = region_min(0.0, window_deg=30)
        left = region_min(90.0, window_deg=40)
        right = region_min(270.0, window_deg=40)

        # Wall-following parameters
        desired_wall_dist = 0.65
        obstacle_threshold = 0.6
        wall_search_threshold = 1.5  # Distance to consider "no wall nearby"

        # Find closest wall if none nearby
        if left > wall_search_threshold and right > wall_search_threshold:
            # No walls nearby: turn towards the closest one
            twist.linear.x = 0.1
            if left < right:
                twist.angular.z = 0.3  # Turn left towards closer left wall
            else:
                twist.angular.z = -0.3  # Turn right towards closer right wall
        else:
            # Wall-hugging logic (follow right wall)
            if front <= obstacle_threshold:
                # Obstacle ahead: turn left
                twist.linear.x = 0.0
                twist.angular.z = 0.6
            elif right > desired_wall_dist + 0.2:
                # Too far from right wall: turn right to get closer
                twist.linear.x = 0.15
                twist.angular.z = -0.3
            else:
                # Close enough to wall: go straight
                twist.linear.x = 0.25
                twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

def main(args=None): 
    rclpy.init(args=args) 
    mapping_node = TurtlebotMappingNode() 
    rclpy.spin(mapping_node) 
    mapping_node.destroy_node() 
    rclpy.shutdown()
