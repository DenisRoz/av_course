#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import math
import random
import time

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.get_logger().info("Controller has been started.")
        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )
        # Subscriber for turtle's position
        self._pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )

    def pose_callback(self, pose: Pose):
        # -------------------- constants you can tweak --------------------
        # Max commanded speeds
        MAX_LINEAR = 2.2    # maximum forward speed (turtlesim units)
        MAX_ANGULAR = 2.5   # maximum angular speed (rad/s)

        # Acceleration limits (used for smoothing)
        MAX_LINEAR_ACC = 1.5   # units per second^2
        MAX_ANGULAR_ACC = 4.0  # rad per second^2

        # Controller gains / behavior
        KP_LIN = 1.05
        KP_ANG = 2.8

        # Approach behavior
        GOAL_REACH_TOL = 0.18    # when reached
        SLOW_START_DIST = 1.0    # start gentle slow-down inside this distance
        MIN_APPROACH_SPEED = 0.18  # do not go slower than this when still approaching

        # Wandering target parameters (same as before)
        TARGET_TIMEOUT = 6.0
        MIN_STEP = 0.8
        MAX_STEP = 3.0
        YAW_ALIGN_THRESHOLD = 0.45  # rad; if yaw error > this, rotate in place
        SAFE_MIN = 1.0
        SAFE_MAX = 9.0
        # ----------------------------------------------------------------

        # --- lazy state initialization (keeps __init__ unchanged) ---
        if not hasattr(self, "_wander_target"):
            self._wander_target = None
            self._target_set_time = 0.0
            self._target_timeout = TARGET_TIMEOUT
            self._min_step = MIN_STEP
            self._max_step = MAX_STEP
            self._goal_reach_tol = GOAL_REACH_TOL
            self._kp_lin = KP_LIN
            self._kp_ang = KP_ANG
            self._max_lin = MAX_LINEAR
            self._max_ang = MAX_ANGULAR
            # smoothing state
            self._last_lin_cmd = 0.0
            self._last_ang_cmd = 0.0
            self._last_cmd_time = time.time()
            # approach tuning
            self._slow_start_dist = SLOW_START_DIST
            self._min_approach_speed = MIN_APPROACH_SPEED
            self._yaw_align_thresh = YAW_ALIGN_THRESHOLD

        # helper local functions
        def angle_normalize(a: float) -> float:
            while a > math.pi:
                a -= 2.0 * math.pi
            while a < -math.pi:
                a += 2.0 * math.pi
            return a

        def clamp(v: float, lo: float, hi: float) -> float:
            return max(lo, min(hi, v))

        # --- pick or update wandering target as before ---
        now = time.time()
        target = self._wander_target
        reached = False
        if target is not None:
            tx, ty = target
            dx = tx - pose.x
            dy = ty - pose.y
            dist_to_target = math.hypot(dx, dy)
            if dist_to_target <= self._goal_reach_tol:
                reached = True
        else:
            dist_to_target = float("inf")

        if (target is None) or reached or (now - self._target_set_time > self._target_timeout):
            jitter = random.uniform(-math.pi/4.0, math.pi/4.0)
            step = random.uniform(self._min_step, self._max_step)
            angle_ahead = pose.theta + jitter
            cand_x = pose.x + math.cos(angle_ahead) * step
            cand_y = pose.y + math.sin(angle_ahead) * step
            # clamp to safe box
            cand_x = clamp(cand_x, SAFE_MIN + 0.15, SAFE_MAX - 0.15)
            cand_y = clamp(cand_y, SAFE_MIN + 0.15, SAFE_MAX - 0.15)
            self._wander_target = (cand_x, cand_y)
            self._target_set_time = now
            tx, ty = self._wander_target
            dx = tx - pose.x
            dy = ty - pose.y
            dist_to_target = math.hypot(dx, dy)

        # Compute heading error toward target
        target_angle = math.atan2(dy, dx)
        yaw_err = angle_normalize(target_angle - pose.theta)

        # Basic steering: proportional angular command
        desired_ang = clamp(self._kp_ang * yaw_err, -self._max_ang, self._max_ang)

        # Forward command: proportional to distance, but avoid collapsing to near-zero too early
        # If yaw error large, prefer rotating in place (no forward)
        if abs(yaw_err) > self._yaw_align_thresh:
            desired_lin = 0.0
        else:
            # scale based on distance-to-target with a gentle taper
            if dist_to_target >= self._slow_start_dist:
                # plenty of room: full proportional speed
                desired_lin = clamp(self._kp_lin * dist_to_target, 0.0, self._max_lin)
            else:
                # inside slow-start distance => use a gentler approach curve that does NOT go to 0 immediately
                # map dist_to_target in [0.._slow_start_dist] to speed in [MIN_APPROACH_SPEED .. max_lin]
                frac = clamp(dist_to_target / max(1e-6, self._slow_start_dist), 0.0, 1.0)
                # use an eased curve (sqrt) so it decays slower near the end
                eased = math.sqrt(frac)
                desired_lin = self._min_approach_speed + (self._max_lin - self._min_approach_speed) * eased
                # if very very near the goal, slow further but don't go to almost-zero until within reach tol
                if dist_to_target <= self._goal_reach_tol:
                    desired_lin = 0.0

            # reduce forward when near walls (prevent slam into wall)
            dist_left = pose.x - SAFE_MIN
            dist_right = SAFE_MAX - pose.x
            dist_bottom = pose.y - SAFE_MIN
            dist_top = SAFE_MAX - pose.y
            min_dist_to_wall = min(dist_left, dist_right, dist_bottom, dist_top)
            # when within ~1.0 unit of wall, start scaling down; but keep a floor so it doesn't stop
            wall_scale = clamp(min_dist_to_wall / 1.0, 0.35, 1.0)
            desired_lin *= wall_scale

        # Safety: if outside safe area, steer to center and keep slow forward
        if not (SAFE_MIN <= pose.x <= SAFE_MAX and SAFE_MIN <= pose.y <= SAFE_MAX):
            center_x = (SAFE_MIN + SAFE_MAX) / 2.0
            center_y = (SAFE_MIN + SAFE_MAX) / 2.0
            angle_to_center = math.atan2(center_y - pose.y, center_x - pose.x)
            yaw_err_center = angle_normalize(angle_to_center - pose.theta)
            desired_ang = clamp(self._kp_ang * yaw_err_center, -self._max_ang, self._max_ang)
            desired_lin = min(desired_lin, 0.25)

        # ------------------ smoothing / ramping ------------------
        t_now = time.time()
        dt = max(1e-4, t_now - self._last_cmd_time)

        # linear ramp
        lin_diff = desired_lin - self._last_lin_cmd
        max_lin_step = MAX_LINEAR_ACC * dt if 'MAX_LINEAR_ACC' in globals() else MAX_LINEAR_ACC
        lin_step = clamp(lin_diff, -MAX_LINEAR_ACC * dt, MAX_LINEAR_ACC * dt)
        new_lin = self._last_lin_cmd + lin_step

        # angular ramp
        ang_diff = desired_ang - self._last_ang_cmd
        ang_step = clamp(ang_diff, -MAX_ANGULAR_ACC * dt, MAX_ANGULAR_ACC * dt)
        new_ang = self._last_ang_cmd + ang_step

        # enforce final clamping to absolute maxima
        new_lin = clamp(new_lin, -self._max_lin, self._max_lin)
        new_ang = clamp(new_ang, -self._max_ang, self._max_ang)

        # publish smoothed command
        cmd = Twist()
        cmd.linear.x = float(new_lin)
        cmd.angular.z = float(new_ang)
        self.cmd_vel_publisher.publish(cmd)

        # save state
        self._last_lin_cmd = new_lin
        self._last_ang_cmd = new_ang
        self._last_cmd_time = t_now

    # end pose_callback

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop the turtle before shutdown
        stop_twist = Twist()
        node.cmd_vel_publisher.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()