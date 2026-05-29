#!/usr/bin/env python3

"""
Improved TurtleBot LiDAR exploration controller.

This version keeps the original idea of the submitted turtlebot.py:
    escape -> find -> follow
with a committed recovery manoeuvre.

Improvements:
    1. Safer LiDAR sector handling with wrap-around.
    2. Slightly more conservative speeds and distances.
    3. More stable committed recovery actions to reduce shaking.
    4. Better corner handling with short reverse + turn.
    5. Clearer constants and comments for report/explanation.
"""

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

import numpy as np


class LidarController(Node):

    def __init__(self):
        super().__init__("lidar_controller")

        self.lidar_sub = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,
        )

        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # Main behaviour state: escape -> find -> follow
        self.state = "escape"
        self.state_start = self.get_clock().now()

        # Recovery state is used to commit to a motion for a fixed duration.
        # This prevents the robot from changing its decision every LiDAR frame.
        self.in_recovery = False
        self.recovery_end = self.get_clock().now()
        self.recovery_lin = 0.0
        self.recovery_ang = 0.0

        self.get_logger().info(f"Node '{self.get_name()}' initialised.")

    def lidar_callback(self, scan_data: LaserScan):
        """Main control loop, called whenever a new LiDAR scan is received."""

        ranges = np.array(scan_data.ranges, dtype=float)
        n = len(ranges)

        def sector_values(center_deg: int, half_deg: int):
            """Return valid LiDAR values inside a wrapped angular sector."""
            idxs = np.arange(center_deg - half_deg, center_deg + half_deg + 1, dtype=int) % n
            vals = ranges[idxs]
            valid = vals[(vals > 0.05) & np.isfinite(vals)]
            return valid

        def sector_min(center_deg: int, half_deg: int) -> float:
            valid = sector_values(center_deg, half_deg)
            return float(valid.min()) if valid.size > 0 else 10.0

        def sector_mean(center_deg: int, half_deg: int) -> float:
            valid = sector_values(center_deg, half_deg)
            return float(valid.mean()) if valid.size > 0 else 10.0

        # TurtleBot LiDAR convention used here:
        # 0 deg = front, 90 deg = left, 270/315 deg = right/front-right.
        front = sector_min(0, 20)
        front_left = sector_min(45, 22)
        front_right = sector_min(315, 22)
        left = sector_mean(90, 14)
        right = sector_mean(270, 14)

        self.get_logger().info(
            f"\nLiDAR Readings:\n"
            f"  Front: {front:.3f} meters\n"
            f"  Left: {left:.3f} meters\n"
            f"  Right: {right:.3f} meters\n",
            throttle_duration_sec = 1,
        ) 

        # -----------------------------
        # Tunable parameters
        # -----------------------------
        LIN_FAST = 0.24       # slightly reduced from 0.30 for safer real-robot movement
        LIN_SLOW = 0.14
        REVERSE = -0.07

        ANG_FAST = 0.62
        ANG_MED = 0.47
        ANG_SLOW = 0.30

        DANGER = 0.30         # emergency threshold
        CLOSE = 0.48          # normal obstacle threshold
        WALL_D = 0.45         # desired left wall distance
        WALL_TOL = 0.10
        WALL_FAR = WALL_D + 0.30

        now = self.get_clock().now()
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_link"

        # -----------------------------
        # 1. Continue recovery if active
        # -----------------------------
        if self.in_recovery:
            if now < self.recovery_end:
                msg.twist.linear.x = self.recovery_lin
                msg.twist.angular.z = self.recovery_ang
                self.cmd_vel_pub.publish(msg)
                return
            self.in_recovery = False

        # -----------------------------
        # 2. Emergency corner / collision recovery
        # -----------------------------
        # If front and both front sides are close, the robot is probably in a corner.
        corner_trap = front < CLOSE and front_left < CLOSE and front_right < CLOSE

        if corner_trap:
            # Reverse while turning towards the more open side.
            ang = ANG_FAST if left > right else -ANG_FAST
            self._start_recovery(lin=REVERSE, ang=ang, duration=1.2)
            msg.twist.linear.x = REVERSE
            msg.twist.angular.z = ang
            self.cmd_vel_pub.publish(msg)
            return

        if min(front, front_left, front_right) < DANGER:
            # Emergency action: back up and turn away from the closer side.
            ang = -ANG_FAST if front_left < front_right else ANG_FAST
            self._start_recovery(lin=REVERSE, ang=ang, duration=1.0)
            msg.twist.linear.x = REVERSE
            msg.twist.angular.z = ang
            self.cmd_vel_pub.publish(msg)
            return

        # -----------------------------
        # 3. Main finite-state machine
        # -----------------------------
        if self.state == "escape":
            elapsed = (now - self.state_start).nanoseconds / 1e9

            if front < CLOSE:
                self.get_logger().info("ESCAPE -> FOLLOW: obstacle encountered")
                self.state = "follow"
                self.state_start = now
            elif elapsed >= 3.5:
                self.get_logger().info("ESCAPE -> FIND: time elapsed")
                self.state = "find"
                self.state_start = now
            else:
                msg.twist.linear.x = LIN_FAST
                msg.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                return

        if self.state == "find":
            # Move forward until a nearby wall or object is found.
            if front < CLOSE or left < WALL_D + 0.35:
                self.get_logger().info("FIND -> FOLLOW: wall detected")
                self.state = "follow"
                self.state_start = now
            else:
                msg.twist.linear.x = LIN_FAST
                msg.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(msg)
                return

        if self.state == "follow":
            # Front obstacle: stop forward motion and commit to a right turn.
            if front < CLOSE:
                duration = 0.9 + max(0.0, CLOSE - front) * 3.0
                self._start_recovery(lin=0.0, ang=-ANG_MED, duration=duration)
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = -ANG_MED
                self.cmd_vel_pub.publish(msg)
                return

            # Avoid front-side obstacles smoothly.
            if front_left < CLOSE:
                msg.twist.linear.x = LIN_SLOW
                msg.twist.angular.z = -ANG_MED
                self.cmd_vel_pub.publish(msg)
                return

            if front_right < CLOSE:
                msg.twist.linear.x = LIN_SLOW
                msg.twist.angular.z = ANG_MED
                self.cmd_vel_pub.publish(msg)
                return

            # Left wall following with a small dead-band.
            if left < WALL_D - WALL_TOL:
                # Too close to the left wall: steer right.
                msg.twist.linear.x = LIN_SLOW
                msg.twist.angular.z = -ANG_SLOW
            elif left > WALL_FAR:
                # Lost the wall: turn left gently to reacquire it.
                msg.twist.linear.x = LIN_SLOW
                msg.twist.angular.z = ANG_SLOW
            elif left > WALL_D + WALL_TOL:
                # Slightly too far from wall: steer left.
                msg.twist.linear.x = LIN_SLOW
                msg.twist.angular.z = ANG_SLOW
            else:
                # Good corridor/wall distance: move forward.
                msg.twist.linear.x = LIN_FAST
                msg.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(msg)

    def _start_recovery(self, lin: float, ang: float, duration: float):
        """Commit to a recovery manoeuvre for a fixed duration."""
        self.in_recovery = True
        self.recovery_lin = lin
        self.recovery_ang = ang
        self.recovery_end = self.get_clock().now() + rclpy.duration.Duration(seconds=duration)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LidarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown request (Ctrl+C) detected...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

