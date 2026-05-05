#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mocap4r2_msgs.msg import RigidBodies
import csv
from datetime import datetime


# ==========================================
# 1. Helper Functions
# ==========================================
def wrap_pi(a: float) -> float:
    """Wraps angle to [-pi, pi] for shortest-path turning."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamps value between lo and hi."""
    return max(lo, min(hi, x))


def yaw_from_quat_zup(qx, qy, qz, qw) -> float:
    """Standard Z-Up Heading (Yaw around Z-axis)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# ==========================================
# 2. Main Node
# ==========================================
class TwoAgentLeaderFollower(Node):
    def __init__(self):
        super().__init__("two_agent_leader_follower_mocap")

        # --- ROS PARAMETERS ---
        self.declare_parameter("leader_cmd_topic", "/burger1/cmd_vel")
        self.declare_parameter("follower_cmd_topic", "/burger2/cmd_vel")
        self.declare_parameter("mocap_topic", "/rigid_bodies")
        self.declare_parameter("leader_rigid_body_name", "6")
        self.declare_parameter("follower_rigid_body_name", "7")

        # --- CONTROL GAINS ---
        self.declare_parameter("rho0", 0.2)       # 0.2 min
        self.declare_parameter("k1", 1.5)         
        self.declare_parameter("k2", 2.0)         
        self.declare_parameter("k_rho", 20.0)     # rho gain
        self.declare_parameter("follower_side", 1)  # 0 for Left, 1 for Right
        self.declare_parameter("know_u1", 1.0)    # 1 = knows u1

        # --- LEADER TRAJECTORY ---
        self.declare_parameter("leader_v", 0.05)
        self.declare_parameter("leader_u_amp", 0.4)   # amplitude of sine wave
        self.declare_parameter("leader_u_freq", 0.5)  # how fast the turn is
        self.declare_parameter("ramp_duration", 3.0)

        # --- LIMITS ---
        self.declare_parameter("v2_max", 0.22)
        self.declare_parameter("u2_max", 2.5)

        # Load Parameters
        self.rho0 = self.get_parameter("rho0").value
        self.k1 = self.get_parameter("k1").value
        self.k2 = self.get_parameter("k2").value
        self.k_rho = self.get_parameter("k_rho").value
        self.side = self.get_parameter("follower_side").value
        self.know = self.get_parameter("know_u1").value
        self.v2_max_param = self.get_parameter("v2_max").value
        self.u2_max_param = self.get_parameter("u2_max").value

        self.target_v1 = self.get_parameter("leader_v").value
        self.u_amp = self.get_parameter("leader_u_amp").value
        self.u_freq = self.get_parameter("leader_u_freq").value
        self.ramp_dur = self.get_parameter("ramp_duration").value

        self.leader_name = self.get_parameter("leader_rigid_body_name").value
        self.follower_name = self.get_parameter("follower_rigid_body_name").value

        # Pubs/Subs
        self.pub_1 = self.create_publisher(
            Twist, self.get_parameter("leader_cmd_topic").value, 10
        )
        self.pub_2 = self.create_publisher(
            Twist, self.get_parameter("follower_cmd_topic").value, 10
        )
        self.sub = self.create_subscription(
            RigidBodies, self.get_parameter("mocap_topic").value, self.on_mocap, 10
        )

        self.pose = {}
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(0.033, self.on_timer)  # ~30Hz

        # --- CSV RECORDING ---
        filename = f"robot_data_{datetime.now().strftime('%H%M%S')}.csv"
        self.f = open(filename, "w", newline="")
        self.writer = csv.writer(self.f)
        self.writer.writerow(
            ["time", "x1", "y1", "th1", "x2", "y2", "th2", "v1", "u1", "v2", "u2", "a2_err"]
        )

        self.get_logger().info(f"Node Started. Side: {'Left' if self.side == 0 else 'Right'}")

    def on_mocap(self, msg: RigidBodies):
        t = self.get_clock().now().nanoseconds * 1e-9
        for rb in msg.rigidbodies:
            x = rb.pose.position.x
            y = rb.pose.position.y
            theta = yaw_from_quat_zup(
                rb.pose.orientation.x,
                rb.pose.orientation.y,
                rb.pose.orientation.z,
                rb.pose.orientation.w,
            )
            self.pose[rb.rigid_body_name] = (x, y, theta, t)

    def on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.start_time

        # 1. LEADER TRAJECTORY (u1 varies with time)
        v1 = self.target_v1
        u1 = 0.2 + self.u_amp * math.sin(math.pi * self.u_freq * elapsed)

        # Apply Smooth Ramp
        scale = min(elapsed / self.ramp_dur, 1.0)
        v1_cmd, u1_cmd = v1 * scale, u1 * scale

        cmd1 = Twist()
        cmd1.linear.x = v1_cmd
        cmd1.angular.z = u1_cmd
        self.pub_1.publish(cmd1)

        # 2. CHECK DATA
        if self.leader_name not in self.pose or self.follower_name not in self.pose:
            return

        x1, y1, th1, _ = self.pose[self.leader_name]
        x2, y2, th2, _ = self.pose[self.follower_name]

        # 3. SHAPE VARIABLES
        dx, dy = x2 - x1, y2 - y1
        rho = max(math.sqrt(dx**2 + dy**2), 0.1)
        phi12, phi21 = math.atan2(dy, dx), math.atan2(-dy, -dx)
        alpha1, alpha2 = wrap_pi(phi12 - th1), wrap_pi(phi21 - th2)

        # SETPOINT CALCULATION
        p_left = 1.0 if self.side == 0 else -1.0
        alpha1_d = (math.pi / 2.0) * p_left
        alpha2_d = wrap_pi(alpha1_d + math.pi * p_left)

        # ERROR WRAPPING (Shortest path fix)
        alpha1_err = wrap_pi(alpha1 - alpha1_d)
        alpha2_err = wrap_pi(alpha2 - alpha2_d)

        # 4. CONTROL LAWS
        f_rho = self.k_rho * (rho**2 - self.rho0**2) / (rho**2)

        # Linear Velocity Law
        term_bearing = (
            v1 * math.sin(alpha1)
            - rho * u1 * self.know
            + rho * self.k1 * math.sin(alpha1_err)
        )

        v2_raw = p_left * term_bearing + (rho * v1 * f_rho)

        # HEADING GOVERNOR: Scale v2 by how well we are pointed
        # This prevents wide looping turns
        cos_err = math.cos(alpha2_err)
        v2 = v2_raw * max(0.0, cos_err)

        # Angular Velocity Law
        u2 = (
            (v1 * math.sin(alpha1) + v2 * math.sin(alpha2)) / rho
            + self.k2 * math.sin(alpha2_err)
        )

        # 5. SAFETY & PUBLISH
        v2 = clamp(v2, 0.0, self.v2_max_param)
        u2 = clamp(u2, -self.u2_max_param, self.u2_max_param)

        cmd2 = Twist()
        cmd2.linear.x = v2
        cmd2.angular.z = u2
        self.pub_2.publish(cmd2)

        # LOGGING
        self.writer.writerow(
            [
                f"{elapsed:.3f}",
                f"{x1:.3f}",
                f"{y1:.3f}",
                f"{th1:.3f}",
                f"{x2:.3f}",
                f"{y2:.3f}",
                f"{th2:.3f}",
                f"{v1_cmd:.3f}",
                f"{u1_cmd:.3f}",
                f"{v2:.3f}",
                f"{u2:.3f}",
                f"{alpha1:.3f}",
                f"{alpha2:.3f}",
            ]
        )

    def shutdown(self):
        stop = Twist()
        for _ in range(5):
            self.pub_1.publish(stop)
            self.pub_2.publish(stop)
            time.sleep(0.1)

        if hasattr(self, "f"):
            self.f.close()

        self.get_logger().info("done.")


def main():
    rclpy.init()
    node = TwoAgentLeaderFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
