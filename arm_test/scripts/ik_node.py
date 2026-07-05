#!/usr/bin/env python3
"""
IK solver for LeArm: converts target (x, y, z) to joint angles,
publishes to /joint_target for arm_base to send to STM32 via CMD_MULT_SERVO_MOVE.

Theory: Analytical geometric IK via subproblem decomposition.
  - 4-DOF planar (base rotation + 3 links) for 3-DOF target → 1 redundant DOF (pitch)
  - Base rotation θ1 = atan2(y, x), then solve 2D planar problem in (r, z) plane
  - Subtract L4 contribution to get wrist position, solve 2-link IK via law of cosines
  - Pitch α is auto-selected from the valid range (no manual tuning needed)

DH params from STM32 firmware kinematics.h:
  L1 = 2.89, L2 = 10.43, L3 = 8.9, L4 = 17.7 (cm)

Joint→Servo mapping from theta2servo() in robot_arm.c (SERVO_TYPE=2):
  servo 6: target_angle = knot[0]°            → pulse = 500 + 4.1667 * angle
  servo 5: target_angle = 90° - knot[1]°      → pulse = 500 + 4.1667 * angle
  servo 4: target_angle = knot[2]°            → pulse = 500 + 4.1667 * angle
  servo 3: target_angle = knot[3]°            → pulse = 500 + 4.1667 * angle
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from arm_msg.msg import Arm, ArmStatus

# ── DH parameters (cm, from STM32 firmware kinematics.h) ──────────────
L1 = 2.89   # base vertical offset (shoulder height)
L2 = 10.43  # upper arm length
L3 = 8.9    # forearm length
L4 = 17.7   # wrist-to-gripper-tip length

# ── Joint limits (degrees, from kinematics.h) ────────────────────────
LIMITS_DEG = {
    'shoulder_pan':  (-90.0, 90.0),
    'shoulder_lift': (0.0, 180.0),
    'elbow':         (-90.0, 90.0),
    'wrist_flex':    (-90.0, 90.0),
    'wrist_roll':    (-90.0, 90.0),
}

# ── Servo mapping ─────────────────────────────────────────────────────
SERVO_FACTOR = 4.166666666666667   # 1000 pulse / 240 deg
SERVO_CENTER = 500

# Joint name ordering for /joint_target
JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow',
               'wrist_flex', 'wrist_roll', 'grip_left']


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


# ═══════════════════════════════════════════════════════════════════════
# Core IK: 2-link solver
# ═══════════════════════════════════════════════════════════════════════

def solve_2link(r_w, h_w, L_a, L_b):
    """Solve 2-link planar IK (shoulder → elbow → wrist).

    Args:
        r_w, h_w: wrist position relative to shoulder (horizontal, vertical)
        L_a, L_b: link lengths (upper arm, forearm)

    Returns:
        List of [(θ_a, θ_b), ...] solutions, or None if unreachable.
        θ_a = absolute angle of link A from horizontal (X-axis)
        θ_b = relative angle of link B from link A
        Both in radians.
    """
    c_sq = r_w * r_w + h_w * h_w
    c = math.sqrt(c_sq)

    if c > L_a + L_b or c < abs(L_a - L_b):
        return None

    cos_b = (c_sq - L_a * L_a - L_b * L_b) / (2.0 * L_a * L_b)
    cos_b = clamp(cos_b, -1.0, 1.0)

    b_down = -math.acos(cos_b)   # elbow-down: θ_b < 0
    b_up   =  math.acos(cos_b)   # elbow-up:   θ_b > 0

    solutions = []
    for b in [b_down, b_up]:
        a = math.atan2(h_w, r_w) - math.atan2(L_b * math.sin(b),
                                               L_a + L_b * math.cos(b))
        solutions.append((a, b))
    return solutions


def check_joint_limits(theta2_deg, theta3_deg, theta4_deg):
    """Return True if all joint angles are within hardware limits."""
    if not (LIMITS_DEG['shoulder_lift'][0] <= theta2_deg <= LIMITS_DEG['shoulder_lift'][1]):
        return False
    if not (LIMITS_DEG['elbow'][0] <= theta3_deg <= LIMITS_DEG['elbow'][1]):
        return False
    if not (LIMITS_DEG['wrist_flex'][0] <= theta4_deg <= LIMITS_DEG['wrist_flex'][1]):
        return False
    return True


# ═══════════════════════════════════════════════════════════════════════
# Pitch auto-selection
# ═══════════════════════════════════════════════════════════════════════

def find_valid_pitch_range(r, h, pitch_step=1.0):
    """Scan pitch space [-90°, 0°] to find all valid pitch values.

    The pitch (α) is the angle of L4 (end effector) from horizontal.
    Negative = pointing downward (typical for grasping from above).

    Returns:
        List of valid pitch values (degrees), sorted.
    """
    valid = []
    for pitch_deg in [i * pitch_step for i in range(-90, 1)]:
        if abs(pitch_deg) < 0.001:
            pitch_deg = 0.0
        pitch_rad = math.radians(pitch_deg)

        r_w = r - L4 * math.cos(pitch_rad)
        h_w = h - L1 - L4 * math.sin(pitch_rad)

        sols = solve_2link(r_w, h_w, L2, L3)
        if sols is None:
            continue

        for t2, t3 in sols:
            t4 = pitch_rad - t2 - t3
            if check_joint_limits(math.degrees(t2), math.degrees(t3),
                                  math.degrees(t4)):
                valid.append(pitch_deg)
                break

    return valid


def select_pitch(r, h, preferred_pitch_deg=-50.0):
    """Auto-select the best pitch for a given target.

    Strategy:
      1. Compute the full valid pitch range for this target.
      2. If the preferred pitch is valid, use it.
      3. Otherwise, pick the valid pitch closest to the preferred one.

    Returns:
        Optimal pitch angle in degrees, or None if target is unreachable.
    """
    valid_pitches = find_valid_pitch_range(r, h, pitch_step=1.0)
    if not valid_pitches:
        return None
    return min(valid_pitches, key=lambda p: abs(p - preferred_pitch_deg))


# ═══════════════════════════════════════════════════════════════════════
# Full IK pipeline
# ═══════════════════════════════════════════════════════════════════════

def ik_solve(x, y, z, preferred_pitch_deg=-50.0):
    """Solve inverse kinematics for the LeArm.

    Args:
        x, y, z: target end-effector position (cm).
        preferred_pitch_deg: desired pitch angle (degrees from horizontal).
            -90° = gripper pointing straight down (ideal for grasping).
            -50° = natural resting posture (good for general movement).
            Auto-adjusted if unreachable for this target.

    Returns:
        (theta1, theta2, theta3, theta4, theta5, pitch_deg) in radians
        (except pitch_deg which is in degrees), or None if unreachable.
    """
    # 1. Base rotation
    theta1 = math.atan2(y, x)

    # 2. Project to radial plane
    r = math.sqrt(x * x + y * y)
    h = z

    # 3. Auto-select pitch
    pitch_deg = select_pitch(r, h, preferred_pitch_deg)
    if pitch_deg is None:
        return None
    pitch_rad = math.radians(pitch_deg)

    # 4. Compute wrist position (remove L4 contribution)
    r_w = r - L4 * math.cos(pitch_rad)
    h_w = h - L1 - L4 * math.sin(pitch_rad)

    # 5. 2-link IK from shoulder to wrist
    sols = solve_2link(r_w, h_w, L2, L3)
    if sols is None:
        return None

    # 6. Choose best solution (prefer elbow-down, stay away from limits)
    best, best_cost = None, float('inf')
    for t2, t3 in sols:
        t4 = pitch_rad - t2 - t3
        t2d, t3d, t4d = math.degrees(t2), math.degrees(t3), math.degrees(t4)
        if not check_joint_limits(t2d, t3d, t4d):
            continue
        cost = abs(t3d + 45.0)  # prefer elbow near -45° (mid-range)
        if cost < best_cost:
            best_cost = cost
            best = (t2, t3, t4)

    if best is None:
        return None

    theta2, theta3, theta4 = best
    theta5 = 0.0
    return theta1, theta2, theta3, theta4, theta5, pitch_deg


def angles_to_servo_pulses(theta1_deg, theta2_deg, theta3_deg,
                           theta4_deg, theta5_deg, grip_deg=0.0):
    """Convert IK joint angles (degrees) to servo pulses [0, 1000]."""
    f = SERVO_FACTOR
    pulses = [0] * 6
    pulses[5] = int(SERVO_CENTER + f * theta1_deg)                       # servo 6: base
    pulses[4] = int(SERVO_CENTER + f * (90.0 - theta2_deg))              # servo 5: shoulder
    pulses[3] = int(SERVO_CENTER + f * theta3_deg)                       # servo 4: elbow
    pulses[2] = int(SERVO_CENTER + f * theta4_deg)                       # servo 3: wrist flex
    pulses[1] = int(SERVO_CENTER + f * theta5_deg)                       # servo 2: wrist roll
    pulses[0] = clamp(int(700.0 - 5.5556 * grip_deg), 200, 700)          # servo 1: gripper
    for i in range(6):
        pulses[i] = clamp(pulses[i], 0, 1000)
    return pulses


# ═══════════════════════════════════════════════════════════════════════
# ROS2 Node
# ═══════════════════════════════════════════════════════════════════════

class IKSolverNode(Node):
    """IK solver: subscribes /arm_cmd (absolute x,y,z),
    publishes /joint_target (JointState)."""

    def __init__(self):
        super().__init__('ik_solver')

        self.declare_parameter('preferred_pitch', -50.0)
        self.declare_parameter('gripper_angle', 90.0)

        self.pref_pitch = self.get_parameter('preferred_pitch').value
        # Default gripper angle (arm_base ignores grip_left from /joint_target
        # and uses /grip_cmd independently, so this value doesn't affect
        # the physical gripper — it's only for logging/monitoring.)
        self._gripper_angle_rad = math.radians(
            self.get_parameter('gripper_angle').value)

        self.sub = self.create_subscription(
            Arm, 'arm_cmd', self._cmd_callback, 10)
        self.joint_pub = self.create_publisher(
            JointState, 'joint_target', 10)
        # Publish arm_status for compatibility with calib_collect.py
        self.status_pub = self.create_publisher(
            ArmStatus, 'arm_status',
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))
        # Gripper is now controlled exclusively via /grip_cmd → arm_base.
        # ik_node no longer needs to track grip state; arm_base ignores
        # grip_left from /joint_target and uses /grip_cmd independently.

        self.get_logger().info(
            f'IK solver ready: L=[{L1},{L2},{L3},{L4}] cm, '
            f'preferred_pitch={self.pref_pitch}°')

    def _cmd_callback(self, msg: Arm):
        x, y, z = msg.x, msg.y, msg.z

        # Physical workspace limits:
        #   X: [8, 37] cm  — folded to fully extended
        #   Y: [-12, 12] cm — symmetric about base
        #   Z: [-2, 22] cm — floor to max reach
        x = clamp(x, 8.0, 37.0)
        y = clamp(y, -12.0, 12.0)
        z = clamp(z, -2.0, 22.0)

        result = ik_solve(x, y, z, self.pref_pitch)
        if result is None:
            self.get_logger().error(
                f'IK FAILED for ({x:.1f}, {y:.1f}, {z:.1f})')
            return

        t1, t2, t3, t4, t5, pitch_deg = result
        td = [math.degrees(v) for v in [t1, t2, t3, t4, t5]]

        self.get_logger().info(
            f'IK: ({x:.1f},{y:.1f},{z:.1f}) → '
            f'θ=[{td[0]:.1f},{td[1]:.1f},{td[2]:.1f},{td[3]:.1f},{td[4]:.1f}]° '
            f'pitch={pitch_deg:.1f}°')

        jmsg = JointState()
        jmsg.header.stamp = self.get_clock().now().to_msg()
        jmsg.name = JOINT_NAMES
        jmsg.position = [t1, t2, t3, t4, t5, self._gripper_angle_rad]
        self.joint_pub.publish(jmsg)

        # Also publish arm_status for calib_collect.py compatibility
        smsg = ArmStatus()
        smsg.header.stamp = self.get_clock().now().to_msg()
        smsg.x = x
        smsg.y = y
        smsg.z = z
        smsg.status = 1
        self.status_pub.publish(smsg)


def main():
    rclpy.init()
    node = IKSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
