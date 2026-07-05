#!/usr/bin/env python3
"""
Tea task scheduler node.

Subscribes to YOLO detection results (/target_grasp_array) and executes
tea/water pouring sequences triggered via ROS2 topic.

Usage:
    ros2 launch arm_task tea_task.launch.py

Trigger:
    ros2 topic pub -1 /task_cmd std_msgs/msg/String "data: '1'"   # tea box
    ros2 topic pub -1 /task_cmd std_msgs/msg/String "data: '2'"   # teapot
"""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from arm_msg.msg import Arm


# ── DH parameters (cm) ──────────────────────────────────────────────
L1 = 2.89
L2 = 10.43
L3 = 8.9
L4 = 17.7
MAX_2LINK = L2 + L3  # 19.33 cm

# ── Constants ───────────────────────────────────────────────────────
CLASS_CADDY = 'caddy'
CLASS_BOTTLE = 'bottle'
CLASS_CUP = 'cup'

DEFAULT_MOVE_TIME_MS = 1500
SETTLE_SEC = 2.0             # hold still after arriving at target
POUR_HOLD_SEC = 3.0           # hold pour position before returning

# Gripper angles: 0°=fully closed (pulse 700), 90°=fully open (pulse 200)
GRIPPER_OPEN_RAD = math.radians(90.0)
GRIPPER_CLOSE_CADDY_RAD = math.radians(25.0)
GRIPPER_CLOSE_BOTTLE_RAD = math.radians(15.0)

# Pour (tea box): rotate wrist_roll to -90°
POUR_WRIST_ROLL_RAD = math.radians(-90.0)
# Pour (teapot): tilt wrist_flex forward like human pouring
POUR_WRIST_FLEX_RAD = math.radians(-60.0)
POUR_TEAPOT_HOLD_SEC = 5.0

# Joint names matching ik_node output
JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow',
               'wrist_flex', 'wrist_roll', 'grip_left']


class TeaTaskNode(Node):
    """Orchestrates tea-making sequences using YOLO detection + IK arm control.

    Task state machine (per task):
        IDLE → MOVING → ARRIVED → SETTLING → ACTING → MOVING → ...
    """

    def __init__(self):
        super().__init__('tea_task_node')

        # ── State ──────────────────────────────────────────────────
        self.detected_objects = {}
        self.cached_joints = None
        self.cached_joint_positions = {}
        self.task_running = False
        self.move_time_ms = DEFAULT_MOVE_TIME_MS

        # ── Subscribers ────────────────────────────────────────────
        self.yolo_sub = self.create_subscription(
            String, '/target_grasp_array', self._yolo_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_target', self._joint_callback, 10)
        self.task_cmd_sub = self.create_subscription(
            String, '/task_cmd', self._task_cmd_callback, 10)

        # ── Publishers ─────────────────────────────────────────────
        self.arm_cmd_pub = self.create_publisher(Arm, '/arm_cmd', 10)
        self.joint_pub = self.create_publisher(
            JointState, '/joint_target', 10)
        # Dedicated gripper channel: arm_base ignores grip_left from
        # /joint_target and only uses /grip_cmd for servo 1.
        self.grip_cmd_pub = self.create_publisher(
            Float64, '/grip_cmd', 10)

        # ── Keyboard thread (fallback) ─────────────────────────────
        self.running = True
        self.kb_thread = threading.Thread(target=self._kb_loop, daemon=True)
        self.kb_thread.start()

        self.get_logger().info('🍵 Tea Task Node ready.')
        self.get_logger().info(
            '   Trigger: ros2 topic pub -1 /task_cmd std_msgs/msg/String "data: \'1\'"')

    # ═══════════════════════════════════════════════════════════════
    # Callbacks
    # ═══════════════════════════════════════════════════════════════

    def _yolo_callback(self, msg: String):
        """Parse YOLO detection results (skipped during task execution)."""
        if self.task_running:
            return  # 🔒 Freeze: don't update positions while task is running
        try:
            targets = json.loads(msg.data)
            for t in targets:
                cls = t.get('class', '')
                x = t.get('x', 0.0)
                y = t.get('y', 0.0)
                z = t.get('z', 0.0)
                score = t.get('score', 0.0)
                self.detected_objects[cls] = (x, y, z, score)
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f'Failed to parse YOLO data: {e}')

    def _joint_callback(self, msg: JointState):
        """Cache latest joint angles from IK solver."""
        self.cached_joints = msg
        for name, pos in zip(msg.name, msg.position):
            self.cached_joint_positions[name] = pos

    def _task_cmd_callback(self, msg: String):
        """Topic-based task trigger."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Received task command: "{cmd}"')
        self._dispatch(cmd)

    def _dispatch(self, cmd: str):
        """Route command to the appropriate task."""
        if cmd in ('1', 'tea', 'caddy', 'teabox'):
            if not self.task_running:
                threading.Thread(target=self._execute_tea_box, daemon=True).start()
            else:
                self.get_logger().warn('Task already running, wait...')
        elif cmd in ('2', 'water', 'bottle', 'teapot'):
            if not self.task_running:
                threading.Thread(target=self._execute_teapot, daemon=True).start()
            else:
                self.get_logger().warn('Task already running, wait...')
        elif cmd in ('q', 'quit', 'stop'):
            self.get_logger().info('Stop requested.')
            self.running = False
        else:
            self.get_logger().info(
                f'Unknown: "{cmd}". Valid: 1/tea/caddy  2/water/bottle  q/quit')

    # ═══════════════════════════════════════════════════════════════
    # Keyboard input (fallback)
    # ═══════════════════════════════════════════════════════════════

    def _kb_loop(self):
        """Terminal input thread."""
        try:
            infile = open('/dev/tty', 'r')
        except (IOError, OSError):
            import sys
            infile = sys.stdin

        while self.running:
            try:
                line = infile.readline()
                if not line:
                    break
                cmd = line.strip().lower()
                if cmd:
                    self._dispatch(cmd)
            except (EOFError, IOError):
                break

    # ═══════════════════════════════════════════════════════════════
    # Safe transit Z computation
    # ═══════════════════════════════════════════════════════════════

    def _compute_transit_z(self, x, y, min_z=8.0):
        """Compute a reachable transit Z for a given XY position."""
        r = math.sqrt(x * x + y * y)
        r_w = r - L4
        if r_w >= MAX_2LINK:
            return min_z
        h_w_max = math.sqrt(MAX_2LINK * MAX_2LINK - r_w * r_w)
        z_max = h_w_max + L1
        z_safe = z_max * 0.8
        return max(z_safe, min_z)

    def _compute_max_z(self, x, y, margin=0.95):
        """Compute the maximum reachable Z at a given XY position.

        Uses pitch=0° (flat end effector) for maximum vertical reach.
        margin: fraction of theoretical max (0.95 = 5% safety margin).
        """
        r = math.sqrt(x * x + y * y)
        r_w = r - L4  # wrist at pitch=0
        if r_w >= MAX_2LINK:
            return 8.0  # unreachable
        h_w = math.sqrt(MAX_2LINK * MAX_2LINK - r_w * r_w)
        return (h_w + L1) * margin

    # ═══════════════════════════════════════════════════════════════
    # State machine primitives
    #
    # Each task step follows a strict 3-phase pattern:
    #   1. MOVING:   publish target → wait for interpolation to finish
    #   2. ARRIVED:  (implicit) arm has reached the target
    #   3. SETTLING: hold position for SETTLE_SEC to let arm stabilize
    #   4. ACTING:   execute gripper/wrist/pour action
    # ═══════════════════════════════════════════════════════════════

    def _move_to(self, x, y, z):
        """State: MOVING — send target position to IK pipeline."""
        msg = Arm()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.arm_cmd_pub.publish(msg)
        self.get_logger().info(f'  [MOVING] ➤ ({x:.1f}, {y:.1f}, {z:.1f})')

    def _wait_interpolation(self):
        """Wait for arm_base interpolation to complete (move_time_ms)."""
        sec = self.move_time_ms / 1000.0
        self.get_logger().info(f'  ⏳ interpolation {sec:.1f}s...')
        time.sleep(sec)

    def _settle(self):
        """State: SETTLING — arm arrived, hold position to stabilize."""
        self.get_logger().info(f'  [SETTLING] holding {SETTLE_SEC}s...')
        time.sleep(SETTLE_SEC)

    def _do_gripper(self, angle_rad, label):
        """Gripper action via dedicated /grip_cmd channel.

        arm_base handles /grip_cmd independently from /joint_target,
        so ik_node's arm movements can never interfere with the gripper.
        """
        msg = Float64()
        msg.data = float(angle_rad)
        self.grip_cmd_pub.publish(msg)
        self.get_logger().info(f'  [MOVING] gripper → {label}')
        self._wait_interpolation()
        self._settle()

    def _do_pour(self):
        """Pour sequence: MOVING → SETTLING → HOLD 5s → MOVING back → SETTLING."""
        # Phase 1: rotate wrist to pour angle
        self._publish_joint_override('wrist_roll', POUR_WRIST_ROLL_RAD)
        self.get_logger().info(f'  [MOVING] wrist → {math.degrees(POUR_WRIST_ROLL_RAD):.0f}°')
        self._wait_interpolation()
        self._settle()

        # Phase 2: hold pour position (let tea/water flow out)
        self.get_logger().info(f'  [HOLD] pouring {POUR_HOLD_SEC:.0f}s...')
        time.sleep(POUR_HOLD_SEC)

        # Phase 3: rotate wrist back to neutral
        self._publish_joint_override('wrist_roll', 0.0)
        self.get_logger().info('  [MOVING] wrist → 0°')
        self._wait_interpolation()
        self._settle()

    def _do_pour_flex(self):
        """Teapot pour: tilt wrist_flex (servo 3) forward like human pouring."""
        # Phase 1: tilt wrist forward
        self._publish_joint_override('wrist_flex', POUR_WRIST_FLEX_RAD)
        self.get_logger().info(
            f'  [MOVING] wrist_flex → {math.degrees(POUR_WRIST_FLEX_RAD):.0f}°')
        self._wait_interpolation()
        self._settle()

        # Phase 2: hold pour
        self.get_logger().info(f'  [HOLD] pouring {POUR_TEAPOT_HOLD_SEC:.0f}s...')
        time.sleep(POUR_TEAPOT_HOLD_SEC)

        # Phase 3: return wrist to neutral
        self._publish_joint_override('wrist_flex', 0.0)
        self.get_logger().info('  [MOVING] wrist_flex → 0°')
        self._wait_interpolation()
        self._settle()

    def _publish_joint_override(self, joint_name, angle_rad):
        """Publish full JointState from cache, overriding one joint."""
        if self.cached_joints is None or not self.cached_joint_positions:
            self.get_logger().warn(
                f'No cached joint state yet, cannot set {joint_name}')
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES

        positions = []
        for name in JOINT_NAMES:
            if name == joint_name:
                positions.append(float(angle_rad))
            else:
                positions.append(
                    self.cached_joint_positions.get(name, 0.0))

        msg.position = positions
        self.joint_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════
    # Task: Tea Box (caddy)
    # ═══════════════════════════════════════════════════════════════

    def _execute_tea_box(self):
        """State machine: pick tea box → pour → return."""
        self.task_running = True
        try:
            # 🔒 Snapshot positions at task start
            if CLASS_CADDY not in self.detected_objects:
                self.get_logger().error('❌ Tea box (caddy) not detected!')
                return
            if CLASS_CUP not in self.detected_objects:
                self.get_logger().error('❌ Cup not detected!')
                return

            caddy = self.detected_objects[CLASS_CADDY]
            cup = self.detected_objects[CLASS_CUP]
            cx, cy, cz = caddy[0], caddy[1], caddy[2]
            ux, uy, uz = cup[0], cup[1], cup[2]
            grab_z = cz
            transit_z = self._compute_transit_z(cx, cy)
            cup_transit_z = self._compute_transit_z(ux, uy)

            self.get_logger().info(
                f'🍵 Tea Box: caddy=({cx:.1f},{cy:.1f},{cz:.1f}) '
                f'cup=({ux:.1f},{uy:.1f},{uz:.1f}) '
                f'transitZ={transit_z:.1f} cupTransitZ={cup_transit_z:.1f}')

            # ── State sequence ──────────────────────────────────

            # S1: Move above caddy
            self._move_to(cx, cy, transit_z)
            self._wait_interpolation()
            self._settle()

            # S2: Lower to grab height
            self._move_to(cx, cy, grab_z)
            self._wait_interpolation()
            self._settle()

            # S3: Close gripper
            self._do_gripper(GRIPPER_CLOSE_CADDY_RAD, 'close')

            # S4: Lift
            self._move_to(cx, cy, transit_z)
            self._wait_interpolation()
            self._settle()

            # S5: Move above cup
            self._move_to(ux, uy-3.0, cup_transit_z+2.0)
            self._wait_interpolation()
            self._settle()

            # S6: Pour
            self._do_pour()

            # S7: Return to caddy position
            self._move_to(cx, cy, transit_z)
            self._wait_interpolation()
            self._settle()

            # S8: Lower to put-down height
            self._move_to(cx, cy, grab_z)
            self._wait_interpolation()
            self._settle()

            # S9: Open gripper
            self._do_gripper(GRIPPER_OPEN_RAD, 'open')

            # S10: Lift away
            self._move_to(cx, cy, transit_z)
            self._wait_interpolation()
            self._settle()

            self.get_logger().info('✅ Tea box task completed!')

        except Exception as e:
            self.get_logger().error(f'Task failed: {e}')
        finally:
            self.task_running = False

    # ═══════════════════════════════════════════════════════════════
    # Task: Teapot (bottle)
    # ═══════════════════════════════════════════════════════════════

    def _execute_teapot(self):
        """Teapot task:
        home → high approach (Y-3) → grab → retract to x=15 + lift high
        → rotate to cup front at max height → pour (wrist_flex tilt) → return.
        """
        self.task_running = True
        try:
            if CLASS_BOTTLE not in self.detected_objects:
                self.get_logger().error('❌ Teapot (bottle) not detected!')
                return
            if CLASS_CUP not in self.detected_objects:
                self.get_logger().error('❌ Cup not detected!')
                return

            bottle = self.detected_objects[CLASS_BOTTLE]
            cup = self.detected_objects[CLASS_CUP]
            bx, by, bz = bottle[0], bottle[1], bottle[2]
            ux, uy, uz = cup[0], cup[1], cup[2]

            # Y compensation: offset -3 cm for better alignment
            gy = by - 3.0
            # Approach Z: at least 10 cm above object
            approach_z = max(bz + 10.0, 12.0)
            grab_z = bz - 2.0
            # Retract: pull back to x=15, keep Y, raise high
            retract_z = self._compute_max_z(15.0, gy, margin=0.90)
            # Pour position: in front of cup at max height
            cup_dist = math.sqrt(ux * ux + uy * uy)
            if cup_dist > 1.0:
                pour_x = ux - 6.0 * (ux / cup_dist)
                pour_y = uy - 6.0 * (uy / cup_dist)
            else:
                pour_x, pour_y = ux - 6.0, uy
            pour_z = self._compute_max_z(pour_x, pour_y)

            self.get_logger().info(
                f'🫖 Teapot: bottle=({bx:.1f},{by:.1f}) grabY={gy:.1f} '
                f'cup=({ux:.1f},{uy:.1f}) '
                f'approachZ={approach_z:.1f} retractZ={retract_z:.1f} '
                f'pour=({pour_x:.1f},{pour_y:.1f},{pour_z:.1f})')

            # ── State sequence ──────────────────────────────────

            # S1: Open gripper and move to home
            self._do_gripper(GRIPPER_OPEN_RAD, 'open')
            self._move_to(15.0, 0.0, 2.0)
            self._wait_interpolation()
            self._settle()

            # S2: Move above teapot (Y-3 compensated, high approach)
            self._move_to(bx, gy, approach_z)
            self._wait_interpolation()
            self._settle()

            # S3: Lower to grab height
            self._move_to(bx, gy, grab_z)
            self._wait_interpolation()
            self._settle()

            # S4: Close gripper
            self._do_gripper(GRIPPER_CLOSE_BOTTLE_RAD, 'close')

            # S5: Retract to x=15 and lift high (safe transit position)
            self._move_to(15.0, gy, retract_z)
            self._wait_interpolation()
            self._settle()

            # S6: Move to pour position (in front of cup, max height)
            self._move_to(pour_x, pour_y, pour_z)
            self._wait_interpolation()
            self._settle()

            # S7: Pour — tilt wrist_flex forward like human pouring
            self._do_pour_flex()

            # S8: Retract to safe position before returning
            self._move_to(15.0, gy, retract_z)
            self._wait_interpolation()
            self._settle()

            # S9: Return above teapot
            self._move_to(bx, gy, approach_z)
            self._wait_interpolation()
            self._settle()

            # S10: Lower to put-down height
            self._move_to(bx, gy, grab_z)
            self._wait_interpolation()
            self._settle()

            # S11: Open gripper
            self._do_gripper(GRIPPER_OPEN_RAD, 'open')

            # S12: Lift away
            self._move_to(bx, gy, approach_z)
            self._wait_interpolation()
            self._settle()

            # S13: Return to home
            self._move_to(15.0, 0.0, 2.0)
            self._wait_interpolation()
            self._settle()

            self.get_logger().info('✅ Teapot task completed!')

        except Exception as e:
            self.get_logger().error(f'Task failed: {e}')
        finally:
            self.task_running = False


def main():
    rclpy.init()
    node = TeaTaskNode()

    def spin():
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    try:
        while node.running:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    node.running = False
    spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
