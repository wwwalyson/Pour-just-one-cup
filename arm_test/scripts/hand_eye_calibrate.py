#!/usr/bin/env python3
"""
Hand-Eye Calibration (Eye-to-Hand) for LeArm.

Camera is fixed, ArUco marker on robot gripper.
Moves arm through a set of calibration poses spanning the workspace.
At each pose, records the marker position in camera frame (via ArUco detection).
Solves T_base_camera via SVD (absolute orientation from point correspondences).

Usage:
    # With ArUco detection running on /aruco_pose topic:
    ros2 run arm_test hand_eye_calibrate.py

    # With manual coordinate entry:
    ros2 run arm_test hand_eye_calibrate.py --manual

Output:
    Updates hand_eye_calibration.yaml with solved T_base_camera.
"""

import math
import os
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from arm_msg.msg import Arm


class HandEyeCalibrator(Node):
    def __init__(self, config_path, manual_mode=False):
        super().__init__('hand_eye_calibrator')

        self.config_path = config_path
        self.manual_mode = manual_mode

        with open(config_path) as f:
            self.config = yaml.safe_load(f)

        self.poses = self.config['calibration_poses']
        self.records = []  # [(x_base, y_base, z_base, x_cam, y_cam, z_cam)]

        # Publisher: send absolute coordinates to /arm_cmd
        self.cmd_pub = self.create_publisher(Arm, 'arm_cmd', 10)

        # Subscriber: ArUco marker pose in camera frame
        if not manual_mode:
            self.aruco_sub = self.create_subscription(
                PoseStamped, 'aruco_pose', self._aruco_callback, 10)
            self.latest_aruco = None

        self.get_logger().info(
            f'Hand-Eye Calibrator: {len(self.poses)} poses, '
            f'mode={"manual" if manual_mode else "auto"}')

    def _aruco_callback(self, msg: PoseStamped):
        """Receive ArUco marker pose in camera frame."""
        self.latest_aruco = msg

    def _send_target(self, x, y, z):
        """Publish target position to /arm_cmd for IK pipeline."""
        msg = Arm()
        msg.x = x
        msg.y = y
        msg.z = z
        self.cmd_pub.publish(msg)

    def _get_marker_cam(self):
        """Get marker position in camera frame.
        Returns (x, y, z) in cm, or None if no detection.
        """
        if self.manual_mode:
            print("\n  Enter marker position in CAMERA frame (cm):")
            try:
                x = float(input("    X_cam: "))
                y = float(input("    Y_cam: "))
                z = float(input("    Z_cam: "))
                return (x, y, z)
            except (ValueError, EOFError):
                print("  Invalid input, skipping this pose")
                return None
        else:
            # Wait for ArUco detection
            timeout = 5.0
            start = time.time()
            while self.latest_aruco is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - start > timeout:
                    self.get_logger().warn('Timeout waiting for ArUco detection')
                    return None
            p = self.latest_aruco.pose.position
            # Convert ROS PoseStamped (meters) to cm
            return (p.x * 100.0, p.y * 100.0, p.z * 100.0)

    def run(self):
        print("\n" + "=" * 60)
        print("  Hand-Eye Calibration (Eye-to-Hand)")
        print("=" * 60)
        print(f"\n  {len(self.poses)} poses to visit.")
        print("  ArUco marker should be attached to the gripper.")
        print("  Camera should see the workspace.")
        if self.manual_mode:
            print("\n  MANUAL mode: enter camera coordinates by hand.")
        print("\n  Press ENTER after arm arrives at each pose.")
        print("  Type 'skip' to skip a pose, 'quit' to abort.")
        print()

        for i, pose in enumerate(self.poses):
            x, y, z = pose['x'], pose['y'], pose['z']
            roll_deg = pose.get('roll', 0.0)

            print(f"[{i+1}/{len(self.poses)}] "
                  f"Moving to: ({x:.1f}, {y:.1f}, {z:.1f}) cm, roll={roll_deg}°")

            # Send target (roll handled by wrist_roll joint in IK)
            self._send_target(x, y, z)
            print(f"  Arm moving... wait for arm to reach target.")

            # Reset latest ArUco
            if not self.manual_mode:
                self.latest_aruco = None

            # Wait for user confirmation
            while True:
                cmd = input("  [ENTER=record, skip=next, quit=abort]: ").strip().lower()
                if cmd == '':
                    break
                elif cmd == 'skip':
                    print("  Skipping.")
                    break
                elif cmd == 'quit':
                    print("  Aborting calibration.")
                    return
            if cmd == 'skip':
                continue

            # Record marker position
            marker = self._get_marker_cam()
            if marker is None:
                print("  No marker detected, skipping.")
                continue

            x_cam, y_cam, z_cam = marker
            self.records.append((x, y, z, x_cam, y_cam, z_cam))
            print(f"  Recorded: base({x:.1f},{y:.1f},{z:.1f}) "
                  f"← cam({x_cam:.1f},{y_cam:.1f},{z_cam:.1f})")

        # Solve
        if len(self.records) < 4:
            self.get_logger().error(
                f'Need at least 4 valid poses, got {len(self.records)}')
            return

        R, t, rms = solve_hand_eye(self.records)
        if R is None:
            self.get_logger().error('SVD solve failed')
            return

        print(f"\n{'='*60}")
        print(f"  Calibration Result ({len(self.records)} poses, RMSE={rms:.2f} cm)")
        print(f"{'='*60}")
        print(f"\n  Rotation R (3×3):")
        for row in range(3):
            print(f"    [{R[row*3]:8.4f}  {R[row*3+1]:8.4f}  {R[row*3+2]:8.4f}]")
        print(f"\n  Translation t (cm):")
        print(f"    [{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}]")

        # Save
        self.config['T_base_camera']['R'] = list(R)
        self.config['T_base_camera']['t'] = list(t)
        with open(self.config_path, 'w') as f:
            yaml.safe_dump(self.config, f, default_flow_style=False,
                           allow_unicode=True, sort_keys=False)
        print(f"\n  Saved to: {self.config_path}")
        print(f"{'='*60}")


def solve_hand_eye(records):
    """Solve T_base_camera from point correspondences via SVD.

    Given N point pairs: p_i^base = R * p_i^cam + t
    Uses Arun's method (SVD of cross-covariance matrix).

    Args:
        records: list of (x_b, y_b, z_b, x_c, y_c, z_c) in cm

    Returns:
        (R, t, rms) where R is 9-element list (row-major 3×3),
        t is 3-element list, rms is RMS error in cm.
    """
    n = len(records)
    # Separate into arrays
    P_base = [[r[0], r[1], r[2]] for r in records]
    P_cam  = [[r[3], r[4], r[5]] for r in records]

    # Compute centroids
    c_base = [sum(p[i] for p in P_base) / n for i in range(3)]
    c_cam  = [sum(p[i] for p in P_cam)  / n for i in range(3)]

    # Center the points
    A = [[P_base[i][j] - c_base[j] for j in range(3)] for i in range(n)]
    B = [[P_cam[i][j]  - c_cam[j]  for j in range(3)] for i in range(n)]

    # Cross-covariance matrix H = A^T * B
    H = [[0.0]*3 for _ in range(3)]
    for i in range(n):
        for r in range(3):
            for c in range(3):
                H[r][c] += A[i][r] * B[i][c]

    # SVD of H via numpy is easier, but let's do it without numpy
    # For 3×3, use closed-form solution from Eigen decomposition of H^T*H
    # Actually, let's just compute the rotation using the Kabsch algorithm
    # via the SVD of H. Since we can't use numpy, we'll do iterative refinement
    # or use a simpler approach.

    # Use the cross-covariance-based quaternion method (Horn 1987)
    # Build the 4×4 matrix for quaternion extraction
    Sxx, Sxy, Sxz = H[0][0], H[0][1], H[0][2]
    Syx, Syy, Syz = H[1][0], H[1][1], H[1][2]
    Szx, Szy, Szz = H[2][0], H[2][1], H[2][2]

    N_mat = [
        [Sxx+Syy+Szz,  Syz-Szy,      Szx-Sxz,      Sxy-Syx],
        [Syz-Szy,      Sxx-Syy-Szz,  Sxy+Syx,      Szx+Sxz],
        [Szx-Sxz,      Sxy+Syx,      -Sxx+Syy-Szz, Syz+Szy],
        [Sxy-Syx,      Szx+Sxz,      Syz+Szy,      -Sxx-Syy+Szz],
    ]

    # Power iteration to find largest eigenvalue/eigenvector
    # (the optimal quaternion is the eigenvector of max eigenvalue)
    q = [1.0, 0.0, 0.0, 0.0]  # initial guess
    for _ in range(50):
        q_new = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            for j in range(4):
                q_new[i] += N_mat[i][j] * q[j]
        norm = math.sqrt(sum(v*v for v in q_new))
        if norm < 1e-10:
            break
        q_new = [v / norm for v in q_new]
        if sum(abs(q_new[i] - q[i]) for i in range(4)) < 1e-10:
            break
        q = q_new

    qw, qx, qy, qz = q[0], q[1], q[2], q[3]

    # Quaternion to rotation matrix
    R = [
        qw*qw+qx*qx-qy*qy-qz*qz,  2*qx*qy-2*qw*qz,          2*qx*qz+2*qw*qy,
        2*qx*qy+2*qw*qz,          qw*qw-qx*qx+qy*qy-qz*qz,  2*qy*qz-2*qw*qx,
        2*qx*qz-2*qw*qy,          2*qy*qz+2*qw*qx,          qw*qw-qx*qx-qy*qy+qz*qz,
    ]

    # Translation
    t = [c_base[0] - R[0]*c_cam[0] - R[1]*c_cam[1] - R[2]*c_cam[2],
         c_base[1] - R[3]*c_cam[0] - R[4]*c_cam[1] - R[5]*c_cam[2],
         c_base[2] - R[6]*c_cam[0] - R[7]*c_cam[1] - R[8]*c_cam[2]]

    # RMS error
    rms = 0.0
    for i in range(n):
        p_pred = [
            R[0]*P_cam[i][0] + R[1]*P_cam[i][1] + R[2]*P_cam[i][2] + t[0],
            R[3]*P_cam[i][0] + R[4]*P_cam[i][1] + R[5]*P_cam[i][2] + t[1],
            R[6]*P_cam[i][0] + R[7]*P_cam[i][1] + R[8]*P_cam[i][2] + t[2],
        ]
        dx = p_pred[0] - P_base[i][0]
        dy = p_pred[1] - P_base[i][1]
        dz = p_pred[2] - P_base[i][2]
        rms += dx*dx + dy*dy + dz*dz
    rms = math.sqrt(rms / n)

    return R, t, rms


def main():
    rclpy.init(args=sys.argv)

    manual_mode = '--manual' in sys.argv

    pkg_share = os.environ.get(
        'ARM_TEST_SHARE',
        os.path.join(os.path.dirname(__file__), '..', '..', 'share', 'arm_test'))
    config_path = os.path.join(pkg_share, 'config', 'hand_eye_calibration.yaml')
    if not os.path.exists(config_path):
        # Try source tree
        alt = os.path.join(os.path.dirname(__file__), '..', 'config',
                           'hand_eye_calibration.yaml')
        if os.path.exists(alt):
            config_path = alt

    if not os.path.exists(config_path):
        print(f"Config not found: {config_path}", file=sys.stderr)
        sys.exit(1)

    calib = HandEyeCalibrator(config_path, manual_mode=manual_mode)
    calib.run()

    calib.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
