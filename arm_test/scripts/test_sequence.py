#!/usr/bin/env python3
"""简易测试节点：按序列发送目标坐标，到达后触发夹取动作。

工作流程:
  1. 订阅 arm_status 获取当前末端位置
  2. 计算增量 dx/dy/dz = target - current
  3. 发布 arm_cmd 发送增量指令
  4. 等待运动时间 + 缓冲时间（机械臂到达目标）
  5. 触发夹取动作（发布 gripper_cmd）
  6. 继续下一步

使用:
  ros2 run arm_test test_sequence
"""

import time
import rclpy
from rclpy.node import Node
from arm_msg.msg import Arm, ArmStatus, GripperCmd


class TestSequence(Node):

    def __init__(self):
        super().__init__('test_sequence')

        # 发布者
        self.arm_cmd_pub = self.create_publisher(Arm, 'arm_cmd', 10)
        self.gripper_cmd_pub = self.create_publisher(GripperCmd, 'gripper_cmd', 10)

        # 订阅 arm_status (transient_local 保证拿到最新位置)
        self.status_sub = self.create_subscription(
            ArmStatus, 'arm_status',
            self._status_callback,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self._cur_x = 15.0
        self._cur_y = 0.0
        self._cur_z = 2.0
        self._status_received = False

        # 等待拿到初始位置
        self.get_logger().info('等待 arm_status...')
        self._wait_for_status()

        # 执行测试序列
        self._run_sequence()

    def _status_callback(self, msg):
        self._cur_x = msg.x
        self._cur_y = msg.y
        self._cur_z = msg.z
        self._status_received = True

    def _wait_for_status(self, timeout=5.0):
        start = time.time()
        while rclpy.ok() and not self._status_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error('超时未收到 arm_status')
                raise RuntimeError('arm_status timeout')

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------

    def move_to(self, x, y, z, move_time_ms=1000):
        """发送增量指令，将末端移动到绝对坐标 (x, y, z) cm"""
        dx = x - self._cur_x
        dy = y - self._cur_y
        dz = z - self._cur_z

        if abs(dx) < 0.05 and abs(dy) < 0.05 and abs(dz) < 0.05:
            self.get_logger().info(
                f'已在目标位置 ({x:.1f}, {y:.1f}, {z:.1f})，跳过移动')
            return

        msg = Arm()
        msg.x = dx
        msg.y = dy
        msg.z = dz
        self.arm_cmd_pub.publish(msg)

        # 更新本地记录的当前位置
        self._cur_x += dx
        self._cur_y += dy
        self._cur_z += dz

        self.get_logger().info(
            f'移动: ({self._cur_x - dx:.1f}, {self._cur_y - dy:.1f}, '
            f'{self._cur_z - dz:.1f}) → ({self._cur_x:.1f}, {self._cur_y:.1f}, '
            f'{self._cur_z:.1f}), 增量=({dx:.1f}, {dy:.1f}, {dz:.1f})')

        # 等待运动完成：运动时间 + 200ms 缓冲
        wait_s = move_time_ms / 1000.0 + 0.2
        time.sleep(wait_s)

    def gripper(self, angle, move_time_ms=500):
        """控制夹取结构开合。注意物理映射: 0=全开, 180=闭合"""
        msg = GripperCmd()
        msg.angle = angle
        msg.time_ms = move_time_ms
        self.gripper_cmd_pub.publish(msg)

        if angle < 10.0:
            action = '张开'
        elif angle > 170.0:
            action = '闭合'
        else:
            action = f'开到 {angle:.0f}°'
        self.get_logger().info(
            f'夹取动作: {action}, 时间={move_time_ms}ms')

        # 等待夹取完成
        wait_s = move_time_ms / 1000.0 + 0.2
        time.sleep(wait_s)

    # ------------------------------------------------------------------
    # 测试序列
    # ------------------------------------------------------------------

    def _run_sequence(self):
        """定义并执行测试序列。

        按需修改这里的坐标和动作来测试不同场景。
        """
        self.get_logger().info('========== 开始测试序列 ==========')

        # ---- 步骤 1: 移动到抓取位置上方 ----
        self.get_logger().info('步骤1: 移动到勺子位置上方')
        self.move_to(13.0, 4.0, 10.0, move_time_ms=1500)

        # ---- 步骤 2: 下降到抓取高度 ----
        self.get_logger().info('步骤2: 下降到抓取高度')
        self.move_to(13.0, 4.0, 3.0, move_time_ms=1000)

        # ---- 步骤 3: 闭合夹爪, 抓取勺子 ----
        self.get_logger().info('步骤3: 闭合夹爪抓取勺子')
        self.gripper(angle=180.0, move_time_ms=800)

        # ---- 步骤 4: 抬起 ----
        self.get_logger().info('步骤4: 抬起物体')
        self.move_to(13.0, 4.0, 12.0, move_time_ms=1000)

        # ---- 步骤 5: 移动到茶杯上方 ----
        self.get_logger().info('步骤5: 移动到茶杯上方')
        self.move_to(17.0, -3.0, 12.0, move_time_ms=1500)

        # ---- 步骤 6: 下降到投放高度 ----
        self.get_logger().info('步骤6: 下降到投放高度')
        self.move_to(17.0, -3.0, 5.0, move_time_ms=1000)

        # ---- 步骤 7: 张开夹爪, 释放 ----
        self.get_logger().info('步骤7: 张开夹爪释放')
        self.gripper(angle=0.0, move_time_ms=800)

        # ---- 步骤 8: 回到默认位置 ----
        self.get_logger().info('步骤8: 回到默认位置')
        self.move_to(15.0, 0.0, 2.0, move_time_ms=1500)

        self.get_logger().info('========== 测试序列完成 ==========')


def main():
    rclpy.init()
    node = TestSequence()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
