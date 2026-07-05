#!/usr/bin/env python3
"""ROS2 机械臂控制客户端库。

参照 ros2_drone_client.py，适配 arm_msg.srv.TeaCommand 服务类型。
"""

from __future__ import annotations

import ctypes
import json
import os
import subprocess
import sys
from typing import Any, Dict, Optional


def _normalize_path_list(path_value: str) -> str:
    paths = []
    for path in path_value.split(':'):
        if not path:
            continue
        if path not in paths:
            paths.append(path)
    return ':'.join(paths)


def _load_ros2_environment() -> None:
    """加载 ROS2 环境，同时 source arm_ws 和 ros2_ws。"""

    workspaces = [
        os.path.expanduser("~/arm_ws/install"),
        os.path.expanduser("~/ros2_ws/install"),
    ]

    ros_env = {}
    for ws_root in workspaces:
        if not os.path.exists(ws_root):
            print(f"[ROS2 Arm] 跳过不存在的 workspace: {ws_root}")
            continue
        result = subprocess.run(
            f"bash -lc 'source {ws_root}/setup.bash 2>/dev/null && env'",
            shell=True,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            print(f"[ROS2 Arm] 无法加载 {ws_root}: {result.stderr.strip()}")
            continue
        for line in result.stdout.splitlines():
            if '=' in line:
                key, value = line.split('=', 1)
                ros_env[key] = value

    # 合并到 os.environ
    existing_env = {
        'PYTHONPATH': os.environ.get('PYTHONPATH', ''),
        'PATH': os.environ.get('PATH', ''),
        'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH', ''),
        'AMENT_PREFIX_PATH': os.environ.get('AMENT_PREFIX_PATH', ''),
    }

    for key in [
        'PYTHONPATH', 'PATH', 'LD_LIBRARY_PATH',
        'AMENT_PREFIX_PATH', 'ROS_DISTRO', 'ROS_VERSION',
    ]:
        if key in ros_env:
            value = ros_env[key]
            if key in existing_env and existing_env[key]:
                value = f"{value}:{existing_env[key]}"
            os.environ[key] = value

    if 'PYTHONPATH' in os.environ:
        for path in os.environ['PYTHONPATH'].split(':'):
            if path and path not in sys.path:
                sys.path.insert(0, path)

    # 补充路径
    extra_paths = [
        os.path.join(os.path.expanduser("~/arm_ws/install"),
                     'arm_msg/lib/python3.10/site-packages'),
        os.path.join(os.path.expanduser("~/ros2_ws/install"),
                     'drone_interfaces/lib/python3.10/site-packages'),
        '/opt/ros/humble/lib/python3.10/site-packages',
        '/opt/ros/humble/local/lib/python3.10/dist-packages',
    ]
    for path in extra_paths:
        if os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)


def _preload_ros2_type_support_libs() -> None:
    """预加载 arm_msg 和 drone_interfaces 的 ROS2 类型支持共享库。"""
    lib_dirs = [
        os.path.join(os.path.expanduser("~/arm_ws/install"), "arm_msg", "lib"),
        os.path.join(os.path.expanduser("~/ros2_ws/install"), "drone_interfaces", "lib"),
    ]
    mode = getattr(ctypes, 'RTLD_GLOBAL', 0)

    lib_names = [
        "libarm_msg__rosidl_generator_py.so",
        "libarm_msg__rosidl_typesupport_c.so",
        "libarm_msg__rosidl_typesupport_fastrtps_c.so",
        "libarm_msg__rosidl_typesupport_fastrtps_cpp.so",
        "libarm_msg__rosidl_generator_c.so",
        "libdrone_interfaces__rosidl_generator_py.so",
        "libdrone_interfaces__rosidl_typesupport_c.so",
        "libdrone_interfaces__rosidl_typesupport_fastrtps_c.so",
        "libdrone_interfaces__rosidl_typesupport_fastrtps_cpp.so",
        "libdrone_interfaces__rosidl_generator_c.so",
    ]

    for lib_name in lib_names:
        for lib_dir in lib_dirs:
            lib_path = os.path.join(lib_dir, lib_name)
            if os.path.exists(lib_path):
                try:
                    ctypes.CDLL(lib_path, mode=mode)
                    print(f"[ROS2 Arm] 预加载共享库: {lib_path}")
                except Exception as exc:
                    print(f"[ROS2 Arm] 预加载共享库失败: {lib_path}: {exc}")


def _import_ros2_types() -> tuple[Any, Any, Any]:
    """动态导入 ROS2 类型，优先使用 arm_msg.srv.TeaCommand。

    Returns:
        (rclpy, Node, ServiceType)
    """
    _load_ros2_environment()
    _preload_ros2_type_support_libs()

    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise ImportError(
            f"无法导入 rclpy，请检查 ROS2 环境。错误: {exc}\n"
            f"当前 PYTHONPATH: {os.environ.get('PYTHONPATH', '')}"
        ) from exc

    # 优先使用 arm_msg.srv.TeaCommand
    service_type = None
    errors = []

    try:
        from arm_msg.srv import TeaCommand
        service_type = TeaCommand
        print("[Info] 使用 arm_msg.srv.TeaCommand")
    except Exception as exc:
        errors.append(f"arm_msg.srv.TeaCommand: {exc}")

    if service_type is None:
        try:
            from drone_interfaces.srv import StringCommand
            service_type = StringCommand
            print("[Info] 使用 drone_interfaces.srv.StringCommand 作为替代")
        except Exception as exc:
            errors.append(f"drone_interfaces.srv.StringCommand: {exc}")

    if service_type is None:
        try:
            from example_interfaces.srv import AddTwoInts
            service_type = AddTwoInts
            print("[Info] 使用 example_interfaces.srv.AddTwoInts 作为替代")
        except Exception as exc:
            errors.append(f"example_interfaces.srv.AddTwoInts: {exc}")

    if service_type is None:
        raise ImportError(
            f"无法导入任何可用的 ROS2 服务类型。"
            f"错误详情: {'; '.join(errors)}"
        )

    return rclpy, Node, service_type


class Ros2ArmClient:
    """ROS2 机械臂控制客户端。

    通过 ROS2 Service 向 tea_task_node 发送泡茶指令。

    Usage:
        client = Ros2ArmClient(service_name="/tea_command", timeout_sec=90)
        response = client.send_command("make_tea")
        client.close()
    """

    def __init__(
        self,
        service_name: str = "/tea_command",
        timeout_sec: float = 90.0,
        node_name: str = "ros2_arm_control_client",
        ros_domain_id: Optional[int] = 0,
    ) -> None:
        self._rclpy, self._Node, self._ServiceType = _import_ros2_types()

        if ros_domain_id is not None:
            os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)

        try:
            if not self._rclpy.get_default_context().ok():
                self._rclpy.init(args=None)
        except Exception as exc:
            import traceback
            raise RuntimeError(
                f"ROS2 初始化失败: {exc}\n{traceback.format_exc()}"
            )

        self._service_name = service_name
        self._timeout_sec = timeout_sec
        self._node = self._Node(node_name)
        try:
            self._client = self._node.create_client(
                self._ServiceType, service_name)
        except Exception as exc:
            import traceback
            raise RuntimeError(
                f"创建 ROS2 客户端失败: {exc}\n{traceback.format_exc()}"
            )

        if not self._client.wait_for_service(timeout_sec=min(timeout_sec, 10.0)):
            self._node.destroy_node()
            raise RuntimeError(
                f"等待 ROS2 服务 {service_name} 可用超时，请确认 tea_task_node 已启动。"
            )

    def list_services(self) -> list:
        try:
            return self._node.get_service_names_and_types()
        except Exception as exc:
            raise RuntimeError(f"获取 ROS2 服务列表失败: {exc}") from exc

    def is_service_visible(self, service_name: str | None = None) -> bool:
        if service_name is None:
            service_name = self._service_name
        for name, _ in self.list_services():
            if name == service_name:
                return True
        return False

    def send_command(self, action: str,
                     params: Optional[Dict[str, Any]] = None) -> str:
        """发送泡茶指令并等待响应。

        Args:
            action: make_tea | pour_tea | pour_water
            params: 额外的 JSON 参数（可选）

        Returns:
            服务端响应的 JSON 字符串
        """
        payload = {"action": action}
        if params:
            payload.update(params)

        json_data = json.dumps(payload, ensure_ascii=False)

        request = self._ServiceType.Request()
        if hasattr(request, 'data'):
            request.data = json_data
        elif hasattr(request, 'a'):
            # AddTwoInts fallback
            request.a = hash(json_data) & 0x7FFFFFFF
            request.b = 0
        else:
            raise RuntimeError(
                f"不支持的 ROS2 服务请求类型: {type(request)}")

        future = self._client.call_async(request)
        self._rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=self._timeout_sec)

        if future.done() and future.result() is not None:
            response = future.result()
            if hasattr(response, 'data'):
                return response.data
            elif hasattr(response, 'sum'):
                return json.dumps({"status": "ok", "msg": "命令已发送"})
            else:
                return json.dumps({"status": "ok", "msg": "命令已发送"})
        else:
            raise RuntimeError(
                f"调用 ROS2 服务 {self._service_name} 失败或超时 "
                f"({self._timeout_sec}s)")

    def close(self) -> None:
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
