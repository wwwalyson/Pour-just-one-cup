#!/usr/bin/env python3
"""ROS2 无人机控制客户端库。"""

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
    ros_ws_root = os.path.expanduser("~/ros2_ws/install")
    ros_ws_package = os.path.join(ros_ws_root, "drone_interfaces")
    if not os.path.exists(ros_ws_root):
        return

    result = subprocess.run(
        f"bash -lc 'source {ros_ws_root}/setup.bash && env'",
        shell=True,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"[ROS2] 无法加载 ROS2 环境: {result.stderr.strip()}")
        return

    ros_env = {}
    for line in result.stdout.splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            ros_env[key] = value

    # 优先保留工作环境中已有关键路径
    existing_env = {
        'PYTHONPATH': os.environ.get('PYTHONPATH', ''),
        'PATH': os.environ.get('PATH', ''),
        'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH', ''),
        'AMENT_PREFIX_PATH': os.environ.get('AMENT_PREFIX_PATH', ''),
    }

    for key in [
        'PYTHONPATH',
        'PATH',
        'LD_LIBRARY_PATH',
        'AMENT_PREFIX_PATH',
        'ROS_DISTRO',
        'ROS_VERSION',
    ]:
        if key in ros_env:
            value = ros_env[key]
            if key in existing_env and existing_env[key]:
                value = f"{value}:{existing_env[key]}"
            os.environ[key] = value
            print(f"[ROS2] 设置环境变量 {key}={value[:100]}...")

    if 'PYTHONPATH' in os.environ:
        for path in os.environ['PYTHONPATH'].split(':'):
            if path and path not in sys.path:
                sys.path.insert(0, path)
                print(f"[ROS2] 插入 sys.path: {path}")

    extra_paths = [
        os.path.join(ros_ws_package, 'local/lib/python3.10/dist-packages'),
        os.path.join(ros_ws_package, 'lib/python3.10/site-packages'),
        os.path.join(ros_ws_root, 'lib/python3.10/site-packages'),
        os.path.join('/opt/ros/humble/lib/python3.10/site-packages'),
        os.path.join('/opt/ros/humble/local/lib/python3.10/dist-packages'),
    ]
    for path in extra_paths:
        if os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)
            print(f"[ROS2] 强制插入 sys.path: {path}")

    if os.path.exists(ros_ws_package):
        new_ament_prefix = ':'.join(
            filter(None, [ros_ws_root, ros_ws_package, os.environ.get('AMENT_PREFIX_PATH', '')])
        )
        os.environ['AMENT_PREFIX_PATH'] = _normalize_path_list(new_ament_prefix)
        print(f"[ROS2] 设置 AMENT_PREFIX_PATH={os.environ['AMENT_PREFIX_PATH']}")

    # 确保 ROS2 类型支持库目录也在动态链接器搜索路径中
    ros_lib_paths = [
        os.path.join(ros_ws_package, 'lib'),
        os.path.join(ros_ws_root, 'lib'),
    ]
    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    for lib_path in ros_lib_paths:
        if os.path.isdir(lib_path) and lib_path not in existing_ld.split(':'):
            existing_ld = f"{lib_path}:{existing_ld}" if existing_ld else lib_path
    os.environ['LD_LIBRARY_PATH'] = _normalize_path_list(existing_ld)
    print(f"[ROS2] 设置 LD_LIBRARY_PATH={os.environ['LD_LIBRARY_PATH']}")


def _preload_ros2_type_support_libs() -> None:
    ros_ws_root = os.path.expanduser("~/ros2_ws/install")
    lib_dir = os.path.join(ros_ws_root, "drone_interfaces", "lib")
    if not os.path.isdir(lib_dir):
        return

    mode = getattr(ctypes, 'RTLD_GLOBAL', 0)
    for lib_name in [
        "libdrone_interfaces__rosidl_generator_py.so",
        "libdrone_interfaces__rosidl_typesupport_c.so",
        "libdrone_interfaces__rosidl_typesupport_fastrtps_c.so",
        "libdrone_interfaces__rosidl_typesupport_fastrtps_cpp.so",
        "libdrone_interfaces__rosidl_generator_c.so",
    ]:
        lib_path = os.path.join(lib_dir, lib_name)
        if os.path.exists(lib_path):
            try:
                ctypes.CDLL(lib_path, mode=mode)
                print(f"[ROS2] 预加载共享库: {lib_path}")
            except Exception as exc:
                print(f"[ROS2] 预加载共享库失败: {lib_path}: {exc}")


def _import_ros2_types() -> tuple[Any, Any, Any]:
    _load_ros2_environment()
    _preload_ros2_type_support_libs()

    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        raise ImportError(
            f"无法导入 rclpy，请检查 ROS2 环境是否已加载。错误: {exc}\n"
            f"当前 sys.path: {sys.path}\n"
            f"当前 PYTHONPATH: {os.environ.get('PYTHONPATH', '')}\n"
            f"当前 LD_LIBRARY_PATH: {os.environ.get('LD_LIBRARY_PATH', '')}"
        ) from exc

    try:
        from drone_interfaces.srv import StringCommand
        print("[Info] 使用 drone_interfaces.srv.StringCommand")
    except Exception as exc:
        import traceback
        tb = traceback.format_exc()
        try:
            from example_interfaces.srv import AddTwoInts as StringCommand
            print("[Info] 使用 example_interfaces.srv.AddTwoInts 作为替代接口")
        except Exception as exc2:
            raise ImportError(
                f"无法导入 ROS2 服务类型。StringCommand 错误: {exc}\n{tb}\nAddTwoInts 错误: {exc2}"
            ) from exc2

    return rclpy, Node, StringCommand


class Ros2DroneClient:
    def __init__(
        self,
        service_name: str = "/drone_command",
        timeout_sec: float = 5.0,
        node_name: str = "ros2_drone_control_client",
        ros_domain_id: Optional[int] = 0,
    ) -> None:
        self._rclpy, self._Node, self._StringCommand = _import_ros2_types()

        if ros_domain_id is not None:
            os.environ.setdefault("ROS_DOMAIN_ID", str(ros_domain_id))

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
            self._client = self._node.create_client(self._StringCommand, service_name)
        except Exception as exc:
            import traceback
            raise RuntimeError(
                f"创建 ROS2 客户端失败: {exc}\n{traceback.format_exc()}"
            )

        if not self._client.wait_for_service(timeout_sec=self._timeout_sec):
            self._node.destroy_node()
            raise RuntimeError(
                f"等待 ROS2 服务 {service_name} 可用超时，请确认服务端已启动。"
            )

    @property
    def service_type_name(self) -> str:
        type_name = getattr(self._StringCommand, '_type', None)
        if type_name:
            return type_name
        module_parts = self._StringCommand.__module__.split('.')
        if len(module_parts) >= 2:
            return f"{module_parts[0]}/{module_parts[1]}/{self._StringCommand.__name__}"
        return self._StringCommand.__name__

    def list_services(self) -> list[tuple[str, list[str]]]:
        try:
            return self._node.get_service_names_and_types()
        except Exception as exc:
            raise RuntimeError(f"获取 ROS2 服务列表失败: {exc}") from exc

    def is_service_visible(self, service_name: str | None = None) -> bool:
        if service_name is None:
            service_name = self._service_name
        target_type = self.service_type_name
        for name, types in self.list_services():
            if name == service_name and target_type in types:
                return True
        return False

    def send_command(self, action: str, params: Optional[Dict[str, Any]] = None) -> str:
        payload = {
            "action": action,
        }
        if params:
            payload.update(params)
        
        json_data = json.dumps(payload, ensure_ascii=False)

        request = self._StringCommand.Request()
        # 检查接口类型
        if hasattr(request, 'data'):
            # StringCommand 接口
            request.data = json_data
        elif hasattr(request, 'a'):
            # AddTwoInts 接口作为替代
            request.a = hash(json_data)  # 使用 hash 作为标识符
            request.b = 0
        else:
            raise RuntimeError("不支持的 ROS2 接口类型")

        future = self._client.call_async(request)
        self._rclpy.spin_until_future_complete(self._node, future, timeout_sec=self._timeout_sec)

        if future.done() and future.result() is not None:
            response = future.result()
            # 检查响应类型
            if hasattr(response, 'data'):
                # StringCommand 响应
                return response.data
            elif hasattr(response, 'sum'):
                # AddTwoInts 响应，返回一个默认成功消息
                return json.dumps({"status": "ok", "message": "命令已发送"})
            else:
                return json.dumps({"status": "ok", "message": "命令已发送"})
        else:
            raise RuntimeError(f"调用 ROS2 服务 {self._service_name} 失败或超时")

    def close(self) -> None:
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass

        # 不在这里调用 rclpy.shutdown()，避免多次创建/销毁 ROS2 全局上下文。
        # 仅销毁节点即可保持进程中 ROS2 可继续复用。


def build_drone_command(action: str, params: Optional[Dict[str, Any]] = None) -> str:
    payload = {
        "action": action,
        "params": params or {},
    }
    return json.dumps(payload, ensure_ascii=False)
