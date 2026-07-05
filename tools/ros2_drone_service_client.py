#!/usr/bin/env python3
"""ROS2 服务客户端：发送无人机控制指令给另一台 RDKX5 服务端。"""

from __future__ import annotations

import argparse
import json
import sys

from core.providers.tools.ros2_drone_client import Ros2DroneClient


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ROS2 Drone Control Service Client for RDKX5."
    )
    parser.add_argument(
        "--service-name",
        default="/drone_command",
        help="ROS2 服务名，服务端必须订阅该名称。默认 /drone_command",
    )
    parser.add_argument(
        "--action",
        required=True,
        help="无人机动作，例如 takeoff、land、move、yaw、hover。",
    )
    parser.add_argument(
        "--params",
        default="{}",
        help="JSON 字符串形式的动作参数，例如 '{\"altitude\": 2.0}'.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="等待 ROS2 服务响应的超时时间（秒）。",
    )
    parser.add_argument(
        "--ros-domain-id",
        type=int,
        default=0,
        help="ROS2 域 ID，同一网络内多台设备需使用相同域 ID。",
    )
    parser.add_argument(
        "--diagnose",
        action="store_true",
        help="仅显示 ROS2 服务列表和目标服务可见性，不发送指令。",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        params = json.loads(args.params)
    except json.JSONDecodeError as exc:
        print("参数 parse 失败，请检查 --params 是否为合法 JSON。", file=sys.stderr)
        print(str(exc), file=sys.stderr)
        return 2

    request_json = json.dumps(
        {"action": args.action, "params": params}, ensure_ascii=False
    )

    client = None
    try:
        client = Ros2DroneClient(
            service_name=args.service_name,
            timeout_sec=args.timeout,
            ros_domain_id=args.ros_domain_id,
        )
        print(f"已创建 ROS2 客户端，目标服务：{args.service_name}")

        services = client.list_services()
        print("当前发现的 ROS2 服务：")
        for name, types in services:
            print(f"  {name} -> {types}")

        visible = client.is_service_visible(args.service_name)
        print(
            f"目标服务 {'可见' if visible else '不可见'}，期望类型：{client.service_type_name}"
        )

        if args.diagnose:
            print("诊断模式已启用，不发送指令。")
            return 0

        if not visible:
            print(
                "警告：目标服务当前不可见，可能是 ROS_DOMAIN_ID 不匹配、网络发现失败或服务名不一致。",
                file=sys.stderr,
            )

        print(f"正在调用 ROS2 服务 {args.service_name}，请求内容：{request_json}")
        response = client.send_command(args.action, params)
        print(f"服务返回：{response}")
    except Exception as exc:
        print(f"ROS2 客户端异常：{exc}", file=sys.stderr)
        return 1
    finally:
        if client is not None:
            try:
                client.close()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
