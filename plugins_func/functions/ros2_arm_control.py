"""机械臂泡茶控制插件。

通过 ROS2 Service 调用 tea_task_node 的 /tea_command 服务，
根据用户语音指令控制机械臂执行泡茶动作。

参照 ros2_drone_control.py 的模式。
"""

import json
from typing import TYPE_CHECKING

from plugins_func.register import register_function, ToolType, ActionResponse, Action
from core.providers.tools.ros2_arm_client import Ros2ArmClient

if TYPE_CHECKING:
    from core.connection import ConnectionHandler

TAG = __name__

ros2_arm_control_function_desc = {
    "type": "function",
    "function": {
        "name": "ros2_arm_control",
        "description": (
            "控制六自由度机械臂完成泡茶动作。"
            "机械臂可以夹取茶盒倒茶叶，也可以夹取茶壶倒水。"
            "当用户说'帮我泡茶'、'泡茶'、'冲茶'时，应使用 make_tea（完整流程：先倒茶叶再倒水）。"
            "当用户说'倒茶叶'、'放茶叶'时，应使用 pour_tea。"
            "当用户说'倒水'、'倒开水'、'冲水'时，应使用 pour_water。"
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "description": (
                        "机械臂动作：make_tea=完整泡茶流程（先倒茶叶再倒水），"
                        "pour_tea=仅倒茶叶，pour_water=仅倒水"
                    ),
                    "enum": ["make_tea", "pour_tea", "pour_water"],
                },
                "service_name": {
                    "type": "string",
                    "description": "ROS2 服务名称，默认 /tea_command。",
                },
                "ros_domain_id": {
                    "type": "integer",
                    "description": "ROS2 域 ID，用于同一网络内节点发现，默认读取插件配置。",
                },
                "timeout": {
                    "type": "integer",
                    "description": "调用 ROS2 服务的超时时间，单位秒。make_tea 建议 120，其他建议 60，默认读取插件配置。",
                },
            },
            "required": ["action"],
        },
    },
}


@register_function("ros2_arm_control", ros2_arm_control_function_desc, ToolType.SYSTEM_CTL)
def ros2_arm_control(
    conn: "ConnectionHandler",
    action: str,
    service_name: str | None = None,
    ros_domain_id: int | None = None,
    timeout: int | None = None,
):
    """通过 ROS2 Service 控制机械臂泡茶。

    Args:
        conn: xiaozhi-server 连接对象
        action: make_tea | pour_tea | pour_water
        service_name: 自定义服务名（可选）
        ros_domain_id: ROS2 域 ID（可选）
        timeout: 超时秒数
    """
    try:
        plugin_config = conn.config.get("plugins", {}).get(
            "ros2_arm_control", {})
        service_name = service_name or plugin_config.get(
            "service_name", "/tea_command")
        ros_domain_id = (
            ros_domain_id
            if ros_domain_id is not None
            else plugin_config.get("ros_domain_id", 0)
        )
        timeout = timeout if timeout is not None else plugin_config.get(
            "timeout", 90)

        # 根据 action 调整默认超时
        if timeout == 90 and action == "make_tea":
            timeout = 120  # 完整流程需要更长时间

        client = Ros2ArmClient(
            service_name=service_name,
            timeout_sec=float(timeout),
            ros_domain_id=ros_domain_id,
        )
        try:
            # 诊断服务可见性
            visible = client.is_service_visible(service_name)
            if not visible:
                services = client.list_services()
                available = [name for name, _ in services]
                error_msg = (
                    f"ROS2 服务 {service_name} 不可见。"
                    f"当前可用服务: {available[:10]}"
                    f"{'...' if len(available) > 10 else ''}。"
                    f"请确认 tea_task_node 已启动。"
                )
                raise RuntimeError(error_msg)

            response = client.send_command(action)
        finally:
            client.close()

        # 解析服务端响应
        try:
            result = json.loads(response)
            status = result.get("status", "unknown")
            msg = result.get("msg", "")
        except json.JSONDecodeError:
            status = "unknown"
            msg = response

        # 根据 action 生成自然语言反馈
        feedback_map = {
            "make_tea": "泡茶流程已完成，先倒茶叶再倒水，请享用！",
            "pour_tea": "倒茶叶完成，茶叶已放入杯中。",
            "pour_water": "倒水完成，热水已注入杯中。",
        }

        if status == "ok":
            feedback = feedback_map.get(action, f"机械臂动作 {action} 已完成。")
            return ActionResponse(
                action=Action.RESPONSE,
                result=response,
                response=feedback,
            )
        else:
            return ActionResponse(
                action=Action.RESPONSE,
                result=response,
                response=f"机械臂操作失败：{msg}",
            )

    except Exception as exc:
        error_text = str(exc)
        if hasattr(conn, "logger"):
            conn.logger.bind(tag=TAG).error(
                f"ros2_arm_control 调用失败: {error_text}")
        return ActionResponse(
            action=Action.ERROR,
            result=error_text,
            response=f"机械臂控制失败：{error_text}",
        )
