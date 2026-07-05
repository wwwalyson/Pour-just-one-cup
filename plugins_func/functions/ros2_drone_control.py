import json
from typing import TYPE_CHECKING

from plugins_func.register import register_function, ToolType, ActionResponse, Action
from core.providers.tools.ros2_drone_client import Ros2DroneClient

if TYPE_CHECKING:
    from core.connection import ConnectionHandler

TAG = __name__

ros2_drone_control_function_desc = {
    "type": "function",
    "function": {
        "name": "ros2_drone_control",
        "description": "通过 ROS2 服务向另一台 RDKX5 发送无人机控制指令，并返回服务端回复。",
        "parameters": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "description": "无人机动作，例如 takeoff、land、move、yaw、hover。",
                },
                "params": {
                    "type": "object",
                    "description": "动作参数，例如 {\"altitude\":2.0}、{\"direction\":\"forward\",\"distance\":3.0}。",
                },
                "service_name": {
                    "type": "string",
                    "description": "ROS2 服务名称，默认 /drone_command。",
                },
                "ros_domain_id": {
                    "type": "integer",
                    "description": "ROS2 域 ID，用于同一网络内节点发现，默认 0。",
                },
                "timeout": {
                    "type": "integer",
                    "description": "调用 ROS2 服务的超时时间，单位秒，默认 5。",
                },
            },
            "required": ["action"],
        },
    },
}


@register_function("ros2_drone_control", ros2_drone_control_function_desc, ToolType.SYSTEM_CTL)
def ros2_drone_control(
    conn: "ConnectionHandler",
    action: str,
    params: dict | str | None = None,
    service_name: str | None = None,
    ros_domain_id: int | None = None,
    timeout: int = 5,
):
    try:
        if isinstance(params, str):
            try:
                params = json.loads(params)
            except json.JSONDecodeError:
                params = {}
        elif params is None:
            params = {}

        plugin_config = conn.config.get("plugins", {}).get("ros2_drone_control", {})
        service_name = service_name or plugin_config.get("service_name", "/drone_command")
        ros_domain_id = (
            ros_domain_id
            if ros_domain_id is not None
            else plugin_config.get("ros_domain_id", 0)
        )
        timeout = timeout or plugin_config.get("timeout", 5)

        client = Ros2DroneClient(
            service_name=service_name,
            timeout_sec=float(timeout),
            ros_domain_id=ros_domain_id,
        )
        try:
            # 诊断服务可见性
            services = client.list_services()
            visible = client.is_service_visible(service_name)
            if not visible:
                available_services = [name for name, _ in services]
                error_msg = (
                    f"ROS2 服务 {service_name} 不可见。当前可用服务: {available_services[:10]}"
                    f"{'...' if len(available_services) > 10 else ''}。请检查 ROS_DOMAIN_ID、网络连接或服务名。"
                )
                raise RuntimeError(error_msg)
            
            response = client.send_command(action, params)
        finally:
            client.close()

        return ActionResponse(
            action=Action.RESPONSE,
            result=response,
            response=f"无人机服务端已响应：{response}",
        )
    except Exception as exc:
        error_text = str(exc)
        if hasattr(conn, "logger"):
            conn.logger.bind(tag=TAG).error(f"ros2_drone_control 调用失败: {error_text}")
        return ActionResponse(
            action=Action.ERROR,
            result=error_text,
            response=f"调用无人机 ROS2 服务失败：{error_text}",
        )
