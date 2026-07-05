#include "arm_cmd/arm_cmd.h"

#include "arm_msg/msg/arm.hpp"
#include "arm_msg/msg/arm_status.hpp"
#include "rclcpp/rclcpp.hpp"

ArmCmd::ArmCmd()
    : rclcpp::Node("arm_cmd"),
      cur_x_(kDefaultX),
      cur_y_(kDefaultY),
      cur_z_(kDefaultZ),
      cmd_sent_(false) {
  this->declare_parameter<float>("target_x", kDefaultTargetX);
  this->declare_parameter<float>("target_y", kDefaultTargetY);
  this->declare_parameter<float>("target_z", kDefaultTargetZ);
  this->declare_parameter<bool>("ik_mode", false);
  this->get_parameter("target_x", target_x_);
  this->get_parameter("target_y", target_y_);
  this->get_parameter("target_z", target_z_);
  bool ik_mode = this->get_parameter("ik_mode").as_bool();

  arm_cmd_pub_ = create_publisher<arm_msg::msg::Arm>("arm_cmd", 10);

  if (ik_mode) {
    // IK mode: publish absolute target coordinates directly.
    // No need for arm_status feedback (bypasses STM32 coordinate accumulation).
    auto msg = arm_msg::msg::Arm();
    msg.x = target_x_;
    msg.y = target_y_;
    msg.z = target_z_;
    arm_cmd_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(),
                "IK mode: 绝对坐标已发送 (%.2f, %.2f, %.2f)",
                target_x_, target_y_, target_z_);
    cmd_sent_ = true;
  } else {
    arm_status_sub_ = create_subscription<arm_msg::msg::ArmStatus>(
        "arm_status", rclcpp::QoS(1).transient_local(),
        [this](const arm_msg::msg::ArmStatus::SharedPtr msg) {
          StatusCallback(msg);
        });
  }

  RCLCPP_INFO(this->get_logger(), "arm_cmd 已启动，目标位置: (%.2f, %.2f, %.2f)",
              target_x_, target_y_, target_z_);
}

void ArmCmd::StatusCallback(const arm_msg::msg::ArmStatus::SharedPtr msg) {
  cur_x_ = msg->x;
  cur_y_ = msg->y;
  cur_z_ = msg->z;

  if (cmd_sent_) return;

  float dx = target_x_ - cur_x_;
  float dy = target_y_ - cur_y_;
  float dz = target_z_ - cur_z_;

  if (std::abs(dx) < 0.1f && std::abs(dy) < 0.1f && std::abs(dz) < 0.1f) {
    RCLCPP_INFO(this->get_logger(), "已在目标位置，无需移动");
    cmd_sent_ = true;
    return;
  }

  SendDelta(dx, dy, dz);
  cmd_sent_ = true;
}

void ArmCmd::SendDelta(float dx, float dy, float dz) {
  auto msg = arm_msg::msg::Arm();
  msg.x = dx;
  msg.y = dy;
  msg.z = dz;
  arm_cmd_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "发送增量指令: dx=%.2f dy=%.2f dz=%.2f", dx,
              dy, dz);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmCmd>());
  rclcpp::shutdown();
  return 0;
}
