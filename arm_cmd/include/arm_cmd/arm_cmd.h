#ifndef ARM_CMD_ARM_CMD_H_
#define ARM_CMD_ARM_CMD_H_

#include "arm_msg/msg/arm.hpp"
#include "arm_msg/msg/arm_status.hpp"
#include "rclcpp/rclcpp.hpp"

constexpr float kDefaultX = 15.0f;
constexpr float kDefaultY = 0.0f;
constexpr float kDefaultZ = 2.0f;
constexpr float kDefaultTargetX = 15.0f;
constexpr float kDefaultTargetY = 0.0f;
constexpr float kDefaultTargetZ = 2.0f;

class ArmCmd : public rclcpp::Node {
 public:
  ArmCmd();
  ~ArmCmd() = default;

 private:
  void StatusCallback(const arm_msg::msg::ArmStatus::SharedPtr msg);
  void SendDelta(float dx, float dy, float dz);

  rclcpp::Publisher<arm_msg::msg::Arm>::SharedPtr arm_cmd_pub_;
  rclcpp::Subscription<arm_msg::msg::ArmStatus>::SharedPtr arm_status_sub_;

  float cur_x_, cur_y_, cur_z_;
  float target_x_, target_y_, target_z_;
  bool cmd_sent_;
};

#endif  // ARM_CMD_ARM_CMD_H_
