#ifndef ARM_BASE_ARM_BASE_H_
#define ARM_BASE_ARM_BASE_H_

#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <serial/serial.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>

#include <cstring>
#include <memory>
#include <string>

#include "arm_msg/msg/arm.hpp"
#include "arm_msg/msg/arm_status.hpp"
#include "arm_msg/msg/gripper_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

// 串口协议帧头
constexpr uint8_t kFrameHeader1 = 0x55;
constexpr uint8_t kFrameHeader2 = 0x55;

// 命令码（与下位机 app_porting.h AppFunctionStatus 对齐）
constexpr uint8_t kCmdVersionQuery    = 1;
constexpr uint8_t kCmdServoOffsetRead = 2;
constexpr uint8_t kCmdMultiServoMove  = 3;
constexpr uint8_t kCmdCoord           = 4;
constexpr uint8_t kCmdActionGroupRun  = 6;
constexpr uint8_t kCmdActionStop      = 7;
constexpr uint8_t kCmdActionErase     = 8;
constexpr uint8_t kCmdServoOffsetSet  = 10;
constexpr uint8_t kCmdServosReset     = 12;
constexpr uint8_t kCmdAngleBackRead   = 13;  // 回读舵机角度

// 协议: data_len = 2 + payload_size(不含帧头)
// 坐标控制帧: [0x55][0x55][data_len=5][CMD=4][X][Y][Z]  共7字节
// data_len=5 即 2 + 3(△X/△Y/△Z)
constexpr int kArmSendSize = 7;
constexpr uint8_t kArmSendLength = 5;

// 夹取结构控制帧 (CMD_MULT_SERVO_MOVE=3)
// [0x55][0x55][data_len=8][CMD=3][count][time_l][time_h][id][duty_l][duty_h]
// 共10字节, count=1(单舵机)
constexpr int kGripperFrameSize = 10;
constexpr uint8_t kGripperDataLen = 8;
constexpr uint8_t kGripperServoId = 1;
constexpr float kGripperMaxAngle = 180.0f;
constexpr float kGripperMaxDuty = 1000.0f;

// 多舵机全关节控制帧 (CMD_MULT_SERVO_MOVE=3, 6舵机)
// [0x55][0x55][data_len=23][CMD=3][count=6][time_l][time_h][id1][duty_l1][duty_h1]...[id6][duty_l6][duty_h6]
// 共25字节
constexpr int kMultiServoFrameSize = 25;
constexpr uint8_t kMultiServoDataLen = 23;
constexpr uint8_t kServoCount = 6;

// 舵机角度→脉宽转换系数 (1000/240)
constexpr float kServoAngleFactor = 4.166666666666667f;
constexpr int kServoCenterPulse = 500;
constexpr int kServoMinPulse = 0;
constexpr int kServoMaxPulse = 1000;

// 夹爪舵机特殊参数 (固件 robot_arm_claw_set)
// pulse = 700 - 5.5556 * open_angle_deg,  200=全开, 700=闭合
constexpr float kGripperPulseFactor = 5.555555555555556f;
constexpr int kGripperMaxPulseVal = 700;
constexpr int kGripperMinPulseVal = 200;

// 默认运动时间 (ms), IK模式建议更长
constexpr int kDefaultMoveTimeMs = 1500;

// 舵机角度回读帧 (CMD_ANGLE_BACK_READING=13)
// 请求: [0x55][0x55][0x02][0x0D]  共4字节
// 回复: [0x55][0x55][0x14][0x0D][id1][dutyL1][dutyH1]...[id6][dutyL6][dutyH6]  共22字节
constexpr int kReadbackReqSize = 4;
constexpr int kReadbackRespSize = 22;
constexpr int kDutyMatchTolerance = 30;  // 脉宽偏差容忍 (±30 = ~7°)

// float(mm) -> int8，精度0.1mm
constexpr float kCoordScale = 10.0f;

// 下位机上电默认坐标（与 robot_arm.h DEFAULT_X/Y/Z 保持一致）
constexpr float kDefaultPosX = 15.0f;
constexpr float kDefaultPosY = 0.0f;
constexpr float kDefaultPosZ = 2.0f;

struct ArmCoord {
  float x;
  float y;
  float z;
};

struct ArmSendData {
  uint8_t tx[kArmSendSize];
};

// 机械臂串口通信节点
// 订阅 "arm_cmd" (arm_msg/msg/Arm)，打包串口帧发给下位机
// 订阅 "gripper_cmd" (arm_msg/msg/GripperCmd)，打包多舵机控制帧
// 发布 "arm_status" (arm_msg/msg/ArmStatus)，当前末端位置供其他节点读取
class ArmBase : public rclcpp::Node {
 public:
  ArmBase();
  ~ArmBase();
  void Control();

 public:
  serial::Serial arm_serial_;

 private:
  void ArmCmdCallback(const arm_msg::msg::Arm::SharedPtr cmd);
  void GripperCmdCallback(const arm_msg::msg::GripperCmd::SharedPtr cmd);
  void GripCmdCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void JointTargetCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void PublishArmStatus();
  int8_t FloatToInt(float val);
  bool QueryVersion();
  int AngleToPulse(float angle_rad, bool is_gripper);
  int GripperAngleToPulse(float grip_angle_rad);
  bool ReadServoAngles(int* duties_out);
  void VerifyAngleReadback();

 private:
  rclcpp::Subscription<arm_msg::msg::Arm>::SharedPtr arm_cmd_sub_;
  rclcpp::Subscription<arm_msg::msg::GripperCmd>::SharedPtr gripper_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr grip_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_target_sub_;
  rclcpp::Publisher<arm_msg::msg::ArmStatus>::SharedPtr arm_status_pub_;

  std::string usart_port_name_;
  int serial_baud_rate_;
  int move_time_ms_;

  ArmSendData send_data_{};
  ArmCoord current_pos_{};
  bool serial_ready_ = false;
  bool ik_mode_ = false;
  int gripper_servo_id_;

  // Gripper state: controlled ONLY via /grip_cmd, NOT via /joint_target.
  // This prevents ik_node's JointState from opening a closed gripper
  // when the arm moves.
  int grip_duty_;  // current gripper servo duty (200=open, 700=closed)

  // Readback verification
  rclcpp::TimerBase::SharedPtr readback_timer_;
  int last_commanded_duties_[kServoCount];

  // Host-side smooth interpolation (compensates for STM32 firmware bug:
  // app_porting.c hardcodes time=20ms in robot_arm_knot_run, ignoring
  // the running_time field from the serial packet).
  // We interpolate on the host: send incremental CMD_MULT_SERVO_MOVE
  // frames every 20ms, each moving a fraction of the total delta.
  int current_duties_[kServoCount];        // last known servo duties
  int interp_target_duties_[kServoCount];  // final target duties
  float interp_duty_inc_[kServoCount];     // duty increment per step
  int interp_steps_remaining_;             // steps left in current interpolation
  rclcpp::TimerBase::SharedPtr interp_timer_;

  void StartInterpolation(const int target_duties[kServoCount], uint16_t time_ms);
  void InterpStepCallback();
  void SendServoFrame(const int duties[kServoCount], uint8_t count, uint16_t time_ms);
};

#endif  // ARM_BASE_ARM_BASE_H_
