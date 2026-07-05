#include "arm_base/arm_base.h"

#include <chrono>
#include <cstring>
#include <map>
#include <thread>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmBase>();
    RCLCPP_INFO(node->get_logger(), "实际波特率: %ld", node->get_parameter("serial_baud_rate").as_int());
    node->Control();
    rclcpp::shutdown();
    return 0;
}

int8_t ArmBase::FloatToInt(float val) {
    float scaled = val * kCoordScale;
    if (scaled > 127.0f) scaled = 127.0f;
    if (scaled < -128.0f) scaled = -128.0f;
    return static_cast<int8_t>(scaled);
}

// 订阅 "arm_cmd"，打包串口帧: [0x55][0x55][5][4][X][Y][Z]，CMD=4 
void ArmBase::ArmCmdCallback(const arm_msg::msg::Arm::SharedPtr cmd) {
    if (!serial_ready_) {
        RCLCPP_WARN(this->get_logger(),
                    "串口通信未就绪（版本查询未通过），丢弃本次指令");
        return;
    }

    send_data_.tx[0] = kFrameHeader1;
    send_data_.tx[1] = kFrameHeader2;
    send_data_.tx[2] = kArmSendLength;
    send_data_.tx[3] = kCmdCoord;
    send_data_.tx[4] = static_cast<uint8_t>(FloatToInt(cmd->x));
    send_data_.tx[5] = static_cast<uint8_t>(FloatToInt(cmd->y));
    send_data_.tx[6] = static_cast<uint8_t>(FloatToInt(cmd->z));

    try {
        arm_serial_.write(send_data_.tx, sizeof(send_data_.tx));
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "串口发送失败: %s", e.what());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "坐标指令已发送");

    // CMD=4 无回复，累加增量更新当前位置后发布
    current_pos_.x += cmd->x;
    current_pos_.y += cmd->y;
    current_pos_.z += cmd->z;
    PublishArmStatus();
}

// 订阅 "gripper_cmd"，打包多舵机控制帧: [0x55][0x55][8][3][1][time_l][time_h][id][duty_l][duty_h]
// CMD_MULT_SERVO_MOVE=3, count=1(单舵机)
void ArmBase::GripperCmdCallback(const arm_msg::msg::GripperCmd::SharedPtr cmd) {
    if (!serial_ready_) {
        RCLCPP_WARN(this->get_logger(),
                    "串口通信未就绪，丢弃夹取指令");
        return;
    }

    // 角度映射到舵机占空比: 0°→0, 180°→1000
    float angle = cmd->angle;
    if (angle < 0.0f) angle = 0.0f;
    if (angle > kGripperMaxAngle) angle = kGripperMaxAngle;
    uint16_t duty = static_cast<uint16_t>(angle / kGripperMaxAngle * kGripperMaxDuty);
    if (duty > 1000) duty = 1000;
    uint16_t time_ms = cmd->time_ms;

    uint8_t frame[kGripperFrameSize];
    frame[0] = kFrameHeader1;
    frame[1] = kFrameHeader2;
    frame[2] = kGripperDataLen;
    frame[3] = kCmdMultiServoMove;
    frame[4] = 1;  // count = 1 (单舵机)
    frame[5] = static_cast<uint8_t>(time_ms & 0xFF);
    frame[6] = static_cast<uint8_t>((time_ms >> 8) & 0xFF);
    frame[7] = static_cast<uint8_t>(gripper_servo_id_);
    frame[8] = static_cast<uint8_t>(duty & 0xFF);
    frame[9] = static_cast<uint8_t>((duty >> 8) & 0xFF);

    try {
        arm_serial_.write(frame, sizeof(frame));
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "夹取指令发送失败: %s", e.what());
        return;
    }

    RCLCPP_INFO(this->get_logger(),
                "夹取指令已发送: angle=%.1f° duty=%d time=%dms servo_id=%d",
                cmd->angle, duty, time_ms, gripper_servo_id_);
}

// 角度(弧度) → 舵机脉宽(0-1000), 非夹爪
int ArmBase::AngleToPulse(float angle_rad, bool is_gripper) {
    float angle_deg = angle_rad * 180.0f / M_PI;
    if (is_gripper) {
        // 夹爪: 0°=闭合→700, 90°=全开→200
        int pulse = static_cast<int>(kGripperMaxPulseVal - kGripperPulseFactor * angle_deg);
        if (pulse < kGripperMinPulseVal) pulse = kGripperMinPulseVal;
        if (pulse > kGripperMaxPulseVal) pulse = kGripperMaxPulseVal;
        return pulse;
    }
    int pulse = kServoCenterPulse + static_cast<int>(angle_deg * kServoAngleFactor);
    if (pulse < kServoMinPulse) pulse = kServoMinPulse;
    if (pulse > kServoMaxPulse) pulse = kServoMaxPulse;
    return pulse;
}

int ArmBase::GripperAngleToPulse(float grip_angle_rad) {
    return AngleToPulse(grip_angle_rad, true);
}

// 订阅 /joint_target (sensor_msgs/JointState)
// 将关节角度转为舵机脉宽，通过 CMD_MULT_SERVO_MOVE 直发下位机
// 绕过 STM32 端闭源 LeArm.lib IK，消除坐标累加漂移
//
// 支持部分关节更新: 未在 msg 中指定的关节保持当前值不变。
// 支持平滑插值: 当 move_time_ms > 20 时，在主机侧分步发送增量帧，
// 补偿 STM32 固件 app_porting.c 里 robot_arm_knot_run 硬编码 time=20 的 bug。
void ArmBase::JointTargetCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!serial_ready_) {
        RCLCPP_WARN(this->get_logger(),
                    "串口通信未就绪，丢弃关节指令");
        return;
    }

    // 关节名 → 舵机ID 映射
    static const std::map<std::string, uint8_t> kJointServoMap = {
        {"shoulder_pan", 6},
        {"shoulder_lift", 5},
        {"elbow", 4},
        {"wrist_flex", 3},
        {"wrist_roll", 2},
        {"grip_left", 1},
    };

    // 从当前已知值开始，只更新 msg 中指定的关节
    int target_duties[kServoCount];
    for (int i = 0; i < kServoCount; i++) {
        target_duties[i] = current_duties_[i];
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
        auto it = kJointServoMap.find(msg->name[i]);
        if (it == kJointServoMap.end()) continue;
        uint8_t servo_id = it->second;
        // grip_left (servo 1) is controlled exclusively via /grip_cmd,
        // NOT via /joint_target. This prevents ik_node's JointState
        // from opening a closed gripper when the arm moves.
        if (servo_id == 1) continue;
        float angle_rad = msg->position[i];

        if (servo_id == 5) {
            // 肩关节: theta2servo 翻转
            float flipped_rad = static_cast<float>(M_PI_2) - angle_rad;
            target_duties[servo_id - 1] = AngleToPulse(flipped_rad, false);
        } else {
            target_duties[servo_id - 1] = AngleToPulse(angle_rad, false);
        }
    }

    uint16_t time_ms = static_cast<uint16_t>(move_time_ms_);

    // 检查是否有变化
    bool changed = false;
    for (int i = 0; i < kServoCount; i++) {
        if (target_duties[i] != current_duties_[i]) {
            changed = true;
            break;
        }
    }
    if (!changed) return;

    RCLCPP_INFO(this->get_logger(),
                "关节指令: target[1-6]=[%d,%d,%d,%d,%d,%d] time=%dms cur=[%d,%d,%d,%d,%d,%d]",
                target_duties[0], target_duties[1], target_duties[2],
                target_duties[3], target_duties[4], target_duties[5], time_ms,
                current_duties_[0], current_duties_[1], current_duties_[2],
                current_duties_[3], current_duties_[4], current_duties_[5]);

    // 平滑插值: STM32 固件忽略了 time 参数（硬编码 20ms），
    // 我们在这里把大范围移动拆成多个 20ms 微步，逐帧发送。
    bool first_command = true;
    for (int i = 0; i < kServoCount; i++) {
        if (current_duties_[i] != 0) { first_command = false; break; }
    }
    if (time_ms > 25 && !first_command) {
        StartInterpolation(target_duties, time_ms);
    } else {
        // 小范围移动（<=25ms），直接发送
        SendServoFrame(target_duties, kServoCount, time_ms);
        memcpy(current_duties_, target_duties, sizeof(current_duties_));
        memcpy(last_commanded_duties_, target_duties, sizeof(last_commanded_duties_));

        if (ik_mode_ && serial_ready_) {
            int delay_ms = time_ms + 100;
            readback_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(delay_ms),
                [this]() {
                    VerifyAngleReadback();
                    readback_timer_->cancel();
                });
        }
    }
}

// 启动主机侧平滑插值
void ArmBase::StartInterpolation(const int target_duties[kServoCount], uint16_t time_ms) {
    constexpr int kInterpStepMs = 20;

    // 取消之前的插值（如果还在进行中）
    if (interp_timer_) {
        interp_timer_->cancel();
        interp_timer_.reset();
    }

    int steps = time_ms / kInterpStepMs;
    if (steps < 1) steps = 1;

    interp_steps_remaining_ = steps;
    for (int i = 0; i < kServoCount; i++) {
        interp_target_duties_[i] = target_duties[i];
        interp_duty_inc_[i] = static_cast<float>(target_duties[i] - current_duties_[i]) / steps;
    }
    // Gripper (servo 1) is controlled independently via /grip_cmd.
    // Don't interpolate it — lock to grip_duty_.
    interp_target_duties_[0] = grip_duty_;
    interp_duty_inc_[0] = 0.0f;

    // 每 20ms 发送一个微步
    interp_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(kInterpStepMs),
        [this]() { InterpStepCallback(); });

    RCLCPP_INFO(this->get_logger(),
                "插值启动: %d steps × %dms, total=%dms", steps, kInterpStepMs, time_ms);
}

// 插值步进回调 (每 20ms 触发一次)
void ArmBase::InterpStepCallback() {
    if (interp_steps_remaining_ <= 0) {
        interp_timer_->cancel();
        interp_timer_.reset();

        // 最后一步: 确保精确到达目标
        SendServoFrame(interp_target_duties_, kServoCount, 20);
        memcpy(current_duties_, interp_target_duties_, sizeof(current_duties_));
        memcpy(last_commanded_duties_, interp_target_duties_, sizeof(last_commanded_duties_));

        RCLCPP_INFO(this->get_logger(), "插值完成: [%d,%d,%d,%d,%d,%d]",
                    interp_target_duties_[0], interp_target_duties_[1],
                    interp_target_duties_[2], interp_target_duties_[3],
                    interp_target_duties_[4], interp_target_duties_[5]);

        // 触发回读验证
        if (ik_mode_ && serial_ready_) {
            int delay_ms = 120;  // 20ms + 100ms 余量
            readback_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(delay_ms),
                [this]() {
                    VerifyAngleReadback();
                    readback_timer_->cancel();
                });
        }
        return;
    }

    // 计算当前步骤的中间值
    int step_duties[kServoCount];
    for (int i = 0; i < kServoCount; i++) {
        float mid = interp_target_duties_[i]
                    - interp_duty_inc_[i] * (interp_steps_remaining_ - 1);
        step_duties[i] = static_cast<int>(mid);
        if (step_duties[i] < 0) step_duties[i] = 0;
        if (step_duties[i] > 1000) step_duties[i] = 1000;
        // 更新当前已知值
        current_duties_[i] = step_duties[i];
    }

    // Override servo 1 (gripper) with the dedicated grip state.
    // Gripper is not interpolated — it stays at the last /grip_cmd value.
    step_duties[0] = grip_duty_;
    current_duties_[0] = grip_duty_;

    SendServoFrame(step_duties, kServoCount, 20);
    interp_steps_remaining_--;
}

// /grip_cmd 回调: 夹爪独立控制，不受 ik_node 的 /joint_target 影响
void ArmBase::GripCmdCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    if (!serial_ready_) return;

    float angle_rad = static_cast<float>(msg->data);
    int new_duty = GripperAngleToPulse(angle_rad);
    if (new_duty == grip_duty_) return;

    int old_duty = grip_duty_;
    grip_duty_ = new_duty;
    current_duties_[0] = grip_duty_;

    // Send gripper-only frame: only servo 1 changes
    int duties[kServoCount];
    for (int i = 0; i < kServoCount; i++) {
        duties[i] = current_duties_[i];
    }
    duties[0] = grip_duty_;

    // Cancel any running arm interpolation before sending gripper frame
    // to avoid interleaving partial arm frames with the gripper command.
    if (interp_timer_) {
        interp_timer_->cancel();
        interp_timer_.reset();
        interp_steps_remaining_ = 0;
    }

    SendServoFrame(duties, kServoCount, 20);

    RCLCPP_INFO(this->get_logger(),
                "夹爪: %.1f° → duty %d (was %d)",
                angle_rad * 180.0 / M_PI, grip_duty_, old_duty);
}

// 发送多舵机控制帧到 STM32
void ArmBase::SendServoFrame(const int duties[kServoCount], uint8_t count, uint16_t time_ms) {
    // STM32 协议: frame = [0x55][0x55][data_len][cmd][payload...]
    // data_len = 2 + payload_size (cmd 占 1 字节 + payload)
    // payload = 1(count) + 2(time) + count*3(id+duty)
    // 总 frame = 2(headers) + data_len
    // 例: count=6 → data_len=5+18=23, frame=25 字节
    uint8_t data_len = 5 + count * 3;
    int frame_size = 2 + data_len;
    uint8_t frame[32];  // 6 舵机时 25 字节

    frame[0] = kFrameHeader1;
    frame[1] = kFrameHeader2;
    frame[2] = data_len;
    frame[3] = kCmdMultiServoMove;
    frame[4] = count;
    frame[5] = static_cast<uint8_t>(time_ms & 0xFF);
    frame[6] = static_cast<uint8_t>((time_ms >> 8) & 0xFF);

    int offset = 7;
    for (uint8_t id = 1; id <= count; ++id) {
        int duty = duties[id - 1];
        if (duty < 0) duty = 0;
        if (duty > 1000) duty = 1000;
        frame[offset]     = id;
        frame[offset + 1] = static_cast<uint8_t>(duty & 0xFF);
        frame[offset + 2] = static_cast<uint8_t>((duty >> 8) & 0xFF);
        offset += 3;
    }

    try {
        arm_serial_.write(frame, frame_size);
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "关节指令发送失败: %s", e.what());
    }
}

// 回读下位机6个舵机的当前脉宽 (CMD_ANGLE_BACK_READING=13)
// 下发 [55 55 02 0D], 下位机回复22字节
bool ArmBase::ReadServoAngles(int* duties_out) {
    if (!arm_serial_.isOpen()) return false;

    uint8_t tx[kReadbackReqSize] = {kFrameHeader1, kFrameHeader2, 0x02,
                                     kCmdAngleBackRead};
    uint8_t rx[kReadbackRespSize] = {0};

    try {
        arm_serial_.flushInput();
        arm_serial_.write(tx, sizeof(tx));

        size_t n = arm_serial_.read(rx, sizeof(rx));
        if (n != kReadbackRespSize) {
            RCLCPP_WARN(this->get_logger(),
                        "回读失败: 期望 %d 字节, 收到 %zu", kReadbackRespSize, n);
            return false;
        }
    } catch (serial::IOException& e) {
        RCLCPP_WARN(this->get_logger(), "回读异常: %s", e.what());
        return false;
    }

    if (rx[0] != kFrameHeader1 || rx[1] != kFrameHeader2) {
        RCLCPP_WARN(this->get_logger(), "回读帧头错误: [%02X %02X]", rx[0], rx[1]);
        return false;
    }

    // 解析: rx[4]=id1, rx[5]=dutyL1, rx[6]=dutyH1, ... ×6
    for (int i = 0; i < kServoCount; i++) {
        int offset = 4 + i * 3;
        uint8_t servo_id = rx[offset];
        uint16_t duty = rx[offset + 1] | (static_cast<uint16_t>(rx[offset + 2]) << 8);
        if (servo_id >= 1 && servo_id <= kServoCount) {
            duties_out[servo_id - 1] = static_cast<int>(duty);
        }
    }
    return true;
}

// 定时器回调: 回读并对比下位机实际角度与上次指令
void ArmBase::VerifyAngleReadback() {
    int actual[kServoCount] = {0};
    if (!ReadServoAngles(actual)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "舵机回读失败, 无法验证");
        return;
    }

    int mismatch = 0;
    for (int i = 0; i < kServoCount; i++) {
        int cmd = last_commanded_duties_[i];
        if (cmd <= 0) continue;  // 未指令的舵机跳过
        int diff = std::abs(actual[i] - cmd);
        if (diff > kDutyMatchTolerance) {
            mismatch++;
            RCLCPP_WARN(this->get_logger(),
                        "舵机%d 偏差! 指令=%d 实际=%d (差%d)", i + 1, cmd, actual[i], diff);
        }
    }

    if (mismatch == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "闭环验证通过: 6舵机均在容差±%d内 [%d,%d,%d,%d,%d,%d]",
                    kDutyMatchTolerance, actual[0], actual[1], actual[2],
                    actual[3], actual[4], actual[5]);
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "闭环验证: %d/6 舵机超差, 实际=[%d,%d,%d,%d,%d,%d]",
                    mismatch, actual[0], actual[1], actual[2],
                    actual[3], actual[4], actual[5]);
    }
}

void ArmBase::PublishArmStatus() {
    auto msg = arm_msg::msg::ArmStatus();
    msg.header.stamp = this->now();
    msg.x = current_pos_.x;
    msg.y = current_pos_.y;
    msg.z = current_pos_.z;
    msg.status = 1;
    arm_status_pub_->publish(msg);
}

bool ArmBase::QueryVersion() {
    // 发送版本查询帧: [0x55][0x55][0x02][0x01]
    // 下位机回复:     [0x55][0x55][0x04][0x01][servo_type][software_ver]
    constexpr uint8_t kQueryFrameSize = 4;
    constexpr uint8_t kRespFrameSize = 6;
    uint8_t tx[kQueryFrameSize] = {kFrameHeader1, kFrameHeader2, 0x02,
                                   kCmdVersionQuery};
    uint8_t rx[kRespFrameSize] = {0};

    if (!arm_serial_.isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "串口未打开，跳过版本查询");
        return false;
    }

    try {
        // 清空接收缓冲区
        arm_serial_.flushInput();

        arm_serial_.write(tx, sizeof(tx));
        RCLCPP_INFO(this->get_logger(), "已发送版本查询: 55 55 02 01");

        // 读取回复，超时 500ms
        size_t n = arm_serial_.read(rx, sizeof(rx));
        if (n != kRespFrameSize) {
            RCLCPP_ERROR(this->get_logger(),
                         "版本查询回复长度错误: 期望 %d, 实际 %zu",
                         kRespFrameSize, n);
            return false;
        }
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "版本查询失败: %s", e.what());
        return false;
    }

    // 校验回复帧
    if (rx[0] != kFrameHeader1 || rx[1] != kFrameHeader2) {
        RCLCPP_ERROR(this->get_logger(),
                     "版本查询回复帧头错误: [%02X %02X]", rx[0], rx[1]);
        return false;
    }
    if (rx[2] != 0x04 || rx[3] != kCmdVersionQuery) {
        RCLCPP_ERROR(this->get_logger(),
                     "版本查询回复格式错误: data_len=%02X cmd=%02X", rx[2],
                     rx[3]);
        return false;
    }

    uint8_t servo_type = rx[4];
    uint8_t sw_version = rx[5];
    RCLCPP_INFO(this->get_logger(),
                "版本查询成功: 舵机类型=%d, 软件版本=%d", servo_type, sw_version);
    return true;
}

void ArmBase::Control() {
    // 启动时先查询版本，确认串口通信正常
    if (arm_serial_.isOpen()) {
        // 重试机制：下位机启动时间不确定，最多重试5次，每次间隔500ms
        constexpr int kMaxRetries = 5;
        for (int i = 0; i < kMaxRetries; ++i) {
            serial_ready_ = QueryVersion();
            if (serial_ready_) {
                RCLCPP_INFO(this->get_logger(), "串口通信握手成功，开始接收指令");
                PublishArmStatus();
                break;
            }
            if (i < kMaxRetries - 1) {
                RCLCPP_WARN(this->get_logger(),
                            "版本查询失败(%d/%d)，500ms后重试...", i + 1, kMaxRetries);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        if (!serial_ready_) {
            RCLCPP_ERROR(this->get_logger(),
                         "串口通信握手失败（已重试%d次），坐标指令将被丢弃。"
                         "请检查接线和下位机状态",
                         kMaxRetries);
        }
    }

    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
}

ArmBase::ArmBase() : rclcpp::Node("arm_base"), serial_baud_rate_(9600), gripper_servo_id_(kGripperServoId), interp_steps_remaining_(0), grip_duty_(kGripperMinPulseVal) {
    memset(&send_data_, 0, sizeof(send_data_));
    memset(last_commanded_duties_, 0, sizeof(last_commanded_duties_));
    memset(current_duties_, 0, sizeof(current_duties_));
    current_duties_[0] = grip_duty_;  // gripper starts open (200)
    memset(interp_target_duties_, 0, sizeof(interp_target_duties_));
    memset(interp_duty_inc_, 0, sizeof(interp_duty_inc_));
    // 初始位置必须与下位机上电默认坐标一致，否则增量计算会偏移
    current_pos_.x = kDefaultPosX;
    current_pos_.y = kDefaultPosY;
    current_pos_.z = kDefaultPosZ;

    this->declare_parameter<std::string>("usart_port_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_baud_rate", 9600);
    this->declare_parameter<int>("gripper_servo_id", kGripperServoId);
    this->declare_parameter<bool>("ik_mode", false);
    this->declare_parameter<int>("move_time_ms", kDefaultMoveTimeMs);
    this->get_parameter("usart_port_name", usart_port_name_);
    this->get_parameter("serial_baud_rate", serial_baud_rate_);
    this->get_parameter("gripper_servo_id", gripper_servo_id_);
    ik_mode_ = this->get_parameter("ik_mode").as_bool();
    this->get_parameter("move_time_ms", move_time_ms_);

    // IK 模式: 只启用 /joint_target 路径，禁用旧的增量坐标 + 夹爪指令
    // 避免新旧两条路径同时发串口指令导致 STM32 冲突
    if (ik_mode_) {
        RCLCPP_INFO(this->get_logger(),
                    "IK 模式已启用: 仅接收 /joint_target (绝对舵机角度), "
                    "move_time=%dms", move_time_ms_);
    } else {
        RCLCPP_INFO(this->get_logger(),
                    "传统坐标模式: 接收 /arm_cmd + /gripper_cmd (增量坐标)");
        arm_cmd_sub_ = create_subscription<arm_msg::msg::Arm>(
            "arm_cmd", 10,
            [this](const arm_msg::msg::Arm::SharedPtr cmd) { ArmCmdCallback(cmd); });

        gripper_cmd_sub_ = create_subscription<arm_msg::msg::GripperCmd>(
            "gripper_cmd", 10,
            [this](const arm_msg::msg::GripperCmd::SharedPtr cmd) {
              GripperCmdCallback(cmd);
            });
    }

    // /joint_target 始终启用（IK 模式的手臂关节输入通道）
    // 注意: grip_left 关节被忽略，夹爪由独立的 /grip_cmd 控制，
    // 避免 ik_node 的 JointState 覆盖 task 节点设置的夹爪状态。
    joint_target_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_target", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          JointTargetCallback(msg);
        });

    // /grip_cmd: 夹爪独立控制通道 (Float64, 弧度)
    // 只有 task 节点通过此 topic 控制夹爪，ik_node 不会影响夹爪状态。
    grip_cmd_sub_ = create_subscription<std_msgs::msg::Float64>(
        "grip_cmd", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          GripCmdCallback(msg);
        });

    // transient_local：晚订阅的节点也能收到最后一条状态，避免启动时序竞争
    arm_status_pub_ = create_publisher<arm_msg::msg::ArmStatus>(
        "arm_status", rclcpp::QoS(1).transient_local());

    try {
        arm_serial_.setPort(usart_port_name_);
        arm_serial_.setBaudrate(serial_baud_rate_);
        serial::Timeout t = serial::Timeout::simpleTimeout(100);
        arm_serial_.setTimeout(t);
        arm_serial_.open();
        // Linux打开串口时DTR和RTS都被拉高。
        // RTS→BOOT0，DTR→RESET。必须先拉低RTS(BOOT0=0)再拉低DTR，
        // 否则STM32复位后会进入ROM bootloader而非运行应用程序。
        arm_serial_.setRTS(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        arm_serial_.setDTR(false);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s: %s",
                     usart_port_name_.c_str(), e.what());
    }

    if (arm_serial_.isOpen())
        RCLCPP_INFO(this->get_logger(), "串口已打开: %s @ %d",
                    usart_port_name_.c_str(), serial_baud_rate_);

    // 不在构造函数里发布初始状态——等版本查询通过后再发布，
    // 避免 arm_cmd 在握手完成前就发送坐标指令
}

ArmBase::~ArmBase() {
    if (arm_serial_.isOpen())
        arm_serial_.close();
    RCLCPP_INFO(this->get_logger(), "arm_base 已关闭");
}
