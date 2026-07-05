# LeArm 上位机串口通信 — 坐标控制数据帧格式与调用链路

## 1. 系统架构概览

工程使用 3 路 USART，各司其职：

| USART | 波特率 | 引脚 | 用途 | 接收方式 |
|-------|--------|------|------|----------|
| USART1 | 9600 | PA9(TX) / PA10(RX) | 上位机/蓝牙通信 | DMA + IDLE 中断 |
| USART2 | 115200 | PA2(TX) / PA3(RX) | 总线舵机通信 | 裸中断 (TXE/TC/RXNE) |
| USART3 | 9600 | PB10(TX) / PB11(RX) | PS2 无线手柄 | DMA + IDLE 中断 |

坐标控制数据流方向：**上位机 → USART1 → 协议解析 → 逆运动学解算 → USART2 → 总线舵机**

---

## 2. 上位机通信协议 (USART1)

### 2.1 帧格式

```
+--------+--------+----------+-----+-------------------+
| 0x55   | 0x55   | data_len | cmd | payload           |
+--------+--------+----------+-----+-------------------+
  1 byte   1 byte   1 byte    1 byte   (data_len - 2) bytes
```

| 字段 | 长度 | 说明 |
|------|------|------|
| header_1 | 1 | 固定 0x55 |
| header_2 | 1 | 固定 0x55 |
| data_len | 1 | 总 payload 长度 = cmd(1) + 实际数据，最小值为 2 |
| cmd | 1 | 命令码，见下表 |
| buffer[] | N | 实际数据，data_len - 2 字节 |

**定义位置:** `Hiwonder/Portings/Inc/app_porting.h:8,42-55`

### 2.2 命令码

```c
CMD_VERSION_QUERY       = 1   // 查询版本
CMD_SERVO_OFFSET_READ   = 2   // 读取舵机偏差
CMD_MULT_SERVO_MOVE     = 3   // 多舵机控制
CMD_COORDINATE_SET      = 4   // ★ 坐标控制
CMD_ACTION_GROUP_RUN    = 6   // 运行动作组
CMD_FULL_ACTION_STOP    = 7   // 停止动作组
CMD_FULL_ACTION_ERASE   = 8   // 擦除动作组
CMD_CHASSIS_CONTROL     = 9   // 底盘控制
CMD_SERVO_OFFSET_SET    = 10  // 设置舵机偏差
CMD_SERVO_OFFSET_DOWNLOAD = 11 // 保存舵机偏差
CMD_SERVOS_RESET        = 12  // 舵机复位
CMD_ANGLE_BACK_READING  = 13  // 舵机角度回读
CMD_ACTION_DOWNLOAD     = 25  // 下载动作组
```

**定义位置:** `Hiwonder/Portings/Inc/app_porting.h:24-38`

### 2.3 坐标控制帧 (CMD_COORDINATE_SET = 4)

**完整的发送帧数据：**

```
55 55 05 04 ΔX ΔY ΔZ
│  │  │  │  │  │  └── Z 轴增量 (int8_t)
│  │  │  │  │  └───── Y 轴增量 (int8_t)
│  │  │  │  └──────── X 轴增量 (int8_t)
│  │  │  └─────────── 命令码 = 4 (坐标控制)
│  │  └────────────── data_len = 5 (cmd 1字节 + payload 4字节... 实际这里 1+3=4?)
│  └───────────────── 帧头2
└──────────────────── 帧头1
```

**注意:** data_len = 2 + 实际数据长度，这里实际数据是 3 字节 (ΔX, ΔY, ΔZ)，所以 data_len = 2 + 3 = 5。

> 实际上 `app_porting.c:289-291` 只使用了 `buffer[0]`, `buffer[1]`, `buffer[2]`，data_len 中 `cmd` 占 1 字节，所以 data_len - 2 = 3 字节有效载荷。

**增量换算：**

```c
x_value += ((float)(int8_t)app.packet.buffer[0]) / 10.0f;  // 步进 0.1 cm
y_value += ((float)(int8_t)app.packet.buffer[1]) / 10.0f;
z_value += ((float)(int8_t)app.packet.buffer[2]) / 10.0f;
```

- 每个增量是 `int8_t`，取值范围 -128 ~ 127
- 除以 10.0 得到厘米单位，精度 0.1cm
- 坐标从默认值开始累积：`DEFAULT_X=15.0, DEFAULT_Y=0.0, DEFAULT_Z=2.0`

**代码位置:** `Hiwonder/Portings/Src/app_porting.c:287-307`

### 2.4 坐标范围限制

| 轴 | 最小值 | 最大值 | 默认值 |
|----|--------|--------|--------|
| X | 10.0 cm | 20.0 cm | 15.0 cm |
| Y | -10.0 cm | 10.0 cm | 0.0 cm |
| Z | 0.0 cm | 25.0 cm | 2.0 cm |

**定义位置:** `Hiwonder/Arm/Inc/robot_arm.h:20-29`

### 2.5 命令码 CMD_MULT_SERVO_MOVE (3) — 多舵机直接控制帧

附这个命令是因为它也常用于上位机调试，与坐标控制是并行链路：

```
55 55 len 03 count time_l time_h [id duty_l duty_h]...
                         │    │       │
                         │    │       └── 每3字节一组 (id, duty_l, duty_h)
                         │    └── 运行时间高8位
                         └── 运行时间低8位
```

---

## 3. 完整调用链路

### 3.1 链路总览

```
上位机发送 55 55 05 04 ΔX ΔY ΔZ
        │
        ▼
  USART1 RX (PA10) ─── DMA1_Channel5 ─── rx_dma_buf[64]
        │
        ▼ (总线空闲 → IDLE 中断 → HAL 回调)
  packet_dma_receive_event_callback()     [app_porting.c:215]
    ├── LED1 翻转 (指示灯)
    ├── lwrb_write() 写入环形缓冲区
    └── HAL_UARTEx_ReceiveToIdle_DMA() 重新装载 DMA
        │
        ▼ (主循环轮询)
  while(1) { app_handler(); }            [main.c:139-143]
        │
        ▼
  app_handler()                           [app_porting.c:246]
        │
        ▼
  unpack(&app)                            [app_porting.c:81]
    ├── 状态机逐字节解包
    ├── 验证 0x55 0x55 帧头
    ├── 提取 data_len, cmd, buffer[]
    └── 将 cmd 映射到 app.status
        │
        ▼
  switch(app.status)
    case CMD_COORDINATE_SET:              [app_porting.c:287]
      ├── ΔX/ΔY/ΔZ 累加到 x_value/y_value/z_value
      └── 调用 robot_arm_coordinate_set()
              │
              ▼
          robot_arm_coordinate_set()      [robot_arm.c:60]
            ├── 构造 VectorObjectTypeDef (x,y,z)
            ├── set_pitch_range() ×2     (LeArm.lib 运动学库)
            ├── 选取最接近目标 pitch 的解
            └── 调用 theta2servo()
                    │
                    ▼
                theta2servo()             [robot_arm.c:16]
                  ├── 角度 = 500 + SERIAL_ANGLE_FACTOR × 关节角
                  │   (SERIAL_ANGLE_FACTOR = 4.1667 = 1000/240)
                  ├── 映射: knot[0]→ID6, knot[1]→ID5, knot[2]→ID4, knot[3]→ID3
                  └── serial_servo_set_position() ×4
                          │
                          ▼
                      serial_servo_set_position()  [serial_servo.c:144]
                        ├── 构造 SerialServoCmdTypeDef 帧
                        ├── cmd = SERIAL_SERVO_MOVE_TIME_WRITE (1)
                        ├── 填充 position, duration 到 args[]
                        └── serial_write_and_read()
                                │
                                ▼
                            serial_write_and_read() [serial_servo.c:81]
                              ├── BUS_EN=0 (写模式)
                              ├── 复制帧到 tx_frame
                              ├── 使能 USART2 TXE + TC 中断
                              └── USART2_IRQHandler 接管
                                      │
                                      ▼
                                  USART2_IRQHandler   [stm32f1xx_it.c:367]
                                    TXE: 逐字节发送
                                    TC:  发送完成, BUS_EN=1 (回到读模式)
```

### 3.2 详细步骤

#### 第 1 步：DMA 接收 + IDLE 中断

**文件:** `Hiwonder/Portings/Src/app_porting.c`

初始化时 `app_init()` (line 221) → `packet_start_receive()` (line 73) → `HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, 64)` 启动 DMA 接收。

当上位机发送一帧数据，USART1 通过 DMA1_Channel5 将数据存入 `rx_dma_buf[64]`。帧发送完毕后总线空闲，产生 IDLE 中断 → HAL 触发 `HAL_UARTEx_RxEventCallback` → 调用注册的回调 `packet_dma_receive_event_callback()` (line 215)：

```c
static void packet_dma_receive_event_callback(UART_HandleTypeDef* huart, uint16_t length) {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);       // 指示灯翻转
    lwrb_write(&app.rb, rx_dma_buf, length);            // 写入环形缓冲区
    app.receive_data(rx_dma_buf, sizeof(rx_dma_buf));   // 重新装载 DMA
}
```

#### 第 2 步：主循环解包

**文件:** `Core/Src/main.c:139-143`

```c
while (1) {
    app_handler();
}
```

**函数:** `app_handler()` → `unpack(&app)` (`Hiwonder/Portings/Src/app_porting.c:81`)

解包状态机：`PACKET_HEADER_1 → PACKET_HEADER_2 → PACKET_DATA_LENGTH → PACKET_FUNCTION → PACKET_DATA`

- 验证 2 字节帧头均为 0x55
- 检查 data_len >= 2
- 检查 cmd < CMD_FUNC_NULL
- 读取 data_len - 2 字节 payload 到 buffer[]
- 将 cmd 转换为 app.status（如 cmd=4 → CMD_COORDINATE_SET）

#### 第 3 步：坐标控制处理

**函数:** `app_handler()`, case CMD_COORDINATE_SET (`app_porting.c:287-307`)

```c
case CMD_COORDINATE_SET:
    x_value += ((float)(int8_t)app.packet.buffer[0]) / 10.0f;
    y_value += ((float)(int8_t)app.packet.buffer[1]) / 10.0f;
    z_value += ((float)(int8_t)app.packet.buffer[2]) / 10.0f;

    if (!robot_arm_coordinate_set(x_value, y_value, z_value,
                                  -40,      // target pitch
                                  -90.0f,    // min pitch
                                  10.0f,     // max pitch
                                  60)) {     // time in ms
        // 无解：回滚坐标，蜂鸣器告警
        stop_flag = 1;
        x_value = stop_x_value;
        y_value = stop_y_value;
        z_value = stop_z_value;
    }
    app.status = CMD_FUNC_NULL;  // 单次执行，立即重置
```

#### 第 4 步：逆运动学解算

**文件:** `Hiwonder/Arm/src/robot_arm.c:60-132`

`robot_arm_coordinate_set()`:
1. 构造 `VectorObjectTypeDef vector = {target_x, target_y, target_z}`
2. 调用 `set_pitch_range(&kinematics_result1, &vector, pitch, min_pitch)` — 以最小俯仰角约束求解
3. 调用 `set_pitch_range(&kinematics_result2, &vector, pitch, max_pitch)` — 以最大俯仰角约束求解
4. 选取 alpha 最接近目标 pitch 的解
5. 将解存入全局 `kinematics` 结构体
6. 调用 `theta2servo(&kinematics, time)`

> `set_pitch_range()` 来自闭源运动学库 `LeArm.lib`。

#### 第 5 步：关节角 → 舵机脉宽

**函数:** `theta2servo()` (`robot_arm.c:16-46`), SERVO_TYPE=2 分支

```c
target_angle[0] = self->knot[0].theta;              // 底部旋转关节
target_angle[1] = 90.0f - self->knot[1].theta;      // 肩关节
target_angle[2] = self->knot[2].theta;               // 肘关节
target_angle[3] = self->knot[3].theta;               // 腕关节

for (uint8_t i = 0; i < 4; i++) {
    serial_servo_set_position(&serial_servo_controller,
                              6 - i,                              // 舵机ID: 6,5,4,3
                              500 + (int)(4.1667f * target_angle[i]), // 脉宽
                              time);                              // 运行时间 ms
}
```

关节 → 舵机 ID 映射：

| knot 索引 | 关节 | 舵机 ID | 角度范围 | 脉宽范围 |
|-----------|------|---------|----------|----------|
| knot[0] | 底座旋转 | 6 | -90° ~ 150° | 125 ~ 875 |
| knot[1] | 肩关节 | 5 | -90° ~ 150° | 125 ~ 875 |
| knot[2] | 肘关节 | 4 | -90° ~ 150° | 125 ~ 875 |
| knot[3] | 腕关节 | 3 | -90° ~ 150° | 125 ~ 875 |

`SERIAL_ANGLE_FACTOR = 1000/240 = 4.1667` (将 240° 范围映射到 1000 脉宽范围)

#### 第 6 步：构造总线舵机帧

**文件:** `Hiwonder/Peripherals/Src/serial_servo.c:144-155`

```c
void serial_servo_set_position(..., uint8_t servo_id, int position, uint16_t duration) {
    SerialServoCmdTypeDef frame;
    position = position > 1000 ? 1000 : position;   // 限幅
    cmd_frame_init(&frame, servo_id, SERIAL_SERVO_MOVE_TIME_WRITE);
    frame.elements.args[0] = GET_LOW_BYTE(position);
    frame.elements.args[1] = GET_HIGH_BYTE(position);
    frame.elements.args[2] = GET_LOW_BYTE(duration);
    frame.elements.args[3] = GET_HIGH_BYTE(duration);
    cmd_frame_complete(&frame, 4);   // 填充 length 和 checksum
    self->serial_write_and_read(self, &frame, true);  // tx_only=true
}
```

#### 第 7 步：USART2 中断驱动发送

**函数:** `serial_write_and_read()` (`serial_servo.c:81-117`)

```c
case SERIAL_SERVO_WRITE_DATA_READY:
    self->write_pin(0);                              // BUS_EN=0, 进入写模式
    memcpy(&self->tx_frame, frame, sizeof(...));     // 复制发送帧
    self->tx_byte_index = 0;
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);      // 打开发送中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);       // 打开发送完成中断
```

**中断处理:** `USART2_IRQHandler()` (`Core/Src/stm32f1xx_it.c:367-416`)

```c
// TXE 中断 — 逐字节发送
if (UART_FLAG_TXE) {
    if (tx_byte_index < tx_frame.elements.length + 3) {
        huart2.Instance->DR = ((uint8_t*)&tx_frame)[tx_byte_index++];
    } else {
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);   // 发完，关 TXE
    }
}

// TC 中断 — 发送完成
if (UART_FLAG_TC) {
    if (tx_only) {
        it_state = SERIAL_SERVO_WRITE_DATA_READY;       // 完成，等待下次调用
    } else {
        write_pin(1);                                    // BUS_EN=1, 读模式
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);     // 等舵机应答
    }
}

// RXNE 中断 — 接收舵机应答 (仅 read 指令)
if (UART_FLAG_RXNE) {
    if (0 == serial_servo_rx_handler(&controller, DR)) {
        it_state = SERIAL_SERVO_READ_DATA_FINISH;
    }
}
```

---

## 4. 总线舵机通信协议 (USART2)

### 4.1 帧格式

```
+--------+--------+----------+--------+---------+-----------+----------+
| 0x55   | 0x55   | servo_id | length | command | args[0..7]| checksum |
+--------+--------+----------+--------+---------+-----------+----------+
  1 byte   1 byte   1 byte    1 byte   1 byte    0~8 bytes   1 byte
```

| 字段 | 说明 |
|------|------|
| header_1 | 固定 0x55 |
| header_2 | 固定 0x55 |
| servo_id | 目标舵机 ID (1~6) |
| length | args 数量 + 3 |
| command | 命令码 |
| args[] | 参数 (最多 8 字节) |
| checksum | `~(sum(servo_id .. last_arg))` |

**定义位置:** `Hiwonder/Peripherals/Inc/serial_servo.h:41-54`

### 4.2 校验和算法

```c
static inline uint8_t serial_servo_checksum(const uint8_t buf[]) {
    uint16_t temp = 0;
    for (int i = 2; i < buf[3] + 2; ++i)  // 从 servo_id 累加到最后一个 arg
        temp += buf[i];
    return (uint8_t)(~temp);
}
```

**位置:** `serial_servo.h:309-316`

### 4.3 位置设置帧示例 (SERIAL_SERVO_MOVE_TIME_WRITE = 1)

发送给 ID=6 舵机，位置=500，时间=60ms：

```
55 55 06 07 01 F4 01 3C 00 XX
│  │  │  │  │  │  │  │  │  └── checksum
│  │  │  │  │  │  │  │  └──── duration high byte
│  │  │  │  │  │  │  └─────── duration low byte
│  │  │  │  │  │  └────────── position high byte
│  │  │  │  │  └───────────── position low byte (= 0xF4 = 244, 但用 GET_LOW_BYTE(500)=244)
│  │  │  │  └──────────────── command = 1 (MOVE_TIME_WRITE)
│  │  │  └─────────────────── length = 4 args + 3 = 7
│  │  └────────────────────── servo_id = 6
│  └───────────────────────── 帧头2
└──────────────────────────── 帧头1
```

> 注意：`position=500`，`GET_LOW_BYTE(500) = 500 & 0xFF = 244 (0xF4)`，`GET_HIGH_BYTE(500) = 1 (0x01)`，小端序合成 `0x01F4 = 500`。

---

## 5. USART2 半双工方向控制

USART2 通过 GPIO `BUS_EN` 控制 RS-485 收发方向：

```c
static void write_pin(uint8_t new_state) {
    HAL_GPIO_WritePin(BUS_EN_GPIO_Port, BUS_EN_Pin, (GPIO_PinState)new_state);
}
// new_state = 0 → 写模式 (TX)
// new_state = 1 → 读模式 (RX)
```

**位置:** `serial_servo.c:66-70`

- `tx_only=true` 时 (如 set_position)：发送完直接完成，不读应答
- `tx_only=false` 时 (如 read_position)：发送完切换到读模式，等待舵机应答帧

---

## 6. 收发缓冲机制

### 环形缓冲区 (lwrb)

USART1 接收使用轻量级环形缓冲区，解耦 DMA 接收和协议解析：

- 缓冲区大小：256 字节 (`rx_fifo[256]`)
- 写入方：`packet_dma_receive_event_callback()` — DMA 回调
- 读取方：`unpack()` — 主循环 `app_handler()` 中调用

**定义:** `Hiwonder/Middlewares/Inc/lwrb.h`

---

## 7. 关键数据结构

### 7.1 上位机协议包

```c
// app_porting.h:42-55
typedef struct {
    uint8_t packet_header[2];    // {0x55, 0x55}
    uint8_t data_len;            // 有效载荷长度
    uint8_t cmd;                 // 命令码
    uint8_t buffer[28];          // 有效载荷 (MAX_PACKET_LENGTH - 4)
} PacketObjectTypeDef;

// app_porting.h:23-38
typedef enum {
    CMD_VERSION_QUERY = 1,
    CMD_SERVO_OFFSET_READ = 2,
    CMD_MULT_SERVO_MOVE = 3,
    CMD_COORDINATE_SET = 4,     // ★ 坐标控制
    CMD_ACTION_GROUP_RUN = 6,
    // ...
} AppFunctionStatus;
```

### 7.2 总线舵机帧

```c
// serial_servo.h:41-54
typedef struct {
    uint8_t header_1;            // 0x55
    uint8_t header_2;            // 0x55
    union {
        struct {
            uint8_t servo_id;    // 舵机 ID
            uint8_t length;      // args 数量 + 3
            uint8_t command;     // 命令码
            uint8_t args[8];     // 参数 (最多 8 字节)
        } elements;
        uint8_t data_raw[11];    // 原始字节数组
    };
} SerialServoCmdTypeDef;
```

### 7.3 舵机控制器状态

```c
// serial_servo.h:76-89
typedef struct {
    SerialServoITState it_state;           // 中断状态机
    SerialServoRecvState rx_state;         // 接收状态机
    SerialServoCmdTypeDef rx_frame;        // 接收帧
    SerialServoCmdTypeDef tx_frame;        // 发送帧
    uint32_t tx_byte_index;                // 发送字节索引
    bool tx_only;                          // 只发不收？
    void (*write_pin)(uint8_t);            // BUS_EN 控制
    int8_t (*serial_write_and_read)(...);  // 读写接口
} SerialServoControllerTypeDef;
```

### 7.4 运动学结构

```c
// kinematics.h:43-62
typedef struct { float x; float y; float z; } VectorObjectTypeDef;
typedef struct { float rad; float theta; } KnotObjectTypeDef;

struct KinematicsObject {
    float alpha;                  // 俯仰角
    VectorObjectTypeDef vector;   // 目标位置
    KnotObjectTypeDef knot[4];    // 4 个关节角
};
```

---

## 8. 文件索引

| 文件 | 作用 |
|------|------|
| `Core/Src/main.c` | 入口，初始化所有外设，主循环调用 `app_handler()` |
| `Core/Src/usart.c` | USART1/2/3 HAL 层初始化，DMA 配置 |
| `Core/Src/stm32f1xx_it.c` | 中断服务：USART1/2/3 IRQ, DMA1_CH3/CH5 IRQ |
| `Hiwonder/Portings/Inc/app_porting.h` | 上位机协议帧结构、命令枚举 |
| `Hiwonder/Portings/Src/app_porting.c` | DMA 回调、`unpack()` 解包、`app_handler()` 命令分发 |
| `Hiwonder/Peripherals/Inc/serial_servo.h` | 舵机协议帧结构、命令码、`serial_servo_rx_handler()` |
| `Hiwonder/Peripherals/Src/serial_servo.c` | 舵机操作函数、`serial_write_and_read()` |
| `Hiwonder/Arm/Inc/robot_arm.h` | 机械臂 API、坐标/关节常量 |
| `Hiwonder/Arm/src/robot_arm.c` | `robot_arm_coordinate_set()`, `theta2servo()` |
| `Hiwonder/Arm/Inc/kinematics.h` | 运动学类型定义、连杆参数 |
| `Hiwonder/Arm/src/LeArm.lib` | 闭源逆运动学库 (`set_pitch_range()`, `kinematics_init()`) |
| `Hiwonder/global.h` | 全局配置 (`SERVO_TYPE=2`, `CONTROL_MODE=0`) |
| `Hiwonder/Middlewares/Inc/lwrb.h` | 环形缓冲区 |
