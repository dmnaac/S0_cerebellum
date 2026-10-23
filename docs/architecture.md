# 四足机器人小脑运动控制架构设计

## 1. 系统总体架构

采用**分层 + 多进程**架构，进程间通过 **LCM** 交换数据，不依赖 ROS。

```
┌─────────────────────────────────────────────────────────────┐
│  应用层：遥控手柄 / 上位机 / 任务规划 / Web 可视化              │
└───────────────────────┬─────────────────────────────────────┘
                        │ user_cmd (LCM)
┌───────────────────────▼─────────────────────────────────────┐
│  command_node：解析输入，生成高层运动指令                       │
│                 输出 vx/vy/yaw_rate + state_name              │
└───────────────────────┬─────────────────────────────────────┘
                        │ user_cmd_t
┌───────────────────────▼─────────────────────────────────────┐
│  rl_policy_node：状态机 + 运动生成                             │
│                 ┌─────────────────────────────────────┐     │
│                 │  StateMachine                       │     │
│                 │  ├─ StaticState: stand/crouch/lie/lock    │
│                 │  └─ GaitState:  crawl/walk/trot/pace/     │
│                 │               bound/gallop                │
│                 └─────────────────────────────────────┘     │
│                 频率：50~100Hz                                │
└──────────────┬────────┴────────────────────────────────────┘
               │ rl_action_t              │ state_status_t
               ▼                          ▼
┌───────────────────────┐    ┌───────────────────────┐
│  motion_control_node  │    │  safety_monitor /     │
│  插值/PD/力控转换      │    │  logger_node / debug  │
│  频率：1kHz            │    └───────────────────────┘
└───────────┬───────────┘
            │ motor_cmd_t
┌───────────▼───────────┐
│  motor_driver_node    │
│  SocketCAN → 12 电机  │
│  频率：1kHz            │
└───────────┬───────────┘
            │ joint_state_t / imu_t
┌───────────▼───────────┐
│  sensor_node          │
│  IMU + 关节反馈聚合    │
└───────────────────────┘
```

---

## 2. 核心进程职责

| 进程 | 输入 | 输出 | 频率 | 核心职责 |
|------|------|------|------|----------|
| `command_node` | 手柄/上位机 | `user_cmd_t` | 50Hz | 解析遥控指令、生成 `state_name`、指令平滑 |
| `sensor_node` | IMU 硬件、关节反馈 | `imu_t`、`robot_state_t` | 1kHz | IMU 采集、数据同步、坐标系转换 |
| `rl_policy_node` | `robot_state_t` + `user_cmd_t` | `rl_action_t`、`state_status_t` | 50~100Hz | 状态机管理、静态姿态/动态步态输出、RL 推理 |
| `motion_control_node` | `rl_action_t` + `JOINT_STATE` | `motor_cmd_t` | 1kHz | 插值到 1kHz、PD 计算、输出限幅 |
| `motor_driver_node` | `motor_cmd_t` | `joint_state_t` | 1kHz | SocketCAN 收发、电机状态机、错误码处理 |
| `safety_monitor` | 所有 LCM 频道 | `safety_status_t` | 1kHz | 限位检查、超时保护、故障响应、E-Stop |
| `logger_node` | 所有 LCM 频道 | 日志文件 | - | 记录时间序列数据用于回放分析 |

---

## 3. LCM 通信频道设计

| 频道名 | 消息 | 发布者 | 订阅者 | 频率 |
|--------|------|--------|--------|------|
| `USER_CMD` | `user_cmd_t` | command_node | rl_policy_node | 50Hz |
| `IMU` | `imu_t` | sensor_node | rl_policy_node, safety_monitor, logger | 200~1000Hz |
| `JOINT_STATE` | `joint_state_t` | motor_driver_node | rl_policy_node, motion_control_node, safety_monitor, logger | 1kHz |
| `ROBOT_STATE` | `robot_state_t` | sensor_node（聚合） | rl_policy_node, safety_monitor, logger | 100Hz |
| `RL_ACTION` | `rl_action_t` | rl_policy_node | motion_control_node, safety_monitor, logger | 50~100Hz |
| `STATE_STATUS` | `state_status_t` | rl_policy_node | safety_monitor, logger, debug_viz | 50~100Hz |
| `MOTOR_CMD` | `motor_cmd_t` | motion_control_node | motor_driver_node, safety_monitor, logger | 1kHz |
| `SAFETY_STATUS` | `safety_status_t` | safety_monitor | 所有节点 | 事件/1kHz |

### 消息定义建议

```cpp
// user_cmd_t.lcm
struct user_cmd_t {
    int64_t utime;
    float vx;          // m/s
    float vy;          // m/s
    float yaw_rate;    // rad/s
    int8_t mode;       // deprecated, use state_name
    string state_name; // e.g., "stand", "trot", "gallop"
};

// imu_t.lcm
struct imu_t {
    int64_t utime;
    float quaternion[4];   // w, x, y, z
    float gyro[3];         // rad/s
    float accel[3];        // m/s^2
};

// joint_state_t.lcm
struct joint_state_t {
    int64_t utime;
    float position[12];    // rad
    float velocity[12];    // rad/s
    float torque[12];      // Nm
    float temperature[12]; // °C
    uint16_t error[12];    // 电机错误码
};

// robot_state_t.lcm
struct robot_state_t {
    int64_t utime;
    imu_t imu;
    joint_state_t joints;
    float previous_action[12];   // offset from default stand
    user_cmd_t cmd;
};

// rl_action_t.lcm
struct rl_action_t {
    int64_t utime;
    float position_offset[12];   // 绝对目标关节位置（状态机已合并 default_stand + offset）
};

// state_status_t.lcm
struct state_status_t {
    int64_t utime;
    string current_state;
    string target_state;
    boolean in_transition;
    float transition_progress;   // 0.0 ~ 1.0
};

// motor_cmd_t.lcm
struct motor_cmd_t {
    int64_t utime;
    float position[12];
    float velocity[12];
    float kp[12];
    float kd[12];
    float torque[12];
};

// safety_status_t.lcm
struct safety_status_t {
    int64_t utime;
    bool e_stop;
    bool comm_timeout;
    uint8_t level;  // 0=normal, 1=warning, 2=fault
    string message;
};
```

---

## 4. 控制时序

以 **1ms 周期**为基准：

```
t=0.0ms   motor_driver_node:  发送 CAN 命令到 12 电机
t=0.2ms   motor_driver_node:  接收 CAN 反馈 -> 发布 JOINT_STATE
t=0.3ms   sensor_node:        读取 IMU -> 发布 IMU
t=0.5ms   sensor_node:        聚合 IMU + JOINT_STATE -> 发布 ROBOT_STATE
t=0.6ms   motion_control_node:订阅 RL_ACTION + JOINT_STATE -> 计算 MOTOR_CMD
t=0.8ms   rl_policy_node:     状态机更新/RL推理 -> 发布 RL_ACTION + STATE_STATUS
t=0.9ms   safety_monitor:     检查限位/超时/故障 -> 发布 SAFETY_STATUS
t=1.0ms   进入下一周期

每 10~20ms (50~100Hz):
          rl_policy_node:     基于 ROBOT_STATE + USER_CMD 更新状态机并发布动作
```

**同步策略**：
- RL 节点取最新一帧 `ROBOT_STATE` 推理，不等待；
- `motion_control_node` 对 `RL_ACTION` 做 **零阶保持 + 线性插值** 到 1kHz；
- 所有消息带 `utime`，便于计算端到端延迟。

---

## 5. RL 策略集成

### 5.1 Observation（典型维度 ~50）

```python
obs = [
    projected_gravity[3],    # 机体坐标系下重力方向
    command[3],              # [vx, vy, yaw_rate]
    angular_velocity[3],     # 机体角速度
    joint_positions[12],     # 关节角度
    joint_velocities[12],    # 关节角速度
    previous_actions[12],    # 上一帧策略输出
]
# 总计约 45 维，可根据策略需求扩展
```

### 5.2 Action

```python
action = [
    joint_position_offset[12]   # 默认站立姿态 + offset
]
# 状态机将 offset 与 default_stand 合并为绝对目标位置输出
# 输出需经过 action_scale、平滑滤波、关节限幅
```

### 5.3 推理引擎选型

| 方案 | 推荐度 | 说明 |
|------|--------|------|
| **ONNX Runtime** | ⭐⭐⭐⭐ | 开发首选，PyTorch 直接导出，CPU 推理足够 50~100Hz |
| **RKNN Toolkit2** | ⭐⭐⭐⭐⭐ | 部署首选，利用 RK3588 NPU（6 TOPS INT8），延迟最低 |
| LibTorch | ⭐⭐⭐ | 体积大、依赖重，不推荐常驻进程 |
| TensorRT | ❌ | RK3588 无 CUDA，不可用 |

**建议路径**：先用 ONNX Runtime 调通，再转 RKNN 利用 NPU。

### 5.4 多步态策略选择

| 方案 | 适用场景 | 说明 |
|------|----------|------|
| **单一多步态策略**（推荐） | 步态差异可控、RK3588 资源有限 | `gait_id` 作为 observation 输入，一个模型覆盖 crawl/walk/trot/pace/bound/gallop |
| **每步态独立策略** | 某步态性能要求极高 | 每个 `GaitState` 配置独立 `policy_name`，状态机切换模型 |
| **混合策略** | 折中方案 | 常见步态共用主策略，特殊步态（bound/gallop）使用专用策略 |

**建议路径**：先用 ONNX Runtime 训练单一多步态策略；若性能不足，再为特定步态独立训练。

---

## 6. 状态机设计

### 6.1 状态分类

| 类型 | 状态 | 控制方式 |
|------|------|----------|
| 静态姿态 | `stand`、`crouch`、`lie`、`lock` | 预设目标位置 + PD 伺服 |
| 动态步态 | `crawl`、`walk`、`trot`、`pace`、`bound`、`gallop` | RL 策略推理输出动作偏移 |

### 6.2 状态切换流程

```
user_cmd_t.state_name ──► StateMachine.requestState()
                              │
                              ▼
                    ┌─────────────────────┐
                    │   检查转移合法性    │
                    │   canTransitionTo() │
                    └──────────┬──────────┘
                               ▼
                    ┌─────────────────────┐
                    │   启动平滑过渡      │
                    │   transition_time   │
                    └──────────┬──────────┘
                               ▼
                    ┌─────────────────────┐
                    │   每帧插值输出      │
                    │   q = lerp(q_prev,  │
                    │        q_target,    │
                    │        smoothstep)  │
                    └──────────┬──────────┘
                               ▼
                         publish RL_ACTION
```

### 6.3 静态状态输出

- 从 `config/states.yaml` 读取 `target_position`
- 输出绝对目标位置：`action = target_position`
- 控制律：`tau = kp * (q_target - q) + kd * (0 - qd)`
- `lock` 状态使用高刚度锁定当前安全姿态

### 6.4 动态状态输出

- 从 `config/states.yaml` 读取 `gait_id` 与 `policy_name`
- 构建 observation（含 `gait_id`）
- 策略输出动作偏移：`action_offset`
- 状态机合并默认站立姿态：`action = default_stand + action_offset`
- 若未加载策略，使用 `default_action` 作为回退

### 6.5 扩展新状态

1. 在 `config/states.yaml` 中添加状态配置
2. 若为标准静态/动态行为，无需改代码
3. 若需特殊逻辑，继承 `State` 基类实现新派生类，并在 `State::createFromConfig` 注册

---

## 7. CAN 总线与电机驱动

### 7.1 硬件拓扑

```
RK3588 USB ──► USB-CAN HUB ──┬── CAN0: 左前腿 (Hip, Thigh, Calf)
                              ├── CAN1: 右前腿 (3 motors)
                              ├── CAN2: 左后腿 (3 motors)
                              └── CAN3: 右后腿 (3 motors)
```

- 12 个电机分散到 4 路 CAN，每路 3 个，降低总线负载；
- 波特率建议 **1Mbps**；
- 若 HUB 只有 2 路 CAN，则每路挂 6 个电机，需优化报文密度。

### 7.2 Linux SocketCAN 配置

```bash
#!/bin/bash
# scripts/setup_can.sh
for dev in can0 can1 can2 can3; do
    sudo ip link set $dev type can bitrate 1000000
    sudo ip link set up $dev
done
```

### 7.3 电机抽象接口

```cpp
class JointMotor {
public:
    virtual bool init(int id, const std::string& can_iface) = 0;
    virtual bool enable() = 0;
    virtual bool disable() = 0;
    virtual bool setCommand(float pos, float vel, float kp, float kd, float torque) = 0;
    virtual bool readState(float& pos, float& vel, float& torque, float& temp, uint16_t& err) = 0;
};
```

针对电机协议实现派生类：`UnitreeMotor`、`MiteCheetahMotor`、`DMMotor` 等。

---

## 8. 安全机制

| 安全项 | 实现位置 | 策略 |
|--------|----------|------|
| 关节限位 | motion_control_node + safety_monitor | 软件限位，超限保持边界值 |
| 速度/力矩限幅 | motion_control_node | 输出饱和 |
| 通信超时 | safety_monitor | MOTOR_CMD / JOINT_STATE 超时 > 50ms 触发 E-Stop |
| 电机故障 | motor_driver_node + safety_monitor | 监控错误码、过温、过流 |
| 姿态异常 | safety_monitor | roll/pitch > 45° 触发保护 |
| 心跳/看门狗 | safety_monitor | 关键节点互发心跳，丢失后电机进入零力模式 |
| 急停 | safety_monitor + 硬件按钮 | 立即切断电机输出或进入 limp 模式 |

---

## 9. 实时性与性能优化

1. **CPU 绑核**
   - `motor_driver_node` + `motion_control_node` 绑定到 RK3588 大核（Cortex-A76）；
   - `rl_policy_node` 绑定到另一 A76 或 NPU；
   - 使用 `taskset` / `sched_setaffinity`。

2. **实时调度**
   - 1kHz 控制线程使用 `SCHED_FIFO`，优先级 80+；
   - 实时线程内禁止内存分配、文件 IO、复杂日志。

3. **LCM 优化**
   - 使用 UDP 多播或共享内存后端；
   - 高频消息尽量复用缓冲区，避免频繁 `malloc`。

4. **RL 推理优化**
   - RKNN 使用 INT8 量化；
   - ONNX Runtime 开启图优化（ORT_ENABLE_ALL）；
   - 预分配输入/输出 tensor。

---

## 10. 推荐目录结构

```
S0_cerebellum/
├── docs/
│   └── architecture.md
├── lcm_types/
│   ├── user_cmd_t.lcm
│   ├── imu_t.lcm
│   ├── joint_state_t.lcm
│   ├── robot_state_t.lcm
│   ├── rl_action_t.lcm
│   ├── state_status_t.lcm
│   ├── motor_cmd_t.lcm
│   └── safety_status_t.lcm
├── include/
│   ├── core/
│   │   ├── state.hpp
│   │   ├── state_context.hpp
│   │   ├── state_machine.hpp
│   │   ├── static_state.hpp
│   │   ├── gait_state.hpp
│   │   ├── periodic_task.hpp
│   │   ├── lowpass_filter.hpp
│   │   └── robot_config.hpp
│   ├── drivers/
│   ├── rl/
│   └── utils/
├── src/
│   ├── nodes/
│   │   ├── command_node.cpp
│   │   ├── sensor_node.cpp
│   │   ├── rl_policy_node.cpp
│   │   ├── motion_control_node.cpp
│   │   ├── motor_driver_node.cpp
│   │   ├── safety_monitor_node.cpp
│   │   └── logger_node.cpp
│   ├── core/
│   │   ├── state.cpp
│   │   ├── state_machine.cpp
│   │   ├── static_state.cpp
│   │   ├── gait_state.cpp
│   │   ├── periodic_task.cpp
│   │   ├── lowpass_filter.cpp
│   │   └── robot_config.cpp
│   ├── drivers/
│   ├── rl/
│   └── utils/
├── config/
│   ├── robot.yaml
│   ├── motors.yaml
│   └── states.yaml
├── scripts/
│   ├── setup_can.sh
│   └── launch.sh
├── tests/
├── CMakeLists.txt
└── README.md
```

---

## 11. 关键技术栈

| 用途 | 选型 |
|------|------|
| 进程通信 | LCM |
| 数学运算 | Eigen3 |
| 配置解析 | yaml-cpp |
| 状态机 | 自研 State 模式 + YAML 配置 |
| RL 推理 | ONNX Runtime / RKNN Toolkit2 |
| CAN 通信 | Linux SocketCAN |
| 线程/调度 | pthread + sched.h |
| 日志 | spdlog（仅非实时线程） |
| 构建 | CMake |

---

## 12. 落地建议

1. **第一阶段**：打通 `motor_driver_node` + `sensor_node`，验证 CAN 通信与 IMU 数据；
2. **第二阶段**：实现 `command_node` + `motion_control_node` + 状态机，验证 stand/crouch/lie/lock 切换；
3. **第三阶段**：接入 RL 策略，训练/验证 crawl/walk/trot/pace/bound/gallop 多步态策略；
4. **第四阶段**：地面行走，迭代调参、安全阈值与观测设计；
5. **第五阶段**：RKNN 转换，NPU 部署，做延迟与功耗优化。
