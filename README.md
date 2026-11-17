# Cerebellum — Quadruped Robot Motion Control

基于 RK3588 + LCM + SocketCAN 的四足机器人小脑运动控制框架，不依赖 ROS。

## 系统架构

详见 `docs/architecture.md`。

## 目录结构

```
.
├── CMakeLists.txt          # CMake 构建脚本
├── README.md               # 本文件
├── config/                 # 机器人、电机与状态机配置
├── docs/                   # 架构文档
├── include/                # 头文件
├── lcm_types/              # LCM 消息定义
├── scripts/                # 启动与配置脚本
├── src/                    # 源码
│   ├── core/               # 核心工具（周期任务、滤波、配置、状态机）
│   ├── drivers/            # 驱动（CAN、电机抽象、RobStride 适配器）
│   ├── nodes/              # 7 个控制节点
│   ├── rl/                 # RL 策略模型抽象
│   └── utils/              # 日志与配置解析
├── third_party/            # 外部引入代码（原样保留，不得修改）
│   ├── robstride/          # RobStride RS03 电机驱动（Apache-2.0）
│   ├── hipnuc/             # HiPNUC IMU 解码库 + Linux 串口层（Apache-2.0）
│   └── battery/            # BMS Modbus RTU 驱动（含规约文档）
└── tests/                  # 测试程序
```

## 依赖

- Ubuntu 22.04 on RK3588
- CMake >= 3.16
- LCM
- Eigen3
- yaml-cpp
- pthread
- Linux SocketCAN (kernel built-in)
- ONNX Runtime / RKNN Toolkit2 (for RL inference)

### 安装依赖（Ubuntu）

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev libyaml-cpp-dev

# Build and install LCM from source
# https://github.com/lcm-proj/lcm
```

## 构建

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## 运行

### 1. 配置 CAN 接口

```bash
sudo ./scripts/setup_can.sh
```

### 2. 启动所有节点

```bash
./scripts/launch.sh
```

或单独启动：

```bash
./build/power_board_node config/power.yaml
./build/battery_node config/battery.yaml
./build/command_node
./build/sensor_node config/robot.yaml
./build/rl_policy_node config/states.yaml config/robot.yaml
./build/motion_control_node
./build/motor_driver_node config/motors.yaml
./build/safety_monitor_node
./build/logger_node
```

## 测试

```bash
cd build
./test_lcm
./test_can can0
```

## 电机驱动说明

- `third_party/robstride/` 为外部引入的 RobStride RS03 电机驱动源码（Apache-2.0），**原样保留，不得修改**；
- `src/drivers/robstride_motor_adapter.cpp` 是适配器，把第三方 `RobStrideMotor` 包装成 `JointMotor` 接口（仅做参数顺序映射与反馈缓存）；
- `motor_driver_node` 只依赖 `JointMotor` 接口与 `createMotor()` 工厂，更换电机品牌时只需新增适配器并在工厂中加一个分支；
- 反馈为请求-应答模式：`setCommand()` 同步返回反馈，`readState()` 读取最近一次反馈。

## IMU

- `third_party/hipnuc/` 为外部引入的 HiPNUC IMU 解码库与 Linux 串口层（Apache-2.0，SPDX 声明随源文件保留），**原样保留，不得修改**；
- `src/drivers/hipnuc_imu_driver.cpp` 是适配器，把 `hipnuc_serial_read()` 包装成 `ImuDriver` 接口（仅做字段映射与有效位转换）；
- `sensor_node` 经 `createImuDriver()` 工厂创建驱动，串口参数（`port`/`baud`）在 `config/robot.yaml` 的 `imu:` 段配置；设备输出 1000Hz，按实际频率发布 `IMU`，`ROBOT_STATE` 聚合为 100Hz；
- 坐标系保持设备原始轴向输出，依赖安装时 IMU 轴向与机体对齐（设备 COORD 设置影响四元数语义，上板需确认）。

## BMS（电池保护板）

- `third_party/battery/` 为外部引入的 BMS Modbus RTU 驱动（原样保留，不得修改），规约文档随附；
- 链路：RK3588 UART(TTL) → TTL 转 485 模块 → BMS，9600 8N1，从机地址 1；
- `battery_node` 1Hz 轮询，发布 `BATTERY_STATUS`（总压/电流/容量/16 串单体电压/双温度/SOC/FET/保护位）；保护位变化时打印日志；每 60s 后台读一次保护参数（仅日志）；
- 串口权限：`sudo usermod -aG dialout $USER`；建议用 udev 规则固定设备名；
- `safety_monitor_node` 订阅 `BATTERY_STATUS`：BMS 保护中 → **E-Stop**；读错误/SOC 低/单体压差大/数据超时 → 警告。

## 电源板

- 电源板接入 USB-CAN HUB 第 5 路（`can4`，独占），协议见《电源板使用说明书 V1.5》；
- `power_board_node` 启动时查询版本号并按 `config/power.yaml` 配置开启各电源轨（默认开 12V + VMBUS），随后以 20Hz 轮询电压/电流/温度/故障位，发布 `POWER_STATUS` 与 `POWER_CURRENTS`；
- 其他节点可通过 `POWER_CMD` 控制电源轨开关、重启、清故障；
- `safety_monitor_node` 订阅 `POWER_STATUS`：电源芯片过流/过温、电源板过温（>100°C）触发 E-Stop，其余故障位与电压异常触发警告。

## 后续开发提示

1. 在 `src/nodes/sensor_node.cpp` 中接入真实 IMU 驱动
2. 在 `src/rl/` 中实现 `OnnxPolicy` / `RknnPolicy`
3. 在 `src/nodes/rl_policy_node.cpp` 中加载训练好的策略模型
4. 调整 `config/robot.yaml`、`config/motors.yaml`、`config/states.yaml` 中的机器人参数

## 状态机

`rl_policy_node` 内置状态机，支持以下状态（在 `config/states.yaml` 中配置）：

- 静态姿态：`stand`、`crouch`、`lie`、`lock`
- 动态步态：`crawl`、`walk`、`trot`、`pace`、`bound`、`gallop`

切换方式：`command_node` 发布的 `user_cmd_t.state_name` 指定目标状态，状态机会自动处理平滑过渡。

新增状态只需在 `config/states.yaml` 中添加条目，并实现对应的 `State` 派生类（参考 `StaticState` / `GaitState`）。

## 安全注意

- 初次上电请使用吊挂或支架固定机器人
- 先验证 `safety_monitor_node` 能正确触发 E-Stop
- 低速逐步开放地面行走
