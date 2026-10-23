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
│   ├── drivers/            # 驱动（CAN、电机抽象）
│   ├── nodes/              # 7 个控制节点
│   ├── rl/                 # RL 策略模型抽象
│   └── utils/              # 日志与配置解析
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
./build/command_node
./build/sensor_node
./build/rl_policy_node
./build/motion_control_node
./build/motor_driver_node
./build/safety_monitor_node
./build/logger_node
```

## 测试

```bash
cd build
./test_lcm
./test_can can0
```

## 后续开发提示

1. 在 `src/drivers/` 中实现具体电机协议（Unitree、DM 等）
2. 在 `src/rl/` 中实现 `OnnxPolicy` / `RknnPolicy`
3. 在 `src/nodes/motor_driver_node.cpp` 中接入真实 CAN 电机驱动
4. 在 `src/nodes/sensor_node.cpp` 中接入真实 IMU 驱动
5. 在 `src/nodes/rl_policy_node.cpp` 中加载训练好的策略模型
6. 调整 `config/robot.yaml` 和 `config/motors.yaml` 中的机器人参数

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
