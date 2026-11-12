#include "motor_ros2/motor_cfg.h"

#include <cstdio>
#include <cstdlib>
#include <map>
#include <thread>

namespace {

// ---------------------------------------------------------------------------
// 参数应答帧解码表：说明书可读写参数表，印刷页 36~37
// 手册规定参数帧 Byte4~7 为"低字节在前，高字节在后"（小端）
// ---------------------------------------------------------------------------
using FieldPtr = data_read_write_one data_read_write::*;

struct ParamDescriptor {
  FieldPtr field;
  uint8_t size; // 1 / 2 / 4 字节
  bool is_float;
};

const std::map<uint16_t, ParamDescriptor> &param_table() {
  static const std::map<uint16_t, ParamDescriptor> table = {
      {idx_of(ParamIndex::run_mode), {&data_read_write::run_mode, 1, false}},
      {idx_of(ParamIndex::iq_ref), {&data_read_write::iq_ref, 4, true}},
      {idx_of(ParamIndex::spd_ref), {&data_read_write::spd_ref, 4, true}},
      {idx_of(ParamIndex::limit_torque), {&data_read_write::imit_torque, 4, true}},
      {idx_of(ParamIndex::cur_kp), {&data_read_write::cur_kp, 4, true}},
      {idx_of(ParamIndex::cur_ki), {&data_read_write::cur_ki, 4, true}},
      {idx_of(ParamIndex::cur_filt_gain), {&data_read_write::cur_filt_gain, 4, true}},
      {idx_of(ParamIndex::loc_ref), {&data_read_write::loc_ref, 4, true}},
      {idx_of(ParamIndex::limit_spd), {&data_read_write::limit_spd, 4, true}},
      {idx_of(ParamIndex::limit_cur), {&data_read_write::limit_cur, 4, true}},
      {idx_of(ParamIndex::mechPos), {&data_read_write::mechPos, 4, true}},
      {idx_of(ParamIndex::iqf), {&data_read_write::iqf, 4, true}},
      {idx_of(ParamIndex::mechVel), {&data_read_write::mechVel, 4, true}},
      {idx_of(ParamIndex::VBUS), {&data_read_write::VBUS, 4, true}},
      {idx_of(ParamIndex::loc_kp), {&data_read_write::loc_kp, 4, true}},
      {idx_of(ParamIndex::spd_kp), {&data_read_write::spd_kp, 4, true}},
      {idx_of(ParamIndex::spd_ki), {&data_read_write::spd_ki, 4, true}},
      {idx_of(ParamIndex::spd_filt_gain), {&data_read_write::spd_filt_gain, 4, true}},
      {idx_of(ParamIndex::acc_rad), {&data_read_write::acc_rad, 4, true}},
      {idx_of(ParamIndex::vel_max), {&data_read_write::vel_max, 4, true}},
      {idx_of(ParamIndex::acc_set), {&data_read_write::acc_set, 4, true}},
      {idx_of(ParamIndex::EPScan_time), {&data_read_write::EPScan_time, 2, false}},
      {idx_of(ParamIndex::cantimeout), {&data_read_write::cantimeout, 4, false}},
      {idx_of(ParamIndex::zero_sta), {&data_read_write::zero_sta, 1, false}},
      {idx_of(ParamIndex::damper), {&data_read_write::damper, 1, false}},
      {idx_of(ParamIndex::add_offset), {&data_read_write::add_offset, 4, true}},
      {idx_of(ParamIndex::alveolous_open), {&data_read_write::alveolous_open, 1, false}},
      {idx_of(ParamIndex::iq_test), {&data_read_write::iq_test, 1, false}},
      {idx_of(ParamIndex::dcc_set), {&data_read_write::dcc_set, 4, true}},
      {idx_of(ParamIndex::faultSta), {&data_read_write::faultSta, 4, false}},
  };
  return table;
}

void fill_frame(struct can_frame &frame, uint8_t comm_type, uint8_t master_id,
                uint8_t motor_id) {
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = (static_cast<uint32_t>(comm_type) << 24) |
                 (static_cast<uint32_t>(master_id) << 8) |
                 static_cast<uint32_t>(motor_id) | CAN_EFF_FLAG;
  frame.can_dlc = 8;
}

uint32_t le32(const std::vector<uint8_t> &d, std::size_t off) {
  if (d.size() < off + 4)
    return 0;
  return static_cast<uint32_t>(d[off]) |
         (static_cast<uint32_t>(d[off + 1]) << 8) |
         (static_cast<uint32_t>(d[off + 2]) << 16) |
         (static_cast<uint32_t>(d[off + 3]) << 24);
}

} // namespace

// ---------------------------------------------------------------------------
// SocketCAN 初始化：CAN 2.0 扩展帧，1 Mbps；仅接收目标电机 ID 的帧
// 说明书 4.1 反馈帧 bit15~8 为当前电机 CAN_ID
// ---------------------------------------------------------------------------
void RobStrideMotor::init_socket() {
  socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd < 0) {
    perror("socket");
    exit(1);
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
    perror("ioctl");
    exit(1);
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    exit(1);
  }

  struct can_filter rfilter[1];
  rfilter[0].can_id = (static_cast<uint32_t>(motor_id) << 8) | CAN_EFF_FLAG;
  rfilter[0].can_mask = (0xFFu << 8) | CAN_EFF_FLAG;
  if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter,
                 sizeof(rfilter)) < 0) {
    perror("setsockopt filter");
    exit(1);
  }
}

int RobStrideMotor::send_frame(const struct can_frame &frame) {
  constexpr int max_retries = 50;
  for (int i = 0; i < max_retries; ++i) {
    ssize_t n = ::write(socket_fd, &frame, sizeof(frame));
    if (n == static_cast<ssize_t>(sizeof(frame))) {
#ifdef RS_MOTOR_DEBUG
      std::cerr << "[can tx] id=0x" << std::hex << std::uppercase
                << (frame.can_id & CAN_EFF_MASK) << std::dec
                << " dlc=" << static_cast<int>(frame.can_dlc) << " data=";
      for (int j = 0; j < frame.can_dlc; ++j)
        std::cerr << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(frame.data[j]) << " ";
      std::cerr << std::dec << std::endl;
#endif
      return 0;
    }

    if (errno == ENOBUFS) {
      // SocketCAN 发送队列暂时满，等待一小段时间后重试同一帧
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }

    perror("write can frame");
    return -1;
  }
  std::cerr << "[warn] write can frame: No buffer space available after "
            << max_retries << " retries" << std::endl;
  return -1;
}

// ---------------------------------------------------------------------------
// 内部 helper：失能态下写 run_mode(0x7005) -> 回读 -> 更新本地状态。
// 调用方负责前置判断（ensure_run_mode / set_zero_position）与是否使能。
// ---------------------------------------------------------------------------
void RobStrideMotor::write_run_mode(uint8_t mode) {
  const uint16_t idx = idx_of(ParamIndex::run_mode);
  Set_RobStrite_Motor_parameter(idx, static_cast<float>(mode), Set_mode);
  Get_RobStrite_Motor_parameter(idx);
  drw.run_mode.data = static_cast<float>(mode);
}

// ---------------------------------------------------------------------------
// 内部 helper：模式切换。
// 原 5 个控制函数（运控 / 速度 / PP / CSP / 电流）各自重复同一段流程：
//   Disenable_Motor -> 写 run_mode(0x7005) -> 回读 -> 更新本地状态 -> enable_motor
// 返回是否真的发生了切换，速度模式据此判断是否补写 limit_cur / acc_rad。
// 注意 set_zero_position() 不在此列：标零后不自动使能，语义不同。
// ---------------------------------------------------------------------------
bool RobStrideMotor::ensure_run_mode(uint8_t mode) {
  if (drw.run_mode.data == static_cast<float>(mode))
    return false;
  Disenable_Motor(0);
  write_run_mode(mode);
  enable_motor();
  return true;
}

// ---------------------------------------------------------------------------
// 内部 helper：发送一帧控制命令并等待应答。
// 把原先散落在 8+ 个函数里的 "send_frame -> 日志 -> receive_status_frame"
// 样板集中到一处；ok_msg 为空时不打印日志。行为与原实现一致。
// ---------------------------------------------------------------------------
bool RobStrideMotor::send_and_wait(const struct can_frame &frame,
                                   const char *ok_msg) {
  if (send_frame(frame) != 0)
    return false;
  if (ok_msg != nullptr)
    std::cout << ok_msg << std::endl;
  return receive_status_frame();
}

// ---------------------------------------------------------------------------
// 应答帧解析
//   类型 2 / 0x18 : 状态反馈（大端 4×u16，温度 ×10）
//   类型 2 + 0xC4 : 版本号读取应答
//   类型 0x11     : 单个参数读取应答（小端）
//   类型 0x15     : 故障反馈帧 fault / warning
//   类型 0        : 设备 ID / 64 位 MCU 唯一标识符
// ---------------------------------------------------------------------------
bool RobStrideMotor::receive_status_frame(double timeout_sec,
                                         uint16_t expect_index) {
  auto result = receive(timeout_sec);
  if (!result)
    return false;

  auto [communication_type, extra_data, host_id, data] = *result;
  (void)extra_data;
  (void)host_id;
  if (data.size() < 8)
    return false;

  switch (communication_type) {
  case Communication_Type_MotorRequest:
  case Communication_Type_ActiveReport: {
    // 注意：本分支只刷新 position_/velocity_/torque_/temperature_，
    // 不会写 params.index。因此当调用方传入 expect_index（参数读取）时，
    // 这里的 true 仅代表"收到了状态帧"，不代表"参数读取成功"；
    // Get_RobStrite_Motor_parameter() 会再用 params.index 复核，防止
    // 陈旧状态帧伪装成读取应答。
    // 版本号读取应答（说明书印刷页 32）
    if (communication_type == Communication_Type_MotorRequest &&
        data[0] == 0x00 && data[1] == MOTOR_VERSION_READ_CMD &&
        data[2] == MOTOR_VERSION_ACK_TAG) {
      for (std::size_t i = 0; i < firmware_version_.size(); ++i)
        firmware_version_[i] = data[3 + i];
      version_valid_ = true;
      return true;
    }

    const ActuatorOperation &lim = limits();
    uint16_t p = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    uint16_t v = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    uint16_t t = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    uint16_t temp = (static_cast<uint16_t>(data[6]) << 8) | data[7];

    // 手册：0~65535 线性对应 -L~+L
    auto dequantize = [](uint16_t u, double limit) {
      return (static_cast<float>(u) / 65535.0f * 2.0f - 1.0f) *
             static_cast<float>(limit);
    };
    position_ = dequantize(p, lim.position);
    velocity_ = dequantize(v, lim.velocity);
    torque_ = dequantize(t, lim.torque);
    temperature_ = static_cast<float>(temp) * 0.1f;
    return true;
  }

  case Communication_Type_GetSingleParameter: {
    uint16_t index =
        static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
    // 手册 4.1.8：读取应答帧 ID 的 bit23~16，00 为读取成功，01 为读取失败
    if (error_code != 0) {
      std::cerr << "[warn] parameter read failed, index 0x" << std::hex
                << index << std::dec << std::endl;
      return false;
    }
    // 调用方指定了期望索引时，丢弃错位帧，防止把其它参数的值写进错误字段
    if (expect_index != 0 && index != expect_index) {
      std::cerr << "[warn] parameter index mismatch: got 0x" << std::hex
                << index << ", expect 0x" << expect_index << std::dec
                << std::endl;
      return false;
    }
    params.index = index;
    auto it = param_table().find(index);
    if (it == param_table().end()) {
      std::cerr << "[warn] unknown parameter index 0x" << std::hex << index
                << std::dec << std::endl;
      return false;
    }
    const ParamDescriptor &desc = it->second;
    float value = 0.0f;
    if (desc.is_float && desc.size == 4) {
      value = Byte_to_float(data);
    } else if (desc.size == 1) {
      value = static_cast<float>(data[4]);
    } else if (desc.size == 2) {
      value = static_cast<float>(static_cast<uint32_t>(data[4]) |
                                 (static_cast<uint32_t>(data[5]) << 8));
    } else {
      value = static_cast<float>(le32(data, 4));
    }
    (drw.*(desc.field)).data = value;
    params.data = value;
    return true;
  }

  case Communication_Type_FaultFeedback: {
    fault_code_ = le32(data, 0);
    warning_code_ = le32(data, 4);
    if (fault_code_ != 0) {
      std::cerr << "[warn] motor fault=0x" << std::hex << fault_code_
                << " warning=0x" << warning_code_ << std::dec << std::endl;
    }
    return true;
  }

  case Communication_Type_Get_ID: {
    std::copy_n(data.begin(), mcu_uid_.size(), mcu_uid_.begin());
    uid_valid_ = true;
    return true;
  }

  default:
    std::cerr << "[warn] unhandled communication type 0x" << std::hex
              << static_cast<int>(communication_type) << std::dec << std::endl;
    return false;
  }
}

// ---------------------------------------------------------------------------
// 单个参数读写（通信类型 18 / 17）
// ---------------------------------------------------------------------------
void RobStrideMotor::write_parameter(uint16_t index, const void *value,
                                     uint8_t size) {
  if (value == nullptr || size == 0 || size > 4) {
    std::cerr << "[warn] invalid parameter payload" << std::endl;
    return;
  }
  struct can_frame frame{};
  fill_frame(frame, Communication_Type_SetSingleParameter, master_id, motor_id);
  frame.data[0] = static_cast<uint8_t>(index & 0xFF);
  frame.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  std::memcpy(&frame.data[4], value, size);
  if (send_frame(frame) == 0)
    receive_status_frame();
}

void RobStrideMotor::write_parameter_u16(uint16_t index, uint16_t value) {
  write_parameter(index, &value, 2);
}

void RobStrideMotor::Set_RobStrite_Motor_parameter(uint16_t Index, float Value,
                                                   char Value_mode) {
  if (Value_mode == Set_parameter) {
    write_parameter(Index, &Value, 4);
  } else if (Value_mode == Set_mode) {
    uint8_t v = static_cast<uint8_t>(Value);
    write_parameter(Index, &v, 1);
  } else {
    std::cerr << "[warn] unknown parameter value mode: " << Value_mode
              << std::endl;
  }
}

bool RobStrideMotor::Get_RobStrite_Motor_parameter(uint16_t Index) {
  struct can_frame frame{};
  fill_frame(frame, Communication_Type_GetSingleParameter, master_id, motor_id);
  frame.data[0] = static_cast<uint8_t>(Index & 0xFF);
  frame.data[1] = static_cast<uint8_t>((Index >> 8) & 0xFF);
  if (send_frame(frame) != 0)
    return false;
  // 只有类型 17（0x11）读取应答会写 params.index；类型 2 / 0x18 状态反馈帧
  // 不会。先清零再复核，可防止陈旧状态帧被误判为"参数读取成功"——这正是
  // 18:22 ParamRW 读到陈旧值 5 的一类根因。19:48 candump 中类型 17 读请求
  // 与读应答一一对应（87/87），正常读取路径不受影响。
  params.index = 0;
  if (!receive_status_frame(0.05, Index))
    return false;
  return params.index == Index;
}

// ---------------------------------------------------------------------------
// 通信类型 3：电机使能运行
// ---------------------------------------------------------------------------
std::tuple<float, float, float, float> RobStrideMotor::enable_motor() {
  struct can_frame frame{};
  fill_frame(frame, Communication_Type_MotorEnable, master_id, motor_id);
  send_and_wait(frame, "[ok] Motor enable command sent.");
  return return_data_pvtt();
}

// ---------------------------------------------------------------------------
// 通信类型 4：电机停止运行。手册：data 区清 0；Byte[0]=1 时清故障
// ---------------------------------------------------------------------------
void RobStrideMotor::Disenable_Motor(uint8_t clear_error) {
  struct can_frame frame{};
  fill_frame(frame, Communication_Type_MotorStop, master_id, motor_id);
  frame.data[0] = clear_error;
  send_and_wait(frame, "[ok] Motor disable command sent.");
}

uint16_t RobStrideMotor::float_to_uint(float x, float x_min, float x_max,
                                       int bits) {
  x = std::clamp(x, x_min, x_max);
  float span = x_max - x_min;
  float denom = static_cast<float>((static_cast<uint32_t>(1) << bits) - 1);
  return static_cast<uint16_t>((x - x_min) * denom / span);
}

// ---------------------------------------------------------------------------
// 通信类型 1：运控模式控制指令
// ID bit23~8 为力矩量化值；数据域大端 pos / vel / Kp / Kd
// ---------------------------------------------------------------------------
std::tuple<float, float, float, float>
RobStrideMotor::send_motion_command(float torque, float position_rad,
                                    float velocity_rad_s, float kp, float kd) {
  ensure_run_mode(move_control_mode);

  const ActuatorOperation &lim = limits();
  uint16_t torque_u = float_to_uint(torque, -static_cast<float>(lim.torque),
                                    static_cast<float>(lim.torque), 16);

  struct can_frame frame{};
  frame.can_id = (static_cast<uint32_t>(Communication_Type_MotionControl)
                  << 24) |
                 (static_cast<uint32_t>(torque_u) << 8) |
                 static_cast<uint32_t>(motor_id) | CAN_EFF_FLAG;
  frame.can_dlc = 8;

  uint16_t pos = float_to_uint(position_rad, -static_cast<float>(lim.position),
                               static_cast<float>(lim.position), 16);
  uint16_t vel = float_to_uint(velocity_rad_s, -static_cast<float>(lim.velocity),
                               static_cast<float>(lim.velocity), 16);
  uint16_t kp_u = float_to_uint(kp, RS03_KP_MIN,
                                static_cast<float>(lim.kp), 16);
  uint16_t kd_u = float_to_uint(kd, RS03_KD_MIN,
                                static_cast<float>(lim.kd), 16);

  frame.data[0] = static_cast<uint8_t>(pos >> 8);
  frame.data[1] = static_cast<uint8_t>(pos);
  frame.data[2] = static_cast<uint8_t>(vel >> 8);
  frame.data[3] = static_cast<uint8_t>(vel);
  frame.data[4] = static_cast<uint8_t>(kp_u >> 8);
  frame.data[5] = static_cast<uint8_t>(kp_u);
  frame.data[6] = static_cast<uint8_t>(kd_u >> 8);
  frame.data[7] = static_cast<uint8_t>(kd_u);

  if (send_frame(frame) == 0)
    receive_status_frame();
  return return_data_pvtt();
}

// ---------------------------------------------------------------------------
// 速度模式（说明书 4.3.3，印刷页 40）
// run_mode=2 -> 使能 -> limit_cur(0x7018) -> acc_rad(0x7022) -> spd_ref(0x700A)
// ---------------------------------------------------------------------------
std::tuple<float, float, float, float>
RobStrideMotor::send_velocity_mode_command(float velocity_rad_s,
                                           float limit_cur_a) {
  if (ensure_run_mode(Speed_control_mode)) {
    // 首次切入速度模式才补写这两个参数，避免每 tick 重复写。
    // 0x7018 limit_cur，手册范围 0~43 A。默认 27 A 为历史值；
    // 供电容量紧张（如 30V/2A）时应由调用方传更保守的上限
    write_parameter(idx_of(ParamIndex::limit_cur), &limit_cur_a, 4);

    float acc = RS03_ACC_RAD_DEFAULT;
    write_parameter(idx_of(ParamIndex::acc_rad), &acc, 4);
  }

  float spd = std::clamp(velocity_rad_s, RS03_V_MIN, RS03_V_MAX);
  Set_RobStrite_Motor_parameter(idx_of(ParamIndex::spd_ref), spd,
                                Set_parameter);
  return return_data_pvtt();
}

// ---------------------------------------------------------------------------
// 位置模式 PP（说明书 4.3.5，印刷页 41）
// run_mode=1 -> 使能 -> vel_max(0x7024) -> acc_set(0x7025) -> loc_ref(0x7016)
// ---------------------------------------------------------------------------
std::tuple<float, float, float, float>
RobStrideMotor::RobStrite_Motor_PosPP_control(float Speed, float Acceleration,
                                              float Angle) {
  ensure_run_mode(PosPP_control_mode);

  float vel = std::max(0.0f, Speed);
  float acc = std::max(0.0f, Acceleration);
  float angle = std::clamp(Angle, RS03_P_MIN, RS03_P_MAX);

  Set_RobStrite_Motor_parameter(idx_of(ParamIndex::vel_max), vel,
                                Set_parameter);
  Set_RobStrite_Motor_parameter(idx_of(ParamIndex::acc_set), acc,
                                Set_parameter);
  Set_RobStrite_Motor_parameter(idx_of(ParamIndex::loc_ref), angle,
                                Set_parameter);
  return return_data_pvtt();
}

// ---------------------------------------------------------------------------
// 位置模式 CSP（说明书 4.3.4，印刷页 40）
// run_mode=5 -> 使能 -> limit_spd(0x7017) -> loc_ref(0x7016)
// ---------------------------------------------------------------------------
std::tuple<float, float, float, float>
RobStrideMotor::RobStrite_Motor_PosCSP_control(float Speed, float Angle) {
  ensure_run_mode(PosCSP_control_mode);

  float limit_spd = std::clamp(Speed, 0.0f, RS03_LIMIT_SPD_MAX);
  float angle = std::clamp(Angle, RS03_P_MIN, RS03_P_MAX);

  Set_RobStrite_Motor_parameter(idx_of(ParamIndex::limit_spd), limit_spd,
                                Set_parameter);
  Set_RobStrite_Motor_parameter(idx_of(ParamIndex::loc_ref), angle,
                                Set_parameter);
  return return_data_pvtt();
}

// ---------------------------------------------------------------------------
// 电流模式（说明书 4.3.2，印刷页 40）
// iq_ref(0x7006) 为 float 物理值，范围 -43~43 A，不做量化
// ---------------------------------------------------------------------------
std::tuple<float, float, float, float>
RobStrideMotor::RobStrite_Motor_Current_control(float IqCommand) {
  ensure_run_mode(Elect_control_mode);

  float iq = std::clamp(IqCommand, RS03_IQ_MIN, RS03_IQ_MAX);
  Set_RobStrite_Motor_parameter(idx_of(ParamIndex::iq_ref), iq,
                                Set_parameter);
  return return_data_pvtt();
}

// ---------------------------------------------------------------------------
// 通信类型 6：设置电机机械零位
// 手册 4.2.6：CSP / 运控模式可标零，PP 模式标零会被屏蔽
// ---------------------------------------------------------------------------
void RobStrideMotor::set_zero_position() {
  Disenable_Motor(0);
  // 手册：零位标定仅在运控模式（0）或 CSP 模式（5）下有效，PP 模式会被屏蔽
  if (drw.run_mode.data != move_control_mode &&
      drw.run_mode.data != PosCSP_control_mode) {
    write_run_mode(move_control_mode);
  }

  struct can_frame frame{};
  fill_frame(frame, Communication_Type_SetPosZero, master_id, motor_id);
  frame.data[0] = 1;
  send_and_wait(frame, "[ok] Set mechanical zero command sent.");
  // 参考 MotorBridge 等实现的完整标零流程：类型 6 之后需写
  // zero_sta(0x7029)=1，零点标志位才会置起（仅写不读，写参数本身
  // 会消耗一封应答帧）。注意：零位掉电即丢失，如需持久化掉电保存，
  // 调用方需再发通信类型 22（save_parameters()）。
  Set_RobStrite_Motor_parameter(idx_of(ParamIndex::zero_sta), 1.0f, Set_mode);
  // 标零后不自动使能：使能时机交由调用方控制（测试各阶段需要先读取
  // 佐证参数再换新反馈帧），与其它参数写入函数的行为保持一致。
}

// ---------------------------------------------------------------------------
// 通信类型 7：设置电机 CAN_ID（bit16~23 为新 ID）
// ---------------------------------------------------------------------------
void RobStrideMotor::Set_CAN_ID(uint8_t Set_CAN_ID) {
  Disenable_Motor(0);

  struct can_frame frame{};
  fill_frame(frame, Communication_Type_Can_ID, master_id, motor_id);
  frame.can_id |= (static_cast<uint32_t>(Set_CAN_ID) << 16);
  send_and_wait(frame, "[ok] Set CAN_ID command sent.");
}

// ---------------------------------------------------------------------------
// 通信类型 0：获取设备 ID 与 64 位 MCU 唯一标识符
// ---------------------------------------------------------------------------
void RobStrideMotor::get_device_id() {
  struct can_frame frame{};
  fill_frame(frame, Communication_Type_Get_ID, master_id, motor_id);
  send_and_wait(frame);
}

// ---------------------------------------------------------------------------
// 版本号读取帧（说明书印刷页 32）：类型 4，Byte[1]=0xC4
// ---------------------------------------------------------------------------
void RobStrideMotor::read_firmware_version() {
  struct can_frame frame{};
  fill_frame(frame, Communication_Type_MotorStop, master_id, motor_id);
  frame.data[0] = 0x00;
  frame.data[1] = MOTOR_VERSION_READ_CMD;
  send_and_wait(frame);
}

// ---------------------------------------------------------------------------
// 通信类型 22 (0x16)：保存当前所有可存储变量
// ---------------------------------------------------------------------------
void RobStrideMotor::save_parameters() {
  struct can_frame frame{};
  fill_frame(frame, Communication_Type_SaveParameters, master_id, motor_id);
  for (uint8_t i = 0; i < 8; ++i)
    frame.data[i] = static_cast<uint8_t>(i + 1);
  send_and_wait(frame);
}

// ---------------------------------------------------------------------------
// 通信类型 23/24/25：F_CMD 位于 Byte[6]，前 6 字节为固定填充
// ---------------------------------------------------------------------------
static void fill_fcmd_payload(struct can_frame &frame, uint8_t cmd) {
  const uint8_t prefix[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
  std::memcpy(frame.data, prefix, sizeof(prefix));
  frame.data[6] = cmd;
  frame.data[7] = 0x00;
}

void RobStrideMotor::set_baudrate(uint8_t baud_code) {
  struct can_frame frame{};
  fill_frame(frame, Communication_Type_SetBaudrate, master_id, motor_id);
  fill_fcmd_payload(frame, baud_code);
  send_and_wait(frame);
}

void RobStrideMotor::set_active_report(bool enable) {
  struct can_frame frame{};
  fill_frame(frame, Communication_Type_ActiveReport, master_id, motor_id);
  // 手册：主动上报默认关闭；开启后默认 10ms 周期，可用 0x7026 调整
  fill_fcmd_payload(frame, enable ? 0x01 : 0x00);
  send_and_wait(frame);
}

void RobStrideMotor::set_motor_protocol(uint8_t protocol) {
  struct can_frame frame{};
  fill_frame(frame, Communication_Type_SetProtocol, master_id, motor_id);
  fill_fcmd_payload(frame, protocol);
  send_and_wait(frame);
}
