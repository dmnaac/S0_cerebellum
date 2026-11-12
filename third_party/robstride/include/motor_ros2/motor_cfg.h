#ifndef MOTOR_ROS2_MOTOR_CFG_H_
#define MOTOR_ROS2_MOTOR_CFG_H_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <map>
#include <net/if.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <tuple>
#include <unistd.h>
#include <vector>

// ---------------------------------------------------------------------------
// 参数写入模式标记（保留原有调用接口）
//   Set_mode      -> 1 字节整数参数，如 run_mode
//   Set_parameter -> 4 字节 IEEE-754 float
// 说明书 4.4.4 / 4.4.5，印刷页 44~46
// ---------------------------------------------------------------------------
inline constexpr char Set_mode = 'j';
inline constexpr char Set_parameter = 'p';

// ---------------------------------------------------------------------------
// 控制模式：说明书参数表 0x7005，印刷页 36
// 手册仅定义 0 运控 / 1 PP / 2 速度 / 3 电流 / 5 CSP，不存在模式 4
// 用 constexpr 而非宏：避免预处理期文本替换污染，且保留整型常量表达式语义
// ---------------------------------------------------------------------------
inline constexpr uint8_t move_control_mode = 0;
inline constexpr uint8_t PosPP_control_mode = 1;
inline constexpr uint8_t Speed_control_mode = 2;
inline constexpr uint8_t Elect_control_mode = 3;
inline constexpr uint8_t PosCSP_control_mode = 5;

// ---------------------------------------------------------------------------
// RS03 协议量程：说明书 4.4 官方程序样例宏，印刷页 41
// 参数量程：说明书可读写参数表，印刷页 36~37
// ---------------------------------------------------------------------------
constexpr float RS03_P_MIN = -12.57f;
constexpr float RS03_P_MAX = 12.57f;
constexpr float RS03_V_MIN = -20.0f;
constexpr float RS03_V_MAX = 20.0f;
constexpr float RS03_KP_MIN = 0.0f;
constexpr float RS03_KP_MAX = 5000.0f;
constexpr float RS03_KD_MIN = 0.0f;
constexpr float RS03_KD_MAX = 100.0f;
constexpr float RS03_T_MIN = -60.0f;
constexpr float RS03_T_MAX = 60.0f;
constexpr float RS03_IQ_MIN = -43.0f;  // 0x7006 iq_ref
constexpr float RS03_IQ_MAX = 43.0f;
constexpr float RS03_LIMIT_TORQUE_MAX = 60.0f; // 0x700B
constexpr float RS03_LIMIT_CUR_MAX = 43.0f;    // 0x7018
constexpr float RS03_LIMIT_SPD_MAX = 20.0f;    // 0x7017
constexpr float RS03_ACC_RAD_DEFAULT = 20.0f;  // 0x7022 acc_rad 默认值
constexpr float RS03_VEL_MAX_DEFAULT = 10.0f;  // 0x7024 vel_max 默认值
constexpr float RS03_ACC_SET_DEFAULT = 10.0f;  // 0x7025 acc_set 默认值

// ---------------------------------------------------------------------------
// 通信类型：说明书 4.1，印刷页 31~35
// 章节标题使用十进制编号，CAN ID 中的通信类型字段为十六进制值
// 用 inline constexpr 取代 #define：类型安全、可调试、不污染预处理命名空间
// ---------------------------------------------------------------------------
inline constexpr uint8_t Communication_Type_Get_ID = 0x00;
inline constexpr uint8_t Communication_Type_MotionControl = 0x01;
inline constexpr uint8_t Communication_Type_MotorRequest = 0x02;
inline constexpr uint8_t Communication_Type_MotorEnable = 0x03;
inline constexpr uint8_t Communication_Type_MotorStop = 0x04;
inline constexpr uint8_t Communication_Type_SetPosZero = 0x06;
inline constexpr uint8_t Communication_Type_Can_ID = 0x07;
inline constexpr uint8_t Communication_Type_GetSingleParameter = 0x11;
inline constexpr uint8_t Communication_Type_SetSingleParameter = 0x12;
inline constexpr uint8_t Communication_Type_FaultFeedback = 0x15;
inline constexpr uint8_t Communication_Type_SaveParameters = 0x16;
inline constexpr uint8_t Communication_Type_SetBaudrate = 0x17;
inline constexpr uint8_t Communication_Type_ActiveReport = 0x18;
inline constexpr uint8_t Communication_Type_SetProtocol = 0x19;

// 版本号读取帧：类型 4，Byte[0]=0x00, Byte[1]=0xC4；应答 Byte[2]=0x56
// 说明书印刷页 32
inline constexpr uint8_t MOTOR_VERSION_READ_CMD = 0xC4;
inline constexpr uint8_t MOTOR_VERSION_ACK_TAG = 0x56;

// 波特率 F_CMD（说明书通信类型 23，印刷页 35）
inline constexpr uint8_t MOTOR_BAUDRATE_1M = 0x01;
inline constexpr uint8_t MOTOR_BAUDRATE_500K = 0x02;
inline constexpr uint8_t MOTOR_BAUDRATE_250K = 0x03;
inline constexpr uint8_t MOTOR_BAUDRATE_125K = 0x04;

// 协议 F_CMD（说明书通信类型 25，印刷页 36）
inline constexpr uint8_t MOTOR_PROTOCOL_PRIVATE = 0x00;
inline constexpr uint8_t MOTOR_PROTOCOL_CANOPEN = 0x01;
inline constexpr uint8_t MOTOR_PROTOCOL_MIT = 0x02;

using ReceiveResult =
    std::optional<std::tuple<uint8_t, uint16_t, uint8_t, std::vector<uint8_t>>>;

class data_read_write_one {
public:
  uint16_t index = 0;
  float data = 0.0f;
};

// ---------------------------------------------------------------------------
// 可读写参数列表：说明书印刷页 36~37 及故障查询参数
// 手册参数表中不存在 0x701D；0x701C 之后直接为 0x701E
// 这是参数索引的唯一来源：param_table() 由此派生
// ---------------------------------------------------------------------------
enum class ParamIndex : uint16_t {
  run_mode       = 0x7005, // uint8  1B W/R  0/1/2/3/5
  iq_ref         = 0x7006, // float  4B W/R  -43~43 A
  spd_ref        = 0x700A, // float  4B W/R  -20~20 rad/s
  limit_torque   = 0x700B, // float  4B W/R  0~60 Nm
  cur_kp         = 0x7010, // float  4B W/R  默认 0.17
  cur_ki         = 0x7011, // float  4B W/R  默认 0.012
  cur_filt_gain  = 0x7014, // float  4B W/R  0~1.0，默认 0.1
  loc_ref        = 0x7016, // float  4B W/R  rad
  limit_spd      = 0x7017, // float  4B W/R  CSP 速度限制 0~20 rad/s
  limit_cur      = 0x7018, // float  4B W/R  速度/位置模式电流限制 0~43 A
  mechPos        = 0x7019, // float  4B R    负载端计圈机械角度
  iqf            = 0x701A, // float  4B R    iq 滤波值
  mechVel        = 0x701B, // float  4B R    负载端转速
  VBUS           = 0x701C, // float  4B R    母线电压
  loc_kp         = 0x701E, // float  4B W/R  位置 kp，默认 60
  spd_kp         = 0x701F, // float  4B W/R  速度 kp，默认 6
  spd_ki         = 0x7020, // float  4B W/R  速度 ki，默认 0.02
  spd_filt_gain  = 0x7021, // float  4B W/R  速度滤波，默认 0.1
  acc_rad        = 0x7022, // float  4B W/R  速度模式加速度，默认 20 rad/s^2
  vel_max        = 0x7024, // float  4B W/R  PP 速度，默认 10 rad/s
  acc_set        = 0x7025, // float  4B W/R  PP 加速度，默认 10 rad/s^2
  EPScan_time    = 0x7026, // uint16 2B W    上报时间，1=10ms
  cantimeout     = 0x7028, // uint32 4B W    CAN 超时阈值
  zero_sta       = 0x7029, // uint8  1B W    零点标志位
  damper         = 0x702A, // uint8  1B W/R  阻尼开关
  add_offset     = 0x702B, // float  4B W/R  零位偏置
  alveolous_open = 0x702C, // uint8  1B W/R  齿槽补偿开关
  iq_test        = 0x702D, // uint8  1B W/R  初始化校准开关
  dcc_set        = 0x702E, // float  4B W/R  PP 减速度
  faultSta       = 0x3022, // uint32 4B R    闩锁故障状态查询（部分固件支持）
};

// 参数索引的类型安全转换：enum class -> uint16_t
constexpr uint16_t idx_of(ParamIndex p) {
  return static_cast<uint16_t>(p);
}

class data_read_write {
public:
  // 各字段的 index 由 ParamIndex 枚举在声明处直接初始化，不再靠构造函数
  // 逐行赋值（原先构造函数与枚举是同一映射的两份拷贝）
  data_read_write_one run_mode{idx_of(ParamIndex::run_mode)};
  data_read_write_one iq_ref{idx_of(ParamIndex::iq_ref)};
  data_read_write_one spd_ref{idx_of(ParamIndex::spd_ref)};
  data_read_write_one imit_torque{idx_of(ParamIndex::limit_torque)}; // 保留原字段名，对应 0x700B
  data_read_write_one cur_kp{idx_of(ParamIndex::cur_kp)};
  data_read_write_one cur_ki{idx_of(ParamIndex::cur_ki)};
  data_read_write_one cur_filt_gain{idx_of(ParamIndex::cur_filt_gain)};
  data_read_write_one loc_ref{idx_of(ParamIndex::loc_ref)};
  data_read_write_one limit_spd{idx_of(ParamIndex::limit_spd)};
  data_read_write_one limit_cur{idx_of(ParamIndex::limit_cur)};
  data_read_write_one mechPos{idx_of(ParamIndex::mechPos)};
  data_read_write_one iqf{idx_of(ParamIndex::iqf)};
  data_read_write_one mechVel{idx_of(ParamIndex::mechVel)};
  data_read_write_one VBUS{idx_of(ParamIndex::VBUS)};
  data_read_write_one loc_kp{idx_of(ParamIndex::loc_kp)};
  data_read_write_one spd_kp{idx_of(ParamIndex::spd_kp)};
  data_read_write_one spd_ki{idx_of(ParamIndex::spd_ki)};
  data_read_write_one spd_filt_gain{idx_of(ParamIndex::spd_filt_gain)};
  data_read_write_one acc_rad{idx_of(ParamIndex::acc_rad)};
  data_read_write_one vel_max{idx_of(ParamIndex::vel_max)};
  data_read_write_one acc_set{idx_of(ParamIndex::acc_set)};
  data_read_write_one EPScan_time{idx_of(ParamIndex::EPScan_time)};
  data_read_write_one cantimeout{idx_of(ParamIndex::cantimeout)};
  data_read_write_one zero_sta{idx_of(ParamIndex::zero_sta)};
  data_read_write_one damper{idx_of(ParamIndex::damper)};
  data_read_write_one add_offset{idx_of(ParamIndex::add_offset)};
  data_read_write_one alveolous_open{idx_of(ParamIndex::alveolous_open)};
  data_read_write_one iq_test{idx_of(ParamIndex::iq_test)};
  data_read_write_one dcc_set{idx_of(ParamIndex::dcc_set)};
  data_read_write_one faultSta{idx_of(ParamIndex::faultSta)};
};

// ---------------------------------------------------------------------------
// 电机类型与运行量程映射
// ROBSTRIDE_03 严格采用 RS03 说明书 4.4 官方量程
// ---------------------------------------------------------------------------
enum class ActuatorType {
  ROBSTRIDE_00 = 0,
  ROBSTRIDE_01 = 1,
  ROBSTRIDE_02 = 2,
  ROBSTRIDE_03 = 3,
  ROBSTRIDE_04 = 4,
  ROBSTRIDE_05 = 5,
  ROBSTRIDE_06 = 6
};

struct ActuatorOperation {
  double position; // rad
  double velocity; // rad/s
  double torque;   // Nm
  double kp;
  double kd;
};

inline const std::map<ActuatorType, ActuatorOperation>
    ACTUATOR_OPERATION_MAPPING = {
        {ActuatorType::ROBSTRIDE_00, {4 * M_PI, 50, 17, 500.0, 5.0}},
        {ActuatorType::ROBSTRIDE_01, {4 * M_PI, 44, 17, 500.0, 5.0}},
        {ActuatorType::ROBSTRIDE_02, {4 * M_PI, 44, 17, 500.0, 5.0}},
        {ActuatorType::ROBSTRIDE_03,
         {RS03_P_MAX, RS03_V_MAX, RS03_T_MAX, RS03_KP_MAX, RS03_KD_MAX}},
        {ActuatorType::ROBSTRIDE_04, {4 * M_PI, 15, 120, 5000.0, 100.0}},
        {ActuatorType::ROBSTRIDE_05, {4 * M_PI, 33, 17, 500.0, 5.0}},
        {ActuatorType::ROBSTRIDE_06, {4 * M_PI, 20, 36, 5000.0, 100.0}},
};

class RobStrideMotor {
public:
  RobStrideMotor(const std::string can_interface, uint8_t master_id,
                 uint8_t motor_id, int actuator_type)
      : iface(can_interface), master_id(master_id), motor_id(motor_id),
        actuator_type(actuator_type) {
    init_socket();
  }

  ~RobStrideMotor() {
    if (socket_fd >= 0)
      close(socket_fd);
  }

  const ActuatorOperation &limits() const {
    return ACTUATOR_OPERATION_MAPPING.at(static_cast<ActuatorType>(actuator_type));
  }

  ReceiveResult receive(double timeout_sec = 0.05) {
    struct timeval timeout{};
    timeout.tv_sec = static_cast<time_t>(timeout_sec);
    timeout.tv_usec = static_cast<suseconds_t>(
        (timeout_sec - static_cast<double>(timeout.tv_sec)) * 1e6);
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    struct can_frame frame{};
    ssize_t nbytes = recv(socket_fd, &frame, sizeof(struct can_frame), 0);
    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
      return std::nullopt;
    }
    if (!(frame.can_id & CAN_EFF_FLAG)) {
      return std::nullopt; // 只处理扩展帧
    }

    uint32_t can_id = frame.can_id & CAN_EFF_MASK;
    uint8_t communication_type = (can_id >> 24) & 0x1F;
    uint16_t extra_data = (can_id >> 8) & 0xFFFF;
    uint8_t host_id = can_id & 0xFF;

    error_code = static_cast<uint8_t>((can_id >> 16) & 0x3F);
    pattern = static_cast<uint8_t>((can_id >> 22) & 0x03);

#ifdef RS_MOTOR_DEBUG
    std::cout << "canid = 0x" << std::hex << std::uppercase << can_id
              << std::dec << std::endl;
    for (int i = 0; i < 8; ++i) {
      std::cout << "data[" << i << "] = 0x" << std::setw(2)
                << std::setfill('0') << std::hex << std::uppercase
                << static_cast<int>(frame.data[i]) << "  ";
    }
    std::cout << std::dec << std::endl;
#endif

    if (frame.can_dlc > 8) {
      return std::nullopt;
    }

#ifdef RS_MOTOR_DEBUG
    std::cerr << "[can rx] id=0x" << std::hex << std::uppercase
              << (frame.can_id & CAN_EFF_MASK) << std::dec
              << " type=" << static_cast<int>(communication_type)
              << " host=0x" << std::hex << static_cast<int>(host_id)
              << " dlc=" << std::dec << static_cast<int>(frame.can_dlc)
              << " data=";
    for (int i = 0; i < frame.can_dlc; ++i)
      std::cerr << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(frame.data[i]) << " ";
    std::cerr << std::dec << std::endl;
#endif

    std::vector<uint8_t> data(frame.data, frame.data + frame.can_dlc);
    return std::make_tuple(communication_type, extra_data, host_id, data);
  }

  std::tuple<float, float, float, float> return_data_pvtt() {
    return std::make_tuple(position_, velocity_, torque_, temperature_);
  }

  // 同步等待并解析应答帧；超时或非预期帧返回 false，不抛异常
  // expect_index 非 0 时，仅接受该参数索引的读取应答，错位帧直接丢弃
  bool receive_status_frame(double timeout_sec = 0.05,
                            uint16_t expect_index = 0);

  void init_socket();
  int send_frame(const struct can_frame &frame);

  // --- 单个参数读写（说明书通信类型 17 / 18） ---
  void write_parameter(uint16_t index, const void *value, uint8_t size);
  void write_parameter_u16(uint16_t index, uint16_t value);
  void Set_RobStrite_Motor_parameter(uint16_t Index, float Value,
                                     char Value_mode);
  // 返回是否收到并解析到读取应答（手册 4.1.8：应答 ID bit23~16 为 00 才算成功）
  bool Get_RobStrite_Motor_parameter(uint16_t Index);

  // --- 控制指令 ---
  std::tuple<float, float, float, float> enable_motor();
  void Disenable_Motor(uint8_t clear_error);
  std::tuple<float, float, float, float>
  send_motion_command(float torque, float position_rad, float velocity_rad_s,
                      float kp = 0.5f, float kd = 0.1f);
  // limit_cur_a：首次切入速度模式时写入的电流上限（0x7018，手册 0~43 A）。
  // 默认 27 A 保持历史行为；供电容量紧张的场合应显式传保守值。
  std::tuple<float, float, float, float>
  send_velocity_mode_command(float velocity_rad_s,
                             float limit_cur_a = 27.0f);
  std::tuple<float, float, float, float>
  RobStrite_Motor_PosPP_control(float Speed, float Acceleration, float Angle);
  std::tuple<float, float, float, float>
  RobStrite_Motor_PosCSP_control(float Speed, float Angle);
  std::tuple<float, float, float, float>
  RobStrite_Motor_Current_control(float IqCommand);

  // --- 说明书 4.1 中原有但实现不完整的功能 ---
  void set_zero_position();            // 类型 6：设置机械零位
  void Set_CAN_ID(uint8_t Set_CAN_ID); // 类型 7

  // --- 说明书 4.1 新增补齐的通信类型 ---
  void get_device_id();                       // 类型 0
  void read_firmware_version();               // 类型 4 + 0xC4
  void save_parameters();                     // 类型 22 (0x16)
  void set_baudrate(uint8_t baud_code);       // 类型 23 (0x17)
  void set_active_report(bool enable);        // 类型 24 (0x18)
  void set_motor_protocol(uint8_t protocol);  // 类型 25 (0x19)

  uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

  float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float denom = static_cast<float>((static_cast<uint32_t>(1) << bits) - 1);
    return static_cast<float>(x_int) * span / denom + x_min;
  }

  float Byte_to_float(const uint8_t *bytedata) {
    uint32_t data = (static_cast<uint32_t>(bytedata[7]) << 24) |
                    (static_cast<uint32_t>(bytedata[6]) << 16) |
                    (static_cast<uint32_t>(bytedata[5]) << 8) |
                    static_cast<uint32_t>(bytedata[4]);
    float data_float;
    std::memcpy(&data_float, &data, sizeof(float));
    return data_float;
  }

  float Byte_to_float(const std::vector<uint8_t> &bytedata) {
    if (bytedata.size() < 8)
      return 0.0f;
    return Byte_to_float(bytedata.data());
  }

private:
  // 内部 helper：失能态下写 run_mode 并回读同步本地状态（不切模式时勿调用，
  // 由 ensure_run_mode / set_zero_position 先判断）。原 5 个控制函数
  //（运控 / 速度 / PP / CSP / 电流）各自重复同一段流程，且 set_zero_position
  // 也内嵌了一份近乎相同的逻辑，统一收敛到这两个 helper。
  void write_run_mode(uint8_t mode);

  // 内部 helper：确保电机处于指定 run_mode。确实发生切换时返回 true，
  // 供速度模式判断是否补写 limit_cur / acc_rad。
  bool ensure_run_mode(uint8_t mode);

  // 内部 helper：发送一帧控制命令并等待应答，集中原先散落在 8+ 个函数里的
  // "send_frame -> 日志 -> receive_status_frame" 样板。ok_msg 为空时不打印。
  bool send_and_wait(const struct can_frame &frame,
                     const char *ok_msg = nullptr);

public:
  std::string iface;
  uint8_t master_id;
  uint8_t motor_id;
  int socket_fd = -1;
  data_read_write_one params;
  data_read_write drw;

  float position_ = 0.0f;
  float velocity_ = 0.0f;
  float torque_ = 0.0f;
  float temperature_ = 0.0f;

  uint8_t error_code = 0;   // 反馈帧 bit21~16 故障信息
  uint8_t pattern = 0;      // 反馈帧 bit23~22 模式状态
  uint32_t fault_code_ = 0; // 类型 21 故障反馈帧 fault
  uint32_t warning_code_ = 0;
  std::array<uint8_t, 8> mcu_uid_{};
  std::array<uint8_t, 4> firmware_version_{};
  bool uid_valid_ = false;
  bool version_valid_ = false;

  int actuator_type;
};

#endif // MOTOR_ROS2_MOTOR_CFG_H_
