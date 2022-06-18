// based on https://github.com/Malaphor/ODriveTeensyCAN

#include "hopper_can_interface/ODrivePCAN.h"
//#include "hopper_can_interface/PCANBasic.h"
#include <memory.h>  // for std::memcpy
#include <cstring>
#include "hopper_can_interface/can_interface.h"
static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;

static const int NodeIDLength = 6;
static const int CommandIDLength = 5;

static const float feedforwardFactor = 1 / 0.001;

// static const int CANBaudRate = 250000;

// Print with stream operator
// template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
// template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

CanInterface Can0;
// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can0;

ODrivePCAN::ODrivePCAN(int CANBaudRate) {
  this->CANBaudRate = CANBaudRate;
  Can0.begin();
  Can0.setBaudRate(CANBaudRate);
}

void ODrivePCAN::sendMessage(int axis_id, int cmd_id, bool remote_transmission_request, int length, uint8_t* signal_bytes) {
  CAN_message_t msg;
  CAN_message_t return_msg;

  msg.id = (axis_id << CommandIDLength) + cmd_id;
  msg.flags.remote = remote_transmission_request;
  msg.len = length;
  if (!remote_transmission_request && cmd_id != CMD_ID_GET_ADC_VOLTAGE) {
    std::memcpy(msg.buf, signal_bytes, length);
    Can0.write(msg);
    return;
  }

  if (cmd_id == CMD_ID_GET_ADC_VOLTAGE) {
    std::memcpy(msg.buf, signal_bytes, length);
    uint32_t return_id = (axis_id << CommandIDLength) + CMD_ID_SEND_ADC_VOLTAGE;

    Can0.write(msg);
    while (true) {
      if (Can0.read(return_msg) && (return_msg.id == return_id)) {
        std::memcpy(signal_bytes, return_msg.buf, sizeof(return_msg.buf));
        return;
      }
    }
  }

  Can0.write(msg);
  while (true) {
    if (Can0.read(return_msg) && (return_msg.id == msg.id)) {
      std::memcpy(signal_bytes, return_msg.buf, sizeof(return_msg.buf));
      return;
    }
  }
}

int ODrivePCAN::Heartbeat() {
  CAN_message_t return_msg;
  if (Can0.read(return_msg) == 1) {
    return (int)(return_msg.id >> 5);
  } else {
    return -1;
  }
}

void ODrivePCAN::SetAxisNodeId(int axis_id, int node_id) {
  uint8_t* node_id_b = (uint8_t*)&node_id;

  sendMessage(axis_id, CMD_ID_SET_AXIS_NODE_ID, false, 4, node_id_b);
}

void ODrivePCAN::SetControllerModes(int axis_id, int control_mode, int input_mode) {
  uint8_t* control_mode_b = (uint8_t*)&control_mode;
  uint8_t* input_mode_b = (uint8_t*)&input_mode;
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  msg_data[0] = control_mode_b[0];
  msg_data[1] = control_mode_b[1];
  msg_data[2] = control_mode_b[2];
  msg_data[3] = control_mode_b[3];
  msg_data[4] = input_mode_b[0];
  msg_data[5] = input_mode_b[1];
  msg_data[6] = input_mode_b[2];
  msg_data[7] = input_mode_b[3];

  sendMessage(axis_id, CMD_ID_SET_CONTROLLER_MODES, false, 8, msg_data);
}

void ODrivePCAN::SetPosition(int axis_id, float position) {
  SetPosition(axis_id, position, 0.0f, 0.0f);
}

void ODrivePCAN::SetPosition(int axis_id, float position, float velocity_feedforward) {
  SetPosition(axis_id, position, velocity_feedforward, 0.0f);
}

void ODrivePCAN::SetPosition(int axis_id, float position, float velocity_feedforward, float current_feedforward) {
  int16_t vel_ff = (int16_t)(feedforwardFactor * velocity_feedforward);
  int16_t curr_ff = (int16_t)(feedforwardFactor * current_feedforward);

  uint8_t* position_b = (uint8_t*)&position;
  uint8_t* velocity_feedforward_b = (uint8_t*)&vel_ff;
  uint8_t* current_feedforward_b = (uint8_t*)&curr_ff;
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  msg_data[0] = position_b[0];
  msg_data[1] = position_b[1];
  msg_data[2] = position_b[2];
  msg_data[3] = position_b[3];
  msg_data[4] = velocity_feedforward_b[0];
  msg_data[5] = velocity_feedforward_b[1];
  msg_data[6] = current_feedforward_b[0];
  msg_data[7] = current_feedforward_b[1];

  sendMessage(axis_id, CMD_ID_SET_INPUT_POS, false, 8, msg_data);
}

void ODrivePCAN::SetVelocity(int axis_id, float velocity) {
  SetVelocity(axis_id, velocity, 0.0f);
}

void ODrivePCAN::SetVelocity(int axis_id, float velocity, float current_feedforward) {
  uint8_t* velocity_b = (uint8_t*)&velocity;
  uint8_t* current_feedforward_b = (uint8_t*)&current_feedforward;
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  msg_data[0] = velocity_b[0];
  msg_data[1] = velocity_b[1];
  msg_data[2] = velocity_b[2];
  msg_data[3] = velocity_b[3];
  msg_data[4] = current_feedforward_b[0];
  msg_data[5] = current_feedforward_b[1];
  msg_data[6] = current_feedforward_b[2];
  msg_data[7] = current_feedforward_b[3];

  sendMessage(axis_id, CMD_ID_SET_INPUT_VEL, false, 8, msg_data);
}

void ODrivePCAN::SetTorque(int axis_id, float torque) {
  uint8_t* torque_b = (uint8_t*)&torque;

  sendMessage(axis_id, CMD_ID_SET_INPUT_TORQUE, false, 4, torque_b);
}

void ODrivePCAN::SetLimits(int axis_id, float velocity_limit, float current_limit) {
  uint8_t* velocity_limit_b = (uint8_t*)&velocity_limit;
  uint8_t* current_limit_b = (uint8_t*)&current_limit;
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  msg_data[0] = velocity_limit_b[0];
  msg_data[1] = velocity_limit_b[1];
  msg_data[2] = velocity_limit_b[2];
  msg_data[3] = velocity_limit_b[3];
  msg_data[4] = current_limit_b[0];
  msg_data[5] = current_limit_b[1];
  msg_data[6] = current_limit_b[2];
  msg_data[7] = current_limit_b[3];

  sendMessage(axis_id, CMD_ID_SET_LIMITS, false, 8, msg_data);
}

void ODrivePCAN::SetTrajVelLimit(int axis_id, float traj_vel_limit) {
  uint8_t* traj_vel_limit_b = (uint8_t*)&traj_vel_limit;

  sendMessage(axis_id, CMD_ID_SET_TRAJ_VEL_LIMIT, false, 4, traj_vel_limit_b);
}

void ODrivePCAN::SetTrajAccelLimits(int axis_id, float traj_accel_limit, float traj_decel_limit) {
  uint8_t* traj_accel_limit_b = (uint8_t*)&traj_accel_limit;
  uint8_t* traj_decel_limit_b = (uint8_t*)&traj_decel_limit;
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  msg_data[0] = traj_accel_limit_b[0];
  msg_data[1] = traj_accel_limit_b[1];
  msg_data[2] = traj_accel_limit_b[2];
  msg_data[3] = traj_accel_limit_b[3];
  msg_data[4] = traj_decel_limit_b[0];
  msg_data[5] = traj_decel_limit_b[1];
  msg_data[6] = traj_decel_limit_b[2];
  msg_data[7] = traj_decel_limit_b[3];

  sendMessage(axis_id, CMD_ID_SET_TRAJ_ACCEL_LIMITS, false, 8, msg_data);
}

void ODrivePCAN::SetTrajInertia(int axis_id, float traj_inertia) {
  uint8_t* traj_inertia_b = (uint8_t*)&traj_inertia;

  sendMessage(axis_id, CMD_ID_SET_TRAJ_INERTIA, false, 4, traj_inertia_b);
}

void ODrivePCAN::SetLinearCount(int axis_id, int linear_count) {
  uint8_t* linear_count_b = (uint8_t*)&linear_count;

  sendMessage(axis_id, CMD_ID_SET_LINEAR_COUNT, false, 4, linear_count_b);
}

void ODrivePCAN::SetPositionGain(int axis_id, float position_gain) {
  uint8_t* position_gain_b = (uint8_t*)&position_gain;

  sendMessage(axis_id, CMD_ID_SET_POS_GAIN, false, 4, position_gain_b);
}

void ODrivePCAN::SetVelocityGains(int axis_id, float velocity_gain, float velocity_integrator_gain) {
  uint8_t* velocity_gain_b = (uint8_t*)&velocity_gain;
  uint8_t* velocity_integrator_gain_b = (uint8_t*)&velocity_integrator_gain;
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  msg_data[0] = velocity_gain_b[0];
  msg_data[1] = velocity_gain_b[1];
  msg_data[2] = velocity_gain_b[2];
  msg_data[3] = velocity_gain_b[3];
  msg_data[4] = velocity_integrator_gain_b[0];
  msg_data[5] = velocity_integrator_gain_b[1];
  msg_data[6] = velocity_integrator_gain_b[2];
  msg_data[7] = velocity_integrator_gain_b[3];

  sendMessage(axis_id, CMD_ID_SET_VEL_GAINS, false, 8, msg_data);
}

//////////// Get functions ///////////

float ODrivePCAN::GetPosition(int axis_id) {
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_ENCODER_ESTIMATES, true, 8, msg_data);

  float output;
  *((uint8_t*)(&output) + 0) = msg_data[0];
  *((uint8_t*)(&output) + 1) = msg_data[1];
  *((uint8_t*)(&output) + 2) = msg_data[2];
  *((uint8_t*)(&output) + 3) = msg_data[3];
  return output;
}

float ODrivePCAN::GetVelocity(int axis_id) {
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_ENCODER_ESTIMATES, true, 8, msg_data);

  float output;
  *((uint8_t*)(&output) + 0) = msg_data[4];
  *((uint8_t*)(&output) + 1) = msg_data[5];
  *((uint8_t*)(&output) + 2) = msg_data[6];
  *((uint8_t*)(&output) + 3) = msg_data[7];
  return output;
}

int32_t ODrivePCAN::GetEncoderShadowCount(int axis_id) {
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_ENCODER_COUNT, true, 8, msg_data);

  int32_t output;
  *((uint8_t*)(&output) + 0) = msg_data[0];
  *((uint8_t*)(&output) + 1) = msg_data[1];
  *((uint8_t*)(&output) + 2) = msg_data[2];
  *((uint8_t*)(&output) + 3) = msg_data[3];
  return output;
}

int32_t ODrivePCAN::GetEncoderCountInCPR(int axis_id) {
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_ENCODER_COUNT, true, 8, msg_data);

  int32_t output;
  *((uint8_t*)(&output) + 0) = msg_data[4];
  *((uint8_t*)(&output) + 1) = msg_data[5];
  *((uint8_t*)(&output) + 2) = msg_data[6];
  *((uint8_t*)(&output) + 3) = msg_data[7];
  return output;
}

float ODrivePCAN::GetIqSetpoint(int axis_id) {
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_IQ, true, 8, msg_data);

  float output;
  *((uint8_t*)(&output) + 0) = msg_data[0];
  *((uint8_t*)(&output) + 1) = msg_data[1];
  *((uint8_t*)(&output) + 2) = msg_data[2];
  *((uint8_t*)(&output) + 3) = msg_data[3];
  return output;
}

float ODrivePCAN::GetIqMeasured(int axis_id) {
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_IQ, true, 8, msg_data);

  float output;
  *((uint8_t*)(&output) + 0) = msg_data[4];
  *((uint8_t*)(&output) + 1) = msg_data[5];
  *((uint8_t*)(&output) + 2) = msg_data[6];
  *((uint8_t*)(&output) + 3) = msg_data[7];
  return output;
}

float ODrivePCAN::GetSensorlessPosition(int axis_id) {
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_SENSORLESS_ESTIMATES, true, 8, msg_data);

  float output;
  *((uint8_t*)(&output) + 0) = msg_data[0];
  *((uint8_t*)(&output) + 1) = msg_data[1];
  *((uint8_t*)(&output) + 2) = msg_data[2];
  *((uint8_t*)(&output) + 3) = msg_data[3];
  return output;
}

float ODrivePCAN::GetSensorlessVelocity(int axis_id) {
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_SENSORLESS_ESTIMATES, true, 8, msg_data);

  float output;
  *((uint8_t*)(&output) + 0) = msg_data[4];
  *((uint8_t*)(&output) + 1) = msg_data[5];
  *((uint8_t*)(&output) + 2) = msg_data[6];
  *((uint8_t*)(&output) + 3) = msg_data[7];
  return output;
}

uint32_t ODrivePCAN::GetMotorError(int axis_id) {
  uint8_t msg_data[4] = {0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_MOTOR_ERROR, true, 4, msg_data);

  uint32_t output;
  *((uint8_t*)(&output) + 0) = msg_data[0];
  *((uint8_t*)(&output) + 1) = msg_data[1];
  *((uint8_t*)(&output) + 2) = msg_data[2];
  *((uint8_t*)(&output) + 3) = msg_data[3];
  return output;
}

uint32_t ODrivePCAN::GetEncoderError(int axis_id) {
  uint8_t msg_data[4] = {0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_ENCODER_ERROR, true, 4, msg_data);

  uint32_t output;
  *((uint8_t*)(&output) + 0) = msg_data[0];
  *((uint8_t*)(&output) + 1) = msg_data[1];
  *((uint8_t*)(&output) + 2) = msg_data[2];
  *((uint8_t*)(&output) + 3) = msg_data[3];
  return output;
}

uint32_t ODrivePCAN::GetAxisError(int axis_id) {
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t output;

  CAN_message_t return_msg;

  int msg_id = (axis_id << CommandIDLength) + CMD_ID_ODRIVE_HEARTBEAT_MESSAGE;

  while (true) {
    if (Can0.read(return_msg) && (return_msg.id == msg_id)) {
      std::memcpy(msg_data, return_msg.buf, sizeof(return_msg.buf));
      *((uint8_t*)(&output) + 0) = msg_data[0];
      *((uint8_t*)(&output) + 1) = msg_data[1];
      *((uint8_t*)(&output) + 2) = msg_data[2];
      *((uint8_t*)(&output) + 3) = msg_data[3];
      return output;
    }
  }
}

uint8_t ODrivePCAN::GetCurrentState(int axis_id) {
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t output;

  CAN_message_t return_msg;

  int msg_id = (axis_id << CommandIDLength) + CMD_ID_ODRIVE_HEARTBEAT_MESSAGE;

  while (true) {
    if (Can0.read(return_msg) && (return_msg.id == msg_id)) {
      std::memcpy(msg_data, return_msg.buf, sizeof(return_msg.buf));
      *((uint8_t*)(&output) + 0) = msg_data[4];
      return output;
    }
  }
}

float ODrivePCAN::GetVbusVoltage(int axis_id) {  // message can be sent to either axis
  uint8_t msg_data[4] = {0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_VBUS_VOLTAGE, true, 4, msg_data);

  float output;
  *((uint8_t*)(&output) + 0) = msg_data[0];
  *((uint8_t*)(&output) + 1) = msg_data[1];
  *((uint8_t*)(&output) + 2) = msg_data[2];
  *((uint8_t*)(&output) + 3) = msg_data[3];
  return output;
}

float ODrivePCAN::GetADCVoltage(int axis_id, uint8_t gpio_num) {
  uint8_t msg_data[4] = {0, 0, 0, 0};

  msg_data[0] = gpio_num;

  sendMessage(axis_id, CMD_ID_GET_ADC_VOLTAGE, false, 1, msg_data);  // RTR must be false!

  float output;  // TODO: make sure float is always 32 bit
  *((uint8_t*)(&output) + 0) = msg_data[0];
  *((uint8_t*)(&output) + 1) = msg_data[1];
  *((uint8_t*)(&output) + 2) = msg_data[2];
  *((uint8_t*)(&output) + 3) = msg_data[3];
  return output;
}

//////////// Other functions ///////////

void ODrivePCAN::Estop(int axis_id) {
  sendMessage(axis_id, CMD_ID_ODRIVE_ESTOP_MESSAGE, false, 0, 0);  // message requires no data, thus the 0, 0
}
void ODrivePCAN::StartAnticogging(int axis_id) {
  sendMessage(axis_id, CMD_ID_START_ANTICOGGING, false, 0, 0);  // message requires no data, thus the 0, 0
}
void ODrivePCAN::RebootOdrive(int axis_id) {  // message can be sent to either axis
  sendMessage(axis_id, CMD_ID_REBOOT_ODRIVE, false, 0, 0);
}
void ODrivePCAN::ClearErrors(int axis_id) {
  sendMessage(axis_id, CMD_ID_CLEAR_ERRORS, false, 0, 0);  // message requires no data, thus the 0, 0
}

//////////// State helper ///////////

bool ODrivePCAN::RunState(int axis_id, int requested_state) {
  sendMessage(axis_id, CMD_ID_SET_AXIS_REQUESTED_STATE, false, 4, (uint8_t*)&requested_state);
  return true;
}
