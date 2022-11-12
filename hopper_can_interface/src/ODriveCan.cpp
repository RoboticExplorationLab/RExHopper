#include "hopper_can_interface/ODriveCan.h"

#include <algorithm>
#include <cstring>

namespace hopper {
namespace can {
namespace {
constexpr float feedforwardFactor = 1 / 0.001;
}  // namespace

ODriveCan::ODriveCan(const Channel channel, const BandRate bandRate) : Base(channel, bandRate) {}

void ODriveCan::initialize(const std::map<std::string, int>& actuatorNameToNodeID) {
  actuatorNameToNodeID_ = actuatorNameToNodeID;

  actuatorsStatus_.reserve(actuatorNameToNodeID_.size());

  encoderEstimate_.reserve(actuatorNameToNodeID_.size());

  for (auto itr = actuatorNameToNodeID_.cbegin(); itr != actuatorNameToNodeID_.cend(); itr++) {
    int node_id = itr->second;
    subscribeTopic(encodeCanMsgID(itr->second, CMD_ID_ODRIVE_HEARTBEAT_MESSAGE),
                   [node_id, this](const TPCANMsg& msg) { heartBeatCallback(node_id, msg); });  // Heartbeat

    subscribeTopic(encodeCanMsgID(itr->second, CMD_ID_GET_ENCODER_ESTIMATES),
                   [node_id, this](const TPCANMsg& msg) { encoderEstimateCallback(node_id, msg); });  // EncoderEstimate

    EncoderEstimate eEstimate{};
    eEstimate.node_id = itr->second;
    encoderEstimate_.push_back(eEstimate);

    ActuatorStatus aStatus{};
    aStatus.node_id = itr->second;
    actuatorsStatus_.push_back(aStatus);
  }
  // Forward interface
  Base::initialize();
}

void ODriveCan::sendMessage(int axis_id, int cmd_id, bool remote_transmission_request, int length, uint8_t* signal_bytes) {
  TPCANMsg msg;
  TPCANMsg return_msg;

  msg.ID = (axis_id << 5U) | cmd_id;
  msg.MSGTYPE = remote_transmission_request ? MsgType::MSG_RTR : MsgType::MSG_STANDARD;
  msg.LEN = length;
  std::memcpy(msg.DATA, signal_bytes, length);
  write(msg);
}

// int ODriveCan::Heartbeat() {
//   CAN_message_t return_msg;
//   if (Can0.read(return_msg) == 1) {
//     return (int)(return_msg.id >> 5);
//   } else {
//     return -1;
//   }
// }

void ODriveCan::SetAxisNodeId(int axis_id, int node_id) {
  uint8_t* node_id_b = (uint8_t*)&node_id;

  sendMessage(axis_id, CMD_ID_SET_AXIS_NODE_ID, false, 4, node_id_b);
}

void ODriveCan::SetControllerModes(int axis_id, int control_mode, int input_mode) {
  uint8_t* control_mode_b = (uint8_t*)&control_mode;
  uint8_t* input_mode_b = (uint8_t*)&input_mode;
  uint8_t msg_data[8] = {0};

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

void ODriveCan::SetPosition(int axis_id, float position) {
  SetPosition(axis_id, position, 0.0f, 0.0f);
}

void ODriveCan::SetPosition(int axis_id, float position, float velocity_feedforward) {
  SetPosition(axis_id, position, velocity_feedforward, 0.0f);
}

void ODriveCan::SetPosition(int axis_id, float position, float velocity_feedforward, float torque_feedforward) {
  int16_t vel_ff = (int16_t)(feedforwardFactor * velocity_feedforward);
  int16_t curr_ff = (int16_t)(feedforwardFactor * torque_feedforward);

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

void ODriveCan::SetVelocity(int axis_id, float velocity, float torque_feedforward) {
  uint8_t* velocity_b = (uint8_t*)&velocity;
  uint8_t* current_feedforward_b = (uint8_t*)&torque_feedforward;
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

void ODriveCan::SetTorque(int axis_id, float torque) {
  uint8_t* torque_b = (uint8_t*)&torque;

  sendMessage(axis_id, CMD_ID_SET_INPUT_TORQUE, false, 4, torque_b);
}

void ODriveCan::SetLimits(int axis_id, float velocity_limit, float current_limit) {
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

void ODriveCan::SetTrajVelLimit(int axis_id, float traj_vel_limit) {
  uint8_t* traj_vel_limit_b = (uint8_t*)&traj_vel_limit;

  sendMessage(axis_id, CMD_ID_SET_TRAJ_VEL_LIMIT, false, 4, traj_vel_limit_b);
}

void ODriveCan::SetTrajAccelLimits(int axis_id, float traj_accel_limit, float traj_decel_limit) {
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

void ODriveCan::SetTrajInertia(int axis_id, float traj_inertia) {
  uint8_t* traj_inertia_b = (uint8_t*)&traj_inertia;

  sendMessage(axis_id, CMD_ID_SET_TRAJ_INERTIA, false, 4, traj_inertia_b);
}

void ODriveCan::SetLinearCount(int axis_id, int linear_count) {
  uint8_t* linear_count_b = (uint8_t*)&linear_count;

  sendMessage(axis_id, CMD_ID_SET_LINEAR_COUNT, false, 4, linear_count_b);
}

void ODriveCan::SetPositionGain(int axis_id, float position_gain) {
  uint8_t* position_gain_b = (uint8_t*)&position_gain;

  sendMessage(axis_id, CMD_ID_SET_POS_GAIN, false, 4, position_gain_b);
}

void ODriveCan::SetVelocityGains(int axis_id, float velocity_gain, float velocity_integrator_gain) {
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

// //////////// Callback functions ///////////

void ODriveCan::heartBeatCallback(int node_id, const TPCANMsg& msg) {
  auto itr = std::find_if(actuatorsStatus_.begin(), actuatorsStatus_.end(), [&](const ActuatorStatus& s) { return s.node_id == node_id; });
  if (itr == actuatorsStatus_.end()) {
    return;
  } else {
    itr->axisError = *((uint32_t*)&msg.DATA[0]);
    itr->axisState = *((uint8_t*)&msg.DATA[4]);
  }
}

void ODriveCan::encoderEstimateCallback(int node_id, const TPCANMsg& msg) {
  auto itr = encoderEstimate_.begin();
  for (; itr != encoderEstimate_.end(); itr++) {
    if (itr->node_id == node_id) {
      break;
    }
  }
  if (itr == encoderEstimate_.end()) {
    throw std::runtime_error("[OdriveCan::encoderEstimateCallback] Error: Cannot find encoder node_id, id = " + std::to_string(node_id));
  } else {
    itr->pos = *((float*)&msg.DATA[0]);
    itr->vel = *((float*)&msg.DATA[4]);
  }
}

// //////////// Get functions ///////////

float ODriveCan::GetPosition(int node_id) {
  auto itr = encoderEstimate_.begin();
  for (; itr != encoderEstimate_.end(); itr++) {
    if (itr->node_id == node_id) {
      break;
    }
  }
  if (itr == encoderEstimate_.end()) {
    throw std::runtime_error("[OdriveCan::GetPosition] Error: Cannot find encoder node_id");
  } else {
    return itr->pos;
  }
}

float ODriveCan::GetVelocity(int node_id) {
  auto itr = encoderEstimate_.begin();
  for (; itr != encoderEstimate_.end(); itr++) {
    if (itr->node_id == node_id) {
      break;
    }
  }
  if (itr == encoderEstimate_.end()) {
    throw std::runtime_error("[OdriveCan::GetVelocity] Error: Cannot find encoder node_id");
  } else {
    return itr->vel;
  }
}

// int32_t ODriveCan::GetEncoderShadowCount(int axis_id) {
//   uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//   sendMessage(axis_id, CMD_ID_GET_ENCODER_COUNT, true, 8, msg_data);

//   int32_t output;
//   *((uint8_t*)(&output) + 0) = msg_data[0];
//   *((uint8_t*)(&output) + 1) = msg_data[1];
//   *((uint8_t*)(&output) + 2) = msg_data[2];
//   *((uint8_t*)(&output) + 3) = msg_data[3];
//   return output;
// }

// int32_t ODriveCan::GetEncoderCountInCPR(int axis_id) {
//   uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//   sendMessage(axis_id, CMD_ID_GET_ENCODER_COUNT, true, 8, msg_data);

//   int32_t output;
//   *((uint8_t*)(&output) + 0) = msg_data[4];
//   *((uint8_t*)(&output) + 1) = msg_data[5];
//   *((uint8_t*)(&output) + 2) = msg_data[6];
//   *((uint8_t*)(&output) + 3) = msg_data[7];
//   return output;
// }

// float ODriveCan::GetIqSetpoint(int axis_id) {
//   uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//   sendMessage(axis_id, CMD_ID_GET_IQ, true, 8, msg_data);

//   float_t output;
//   *((uint8_t*)(&output) + 0) = msg_data[0];
//   *((uint8_t*)(&output) + 1) = msg_data[1];
//   *((uint8_t*)(&output) + 2) = msg_data[2];
//   *((uint8_t*)(&output) + 3) = msg_data[3];
//   return output;
// }

float ODriveCan::GetIqMeasured(int axis_id) {
  uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  sendMessage(axis_id, CMD_ID_GET_IQ, true, 8, msg_data);

  float output;
  *((uint8_t*)(&output) + 0) = msg_data[4];
  *((uint8_t*)(&output) + 1) = msg_data[5];
  *((uint8_t*)(&output) + 2) = msg_data[6];
  *((uint8_t*)(&output) + 3) = msg_data[7];
  return output;
}

// float ODriveCan::GetSensorlessPosition(int axis_id) {
//   uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//   sendMessage(axis_id, CMD_ID_GET_SENSORLESS_ESTIMATES, true, 8, msg_data);

//   float_t output;
//   *((uint8_t*)(&output) + 0) = msg_data[0];
//   *((uint8_t*)(&output) + 1) = msg_data[1];
//   *((uint8_t*)(&output) + 2) = msg_data[2];
//   *((uint8_t*)(&output) + 3) = msg_data[3];
//   return output;
// }

// float ODriveCan::GetSensorlessVelocity(int axis_id) {
//   uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//   sendMessage(axis_id, CMD_ID_GET_SENSORLESS_ESTIMATES, true, 8, msg_data);

//   float_t output;
//   *((uint8_t*)(&output) + 0) = msg_data[4];
//   *((uint8_t*)(&output) + 1) = msg_data[5];
//   *((uint8_t*)(&output) + 2) = msg_data[6];
//   *((uint8_t*)(&output) + 3) = msg_data[7];
//   return output;
// }

// uint32_t ODriveCan::GetMotorError(int axis_id) {
//   uint8_t msg_data[4] = {0, 0, 0, 0};

//   sendMessage(axis_id, CMD_ID_GET_MOTOR_ERROR, true, 4, msg_data);

//   uint32_t output;
//   *((uint8_t*)(&output) + 0) = msg_data[0];
//   *((uint8_t*)(&output) + 1) = msg_data[1];
//   *((uint8_t*)(&output) + 2) = msg_data[2];
//   *((uint8_t*)(&output) + 3) = msg_data[3];
//   return output;
// }

// uint32_t ODriveCan::GetEncoderError(int axis_id) {
//   uint8_t msg_data[4] = {0, 0, 0, 0};

//   sendMessage(axis_id, CMD_ID_GET_ENCODER_ERROR, true, 4, msg_data);

//   uint32_t output;
//   *((uint8_t*)(&output) + 0) = msg_data[0];
//   *((uint8_t*)(&output) + 1) = msg_data[1];
//   *((uint8_t*)(&output) + 2) = msg_data[2];
//   *((uint8_t*)(&output) + 3) = msg_data[3];
//   return output;
// }

// uint32_t ODriveCan::GetAxisError(int axis_id) {
//   uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//   uint32_t output;

//   CAN_message_t return_msg;

//   int msg_id = (axis_id << CommandIDLength) + CMD_ID_ODRIVE_HEARTBEAT_MESSAGE;

//   while (true) {
//     if (Can0.read(return_msg) && (return_msg.id == msg_id)) {
//       memcpy(msg_data, return_msg.buf, sizeof(return_msg.buf));
//       *((uint8_t*)(&output) + 0) = msg_data[0];
//       *((uint8_t*)(&output) + 1) = msg_data[1];
//       *((uint8_t*)(&output) + 2) = msg_data[2];
//       *((uint8_t*)(&output) + 3) = msg_data[3];
//       return output;
//     }
//   }
// }

// uint8_t ODriveCan::GetCurrentState(int axis_id) {
//   uint8_t msg_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//   uint8_t output;

//   CAN_message_t return_msg;

//   int msg_id = (axis_id << CommandIDLength) + CMD_ID_ODRIVE_HEARTBEAT_MESSAGE;

//   while (true) {
//     if (Can0.read(return_msg) && (return_msg.id == msg_id)) {
//       memcpy(msg_data, return_msg.buf, sizeof(return_msg.buf));
//       *((uint8_t*)(&output) + 0) = msg_data[4];
//       return output;
//     }
//   }
// }

// float ODriveCan::GetVbusVoltage(int axis_id) {  // message can be sent to either axis
//   uint8_t msg_data[4] = {0, 0, 0, 0};

//   sendMessage(axis_id, CMD_ID_GET_VBUS_VOLTAGE, true, 4, msg_data);

//   float_t output;
//   *((uint8_t*)(&output) + 0) = msg_data[0];
//   *((uint8_t*)(&output) + 1) = msg_data[1];
//   *((uint8_t*)(&output) + 2) = msg_data[2];
//   *((uint8_t*)(&output) + 3) = msg_data[3];
//   return output;
// }

// float ODriveCan::GetADCVoltage(int axis_id, uint8_t gpio_num) {
//   uint8_t msg_data[4] = {0, 0, 0, 0};

//   msg_data[0] = gpio_num;

//   sendMessage(axis_id, CMD_ID_GET_ADC_VOLTAGE, false, 1, msg_data);  // RTR must be false!

//   float_t output;
//   *((uint8_t*)(&output) + 0) = msg_data[0];
//   *((uint8_t*)(&output) + 1) = msg_data[1];
//   *((uint8_t*)(&output) + 2) = msg_data[2];
//   *((uint8_t*)(&output) + 3) = msg_data[3];
//   return output;
// }

// //////////// Other functions ///////////

// void ODriveCan::Estop(int axis_id) {
//   sendMessage(axis_id, CMD_ID_ODRIVE_ESTOP_MESSAGE, false, 0, 0);  // message requires no data, thus the 0, 0
// }
// void ODriveCan::StartAnticogging(int axis_id) {
//   sendMessage(axis_id, CMD_ID_START_ANTICOGGING, false, 0, 0);  // message requires no data, thus the 0, 0
// }
// void ODriveCan::RebootOdrive(int axis_id) {  // message can be sent to either axis
//   sendMessage(axis_id, CMD_ID_REBOOT_ODRIVE, false, 0, 0);
// }
// void ODriveCan::ClearErrors(int axis_id) {
//   sendMessage(axis_id, CMD_ID_CLEAR_ERRORS, false, 0, 0);  // message requires no data, thus the 0, 0
// }

// //////////// State helper ///////////

bool ODriveCan::RunState(int axis_id, int requested_state) {
  sendMessage(axis_id, CMD_ID_SET_AXIS_REQUESTED_STATE, false, 4, (uint8_t*)&requested_state);
  return true;
}

}  // namespace can
}  // namespace hopper