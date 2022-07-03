#include "hopper_can_interface/CanInterface.h"

#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace hopper {
namespace can {

CanInterface::CanInterface(const Channel channel, const BandRate bandRate) : channel_(channel), bandRate_(bandRate){};

CanInterface::~CanInterface() {
  if (writeThread_.joinable()) {
    writeThread_.join();
  }
  if (readThread_.joinable()) {
    readThread_.join();
  }
}

CanInterface::Status CanInterface::initialize() {
  Status status = CAN_Initialize(channel_, bandRate_);

  if (status != PCAN_ERROR_OK) {
    throw std::runtime_error(errorMessage(status));
  };

  return status;
}

CanInterface::Status CanInterface::write() {
  TPCANMsg msgCanMessage;
  msgCanMessage.ID = 0x01;
  msgCanMessage.LEN = 8;
  msgCanMessage.MSGTYPE = PCAN_MESSAGE_ECHO;
  for (int i = 0; i < 8; i++) {
    msgCanMessage.DATA[i] = i;
  }
  Status status = CAN_Write(channel_, &msgCanMessage);

  if (status != PCAN_ERROR_OK) {
    throw std::runtime_error(errorMessage(status));
  };

  return status;
}

std::string CanInterface::errorMessage(Status error) {
  char buffer[500];
  std::stringstream ss;

  if (CAN_GetErrorText(error, 0x09, buffer) != PCAN_ERROR_OK) {
    ss << "An error occurred. Error - code "
       << "0x" << std::uppercase << std::setfill('0') << std::setw(4) << std::hex << error << "could not be resolved\n";

    return ss.str();
  } else {
    ss << "Error: "
       << "0x" << std::uppercase << std::setfill('0') << std::setw(4) << std::hex << error << "  " << buffer << "\n";

    return ss.str();
  }
}
}  // namespace can
}  // namespace hopper