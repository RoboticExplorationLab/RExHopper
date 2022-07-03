#include "hopper_can_interface/CanInterface.h"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace hopper {
namespace can {

CanInterface::CanInterface(const Channel channel, const BandRate bandRate) : channel_(channel), bandRate_(bandRate), keepRunning_{true} {}

CanInterface::~CanInterface() {
  {
    std::lock_guard<std::mutex> lock(writeBuffer_.getMutex());
    keepRunning_ = false;
  }
  writeReadyCondition_.notify_all();

  if (writeThread_.joinable()) {
    writeThread_.join();
  }
  if (readThread_.joinable()) {
    readThread_.join();
  }

  Status status = CAN_Uninitialize(channel_);
  if (status != PCAN_ERROR_OK) {
    std::cerr << errorMessage(status) << std::endl;
    // throw std::runtime_error(errorMessage(status));
  }
}

CanInterface::Status CanInterface::initialize() {
  Status status = CAN_Initialize(channel_, bandRate_);

  if (status != PCAN_ERROR_OK) {
    throw std::runtime_error(errorMessage(status));
  }

  writeThread_ = std::thread(&CanInterface::writeWorker, this);
  // readThread_ = std::thread(&CanInterface::readWorker, this);

  return status;
}

void CanInterface::writeWorker() {
  TPCANMsg msgCanMessage;
  msgCanMessage.MSGTYPE = PCAN_MESSAGE_STANDARD;
  Status status;

  while (true) {
    {
      std::unique_lock<std::mutex> lock(writeBuffer_.getMutex());
      writeReadyCondition_.wait(lock, [this] { return writeMsgReady_ || !keepRunning_; });

      if (!keepRunning_) {
        break;
      }
      writeBuffer_.readMsgFromBuffer(msgCanMessage.ID, msgCanMessage.DATA, msgCanMessage.LEN);
      writeMsgReady_ = false;
    }

    status = CAN_Write(channel_, &msgCanMessage);
    if (status != PCAN_ERROR_OK) {
      std::cerr << errorMessage(status) << std::endl;
      // throw std::runtime_error("");
    };
  }
}

void CanInterface::readWorker() {
  while (keepRunning_) {
  }
}

CanInterface::Status CanInterface::write(const uint32_t id, const uint8_t* srcPtr, const uint8_t length) {
  std::lock_guard<std::mutex> lock(writeBuffer_.getMutex());
  writeBuffer_.writeMsgToBuffer(id, srcPtr, length);
  writeMsgReady_ = true;
  writeReadyCondition_.notify_all();

  return PCAN_ERROR_OK;
}

std::string CanInterface::errorMessage(Status error) {
  std::stringstream ss;

  if (CAN_GetErrorText(error, 0x09, errorMessagebBuffer_) != PCAN_ERROR_OK) {
    ss << "An error occurred. Error - code "
       << "0x" << std::uppercase << std::setfill('0') << std::setw(4) << std::hex << error << "could not be resolved\n";

    return ss.str();
  } else {
    ss << "Error: "
       << "0x" << std::uppercase << std::setfill('0') << std::setw(4) << std::hex << error << "  " << errorMessagebBuffer_ << "\n";

    return ss.str();
  }
}
}  // namespace can
}  // namespace hopper