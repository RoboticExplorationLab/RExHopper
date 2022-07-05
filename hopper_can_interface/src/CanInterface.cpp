#include "hopper_can_interface/CanInterface.h"

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace hopper {
namespace can {

CanInterface::CanInterface(const Channel channel, const BandRate bandRate) : channel_(channel), bandRate_(bandRate) {
  // Allocate buffer
  activeWriteMsgPtr_.reset(new TPCANMsg);
  bufferedWriteMsgPtr_.reset(new TPCANMsg);

  Status status;
  CAN_GetValue(channel, PCAN_CHANNEL_CONDITION, &status, sizeof(status));

  if (status != PCAN_CHANNEL_AVAILABLE) {
    std::stringstream ss;
    ss << "[CanInterface::CanInterface] PCAN device " << toHex(channel_) << " is ";
    if (status == PCAN_CHANNEL_UNAVAILABLE) {
      ss << "UNAVAILABLE";
    } else {
      ss << "OCCUPIED";
    }
    throw std::runtime_error(ss.str());
  }
}

CanInterface::~CanInterface() {
  {
    std::lock_guard<std::mutex> lock(bufferedMsgReadyMutex_);
    keepRunning_ = false;
  }
  bufferedMsgReadyCondition_.notify_all();

  if (writeThread_.joinable()) {
    writeThread_.join();
  }
  if (readThread_.joinable()) {
    readThread_.join();
  }

  CAN_Uninitialize(channel_);
}

CanInterface::Status CanInterface::initialize() {
  Status status = CAN_Initialize(channel_, bandRate_);

  if (status != PCAN_ERROR_OK) {
    throw std::runtime_error(errorMessage(status));
  }

  writeThread_ = std::thread(&CanInterface::writeWorker, this);
  readThread_ = std::thread(&CanInterface::readWorker, this);

  return status;
}

void CanInterface::writeWorker() {
  Status status;
  while (true) {
    {
      std::unique_lock<std::mutex> lock(bufferedMsgReadyMutex_);
      bufferedMsgReadyCondition_.wait(lock, [this] { return bufferedMsgReady_ || !keepRunning_; });

      if (!keepRunning_) {
        break;
      }

      activeWriteMsgPtr_.swap(bufferedWriteMsgPtr_);
      bufferedMsgReady_ = false;
    }

    while (true) {
      status = CAN_Write(channel_, activeWriteMsgPtr_.get());

      if (status == PCAN_ERROR_OK) {
        break;
      } else {
        std::lock_guard<std::mutex> lock(bufferedMsgReadyMutex_);
        if (bufferedMsgReady_) {
          break;
        }
      }

      // NOTE: If the transmit buffer is full, sleep for 200microseconds and resend.
      std::cerr << "\033[1;31mTransmit Buffer full.\033[0m\n";
      std::this_thread::sleep_for(std::chrono::microseconds{500});
    }
  }
}

void CanInterface::readWorker() {
  TPCANMsg msgBuffer;
  Status status;
  using clock = std::chrono::high_resolution_clock;
  const std::chrono::microseconds duration{500};
  const auto dt = std::chrono::duration_cast<clock::duration>(duration);
  const auto start = clock::now();
  auto sleepTill = start;

  while (keepRunning_) {
    do {
      status = CAN_Read(channel_, &msgBuffer, nullptr);

      if (status == PCAN_ERROR_OK) {  // If received
        auto iter = idCallbackMap_.find(msgBuffer.ID);
        if (iter != idCallbackMap_.end()) {
          iter->second(msgBuffer);
        }
      } else if (status != PCAN_ERROR_QRCVEMPTY) {  // If error except receive buffer is empty
        std::string errorMsg = std::string("\033[1;31m[CanInterface::readWorker] ") + errorMessage(status);
        std::cerr << errorMsg << "\033[0m\n";
      }
    } while (status == PCAN_ERROR_OK);

    // Sleep
    sleepTill += dt;
    std::this_thread::sleep_until(sleepTill);
  }
}

void CanInterface::writeAsync(TPCANMsg& msg) {
  {
    std::lock_guard<std::mutex> lock(bufferedMsgReadyMutex_);
    *bufferedWriteMsgPtr_ = msg;
    bufferedMsgReady_ = true;
  }
  bufferedMsgReadyCondition_.notify_all();
}

void CanInterface::write(TPCANMsg& msg) {
  Status status = CAN_Write(channel_, &msg);
  if (status != PCAN_ERROR_OK) {
    std::string errorMsg = std::string("[CanInterface::write] ") + errorMessage(status);
    throw std::runtime_error(errorMsg);
  }
}

void CanInterface::subscribeTopic(uint32_t id, Callback callback) {
  idCallbackMap_.emplace(id, std::move(callback));
}

inline std::string CanInterface::toHex(int num) const {
  std::stringstream ss;
  ss << "0x" << std::uppercase << std::setfill('0') << std::setw(4) << std::hex << num;
  return ss.str();
}

std::string CanInterface::dataToHex(uint8_t* data, size_t len) {
  std::stringstream ss;
  for (int i = 0; i < len; ++i) {
    ss << toHex(data[i]) << " ";
  }
  return ss.str();
}

std::string CanInterface::errorMessage(Status error) {
  // No error - return
  if (error == PCAN_ERROR_OK) return "";

  std::stringstream ss;
  if (CAN_GetErrorText(error, 0x09, errorMsgBuffer_) != PCAN_ERROR_OK) {
    ss << "An error occurred. Error - code " << toHex(error) << "could not be resolved\n";
    return ss.str();
  } else {
    ss << "Error: " << toHex(error) << "  " << errorMsgBuffer_ << "\n";
    return ss.str();
  }
}
}  // namespace can
}  // namespace hopper