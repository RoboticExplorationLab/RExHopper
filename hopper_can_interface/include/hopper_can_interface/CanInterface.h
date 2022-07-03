#pragma once

#include <PCANBasic.h>

#include <atomic>
#include <condition_variable>
#include <string>
#include <thread>

#include "hopper_can_interface/BufferedDataArray.h"

namespace hopper {
namespace can {

enum Channel : TPCANHandle { CAN1 = PCAN_PCIBUS1, CAN2 = PCAN_PCIBUS2, CAN3 = PCAN_PCIBUS3, CAN4 = PCAN_PCIBUS4 };

enum BandRate : TPCANBaudrate {
  BAUD_1M = PCAN_BAUD_1M,
  BAUD_800K = PCAN_BAUD_800K,
  BAUD_500K = PCAN_BAUD_500K,
  BAUD_250K = PCAN_BAUD_250K,
  BAUD_125K = PCAN_BAUD_125K
};

class CanInterface final {
 public:
  using Status = TPCANStatus;
  explicit CanInterface(const Channel channel, const BandRate bandRate);
  ~CanInterface();

  Status initialize();
  Status write(const uint32_t id, const uint8_t* srcPtr, const uint8_t length);

 private:
  void writeWorker();
  void readWorker();

  std::atomic_bool keepRunning_;
  std::condition_variable writeReadyCondition_;
  bool writeMsgReady_;

  std::string errorMessage(Status status);

  const Channel channel_;
  const BandRate bandRate_;

  char errorMessagebBuffer_[500];

  std::thread writeThread_;
  std::thread readThread_;

  BufferedCanMsg writeBuffer_;
};
}  // namespace can
}  // namespace hopper