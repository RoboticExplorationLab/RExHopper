#pragma once

#include <PCANBasic.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
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

enum MsgType : TPCANMessageType { MSG_STANDARD = PCAN_MESSAGE_STANDARD, MSG_RTR = PCAN_MESSAGE_RTR, MSG_EXTENDED = PCAN_MESSAGE_EXTENDED };

class CanInterface final {
 public:
  using Status = TPCANStatus;
  explicit CanInterface(const Channel channel, const BandRate bandRate);
  ~CanInterface();

  Status initialize();
  void write();

  TPCANMsg& getWriteMsgBuffer();

 private:
  std::string errorMessage(Status status);
  std::string toHex(int num);

  const Channel channel_;
  const BandRate bandRate_;

  /**
   * Transmitter and receiver threads
   */
  void writeWorker();
  void readWorker();

  /**
   * Thread instances. Move assigned after constructor to support virtual functions (Not use yet).
   */
  std::thread writeThread_;
  std::thread readThread_;

  /**
   *  Flag is protected by bufferedMsgReadyMutex_, but shared by read, write and main threads. Thus, atomic here.
   */
  std::atomic_bool keepRunning_;

  /**
   * Synchronization for transmitter
   */
  std::condition_variable bufferedMsgReadyCondition_;
  bool bufferedMsgReady_;
  std::mutex bufferedMsgReadyMutex_;
  std::unique_ptr<TPCANMsg> activeWriteMsgPtr_;
  std::unique_ptr<TPCANMsg> bufferedWriteMsgPtr_;

  /**
   *  Error message will be wrote here.
   */
  char errorMsgBuffer_[500];
};
}  // namespace can
}  // namespace hopper