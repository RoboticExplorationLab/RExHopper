#pragma once

#include <PCANBasic.h>

#include <memory>
#include <mutex>
#include <atomic>

namespace hopper {
namespace can {
class BufferedCanMsg final {
 public:
  BufferedCanMsg();
  void writeMsgToBuffer(const TPCANMsg& src);
  void updateFromBuffer();

  TPCANMsg& get() { return *activeMsgPtr_; }

  const int getMissCounter() { return missCounter_; }
  const bool bufferReady() { return bufferReady_; }

 private:
  std::atomic_bool bufferReady_;
  std::atomic_int missCounter_;

  std::unique_ptr<TPCANMsg> activeMsgPtr_;
  std::unique_ptr<TPCANMsg> bufferedMsgPtr_;
  mutable std::mutex m_;
};
}  // namespace can
}  // namespace hopper