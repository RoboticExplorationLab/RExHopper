#pragma once

#include <cstdint>
#include <mutex>

namespace hopper {
namespace can {
class BufferedCanMsg final {
 public:
  void writeMsgToBuffer(const uint32_t id, const uint8_t* srcPtr, const uint8_t length);
  void readMsgFromBuffer(uint32_t& id, uint8_t* dstPtr, uint8_t& length) const;

  void lockAndWriteMsgToBuffer(const uint32_t id, const uint8_t* srcPtr, const uint8_t length);
  void lockAndReadMsgFromBuffer(uint32_t& id, uint8_t* dstPtr, uint8_t& length) const;

  std::mutex& getMutex() const noexcept { return m_; }

 private:
  uint32_t id_;
  uint8_t length_;
  uint8_t data_[8];
  mutable std::mutex m_;
};
}  // namespace can
}  // namespace hopper