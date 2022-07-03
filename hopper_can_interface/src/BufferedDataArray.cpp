#include "hopper_can_interface/BufferedDataArray.h"

#include <cstring>

namespace hopper {
namespace can {

void BufferedCanMsg::writeMsgToBuffer(const uint32_t id, const uint8_t* srcPtr, const uint8_t length) {
  id_ = id;
  length_ = length;
  std::memcpy(data_, srcPtr, length);
}

void BufferedCanMsg::readMsgFromBuffer(uint32_t& id, uint8_t* dstPtr, uint8_t& length) const {
  id = id_;
  length = length_;
  std::memcpy(dstPtr, data_, length_);
}

void BufferedCanMsg::lockAndWriteMsgToBuffer(const uint32_t id, const uint8_t* srcPtr, const uint8_t length) {
  std::lock_guard<std::mutex> lock(m_);
  id_ = id;
  length_ = length;
  std::memcpy(data_, srcPtr, length);
}

void BufferedCanMsg::lockAndReadMsgFromBuffer(uint32_t& id, uint8_t* dstPtr, uint8_t& length) const {
  std::lock_guard<std::mutex> lock(m_);
  id = id_;
  length = length_;
  std::memcpy(dstPtr, data_, length_);
}

}  // namespace can
}  // namespace hopper