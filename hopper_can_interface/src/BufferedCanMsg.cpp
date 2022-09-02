#include "hopper_can_interface/BufferedCanMsg.h"

#include <cstring>

namespace hopper {
namespace can {

BufferedCanMsg::BufferedCanMsg() : bufferReady_{false}, missCounter_{0} {
  bufferedMsgPtr_.reset(new TPCANMsg);
  activeMsgPtr_.reset(new TPCANMsg);
}
void BufferedCanMsg::writeMsgToBuffer(const TPCANMsg& src) {
  std::lock_guard<std::mutex> lock(m_);
  *bufferedMsgPtr_ = src;
  if (bufferReady_) ++missCounter_;
  bufferReady_ = true;
}

void BufferedCanMsg::updateFromBuffer() {
  std::lock_guard<std::mutex> lock(m_);
  activeMsgPtr_.swap(bufferedMsgPtr_);
  bufferReady_ = false;
}

}  // namespace can
}  // namespace hopper