#include "hopper_can_interface/CanInterface.h"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "Benchmark.h"
#include "hopper_can_interface/BufferedCanMsg.h"

using namespace hopper::can;

std::string toHex(int num) {
  std::stringstream ss;
  ss << "0x" << std::uppercase << std::setfill('0') << std::setw(4) << std::hex << num;
  return ss.str();
}

std::string dataToHex(uint8_t* data, size_t len) {
  std::stringstream ss;
  for (int i = 0; i < len; ++i) {
    ss << toHex(data[i]) << " ";
  }
  return ss.str();
}

int main() {
  CanInterface can2(Channel::CAN2, BandRate::BAUD_1M);

  RepeatedTimer timer;

  BufferedCanMsg msgBuffer;

  can2.subscribeTopic(0x22, [&msgBuffer](const TPCANMsg& msg) {
    msgBuffer.writeMsgToBuffer(msg);
  });

  can2.initialize();

  std::cout << "Start can..."
            << "\n";

  bool receivedFirstMsg = false;
  while (true) {
    if (msgBuffer.bufferReady()) {
      if (receivedFirstMsg)
        timer.endTimer();
      else
        receivedFirstMsg = true;

      msgBuffer.updateFromBuffer();

      std::cout << "Receive ID: " << msgBuffer.get().ID << " LEN: " << static_cast<int>(msgBuffer.get().LEN)
                << " DATA: " << dataToHex(msgBuffer.get().DATA, msgBuffer.get().LEN) << "Ave: " << timer.getAverageInMilliseconds()
                << " missCounter: " << msgBuffer.getMissCounter() << "\n";

      timer.startTimer();
    }
  }
}