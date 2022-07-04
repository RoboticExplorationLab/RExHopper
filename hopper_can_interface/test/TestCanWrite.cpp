#include "hopper_can_interface/CanInterface.h"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "Benchmark.h"

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
  CanInterface can1(Channel::CAN1, BandRate::BAUD_1M);
  RepeatedTimer timer;
  can1.initialize();

  TPCANMsg msg;
  msg.ID = 0x22;
  msg.LEN = 8;
  msg.MSGTYPE = MsgType::MSG_STANDARD;

  bool transmitFirstMsg = false;
  while (true) {
    for (int i = 0; i < 8; i++) {
      ++msg.DATA[i];
      msg.DATA[i] %= 0x10;
    }

    can1.writeAsync(msg);

    std::cout << "Transmit ID: " << msg.ID << " LEN: " << static_cast<int>(msg.LEN) << " DATA: " << dataToHex(msg.DATA, msg.LEN)
              << "Ave: " << timer.getAverageInMilliseconds() << "\n";

    if (transmitFirstMsg)
      timer.endTimer();
    else
      transmitFirstMsg = true;

    timer.startTimer();

    std::this_thread::sleep_for(std::chrono::microseconds{1000});
  }
}