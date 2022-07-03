#include "hopper_can_interface/CanInterface.h"

#include <cstdint>
#include <iostream>
#include <thread>

#include "Benchmark.h"

using namespace hopper::can;

int main() {
  CanInterface can1(Channel::CAN1, BandRate::BAUD_1M);
  uint8_t data[8] = {0};
  RepeatedTimer timer;
  can1.initialize();

  TPCANMsg msg;
  msg.ID = 0x22;
  msg.LEN = 8;
  msg.MSGTYPE = MsgType::MSG_STANDARD;

  can1.getWriteMsgBuffer().ID = 0x22;
  can1.getWriteMsgBuffer().LEN = 8;
  can1.getWriteMsgBuffer().MSGTYPE = MsgType::MSG_STANDARD;

  while (true) {
    can1.getWriteMsgBuffer().ID = 0x22;
    can1.getWriteMsgBuffer().LEN = 8;
    can1.getWriteMsgBuffer().MSGTYPE = MsgType::MSG_STANDARD;
    for (int i = 0; i < 8; i++) {
      ++data[i];
      data[i] %= 0x10;
      msg.DATA[i] = data[i];
      can1.getWriteMsgBuffer().DATA[i] = data[i];
    }
    timer.startTimer();
    can1.write();
    // CAN_Write(Channel::CAN1, &msg);
    timer.endTimer();

    std::cerr << "Ave: " << timer.getAverageInMilliseconds() << "Max: " << timer.getMaxIntervalInMilliseconds() << "\n";

    std::this_thread::sleep_for(std::chrono::microseconds{200});
  }
}