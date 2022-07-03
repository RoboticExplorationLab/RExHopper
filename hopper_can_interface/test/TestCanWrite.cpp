#include "hopper_can_interface/CanInterface.h"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

using namespace hopper::can;

int main() {
  CanInterface can1(Channel::CAN1, BandRate::BAUD_1M);
  uint8_t data[8] = {};

  const std::chrono::milliseconds duration{20};
  can1.initialize();

  while (true) {
    for (int i = 0; i < 8; i++) {
      ++data[i];
    }
    can1.write(0x22, data, 8);
    std::cout << "Write successed"
              << "\n";
    std::this_thread::sleep_for(duration);
  }
}