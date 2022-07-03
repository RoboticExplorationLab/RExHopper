#include "hopper_can_interface/CanInterface.h"

#include <chrono>
#include <thread>

using namespace hopper::can;

int main() {
  CanInterface can1(Channel::CAN1, BandRate::BAUD_800K);

  const std::chrono::milliseconds duration{20};
  can1.initialize();

  while (true) {
    can1.write();

    std::this_thread::sleep_for(duration);
  }
}