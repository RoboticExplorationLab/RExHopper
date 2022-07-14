#include "hopper_can_interface/CanInterface.h"

#include <condition_variable>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

#include "Benchmark.h"
#include "hopper_can_interface/BufferedCanMsg.h"

using namespace hopper::can;

BufferedCanMsg msgBuffer;
std::condition_variable receivedCondition;
bool msgReceived = false;
std::mutex receivedMutex;

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

void worker() {
  bool receivedFirstMsg = false;
  RepeatedTimer timer;

  while (true) {
    {
      std::unique_lock<std::mutex> lock(receivedMutex);
      receivedCondition.wait(lock, []() { return msgReceived; });
      msgReceived = false;
    }

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

int main() {
  CanInterface can2(Channel::CAN2, BandRate::BAUD_1M);

  can2.subscribeTopic(0x22, [&](const TPCANMsg& msg) {
    msgBuffer.writeMsgToBuffer(msg);
    {
      std::lock_guard<std::mutex> lock(receivedMutex);
      msgReceived = true;
    }
    receivedCondition.notify_all();
  });

  can2.initialize();

  std::cout << "Start can..."
            << "\n";

  std::thread t(worker);

  t.join();
}