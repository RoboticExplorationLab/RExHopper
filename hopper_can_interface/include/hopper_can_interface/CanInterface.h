#pragma once

#include <PCANBasic.h>

#include <string>
#include <thread>

namespace hopper {
namespace can {

enum Channel : TPCANHandle { CAN1 = PCAN_PCIBUS1, CAN2 = PCAN_PCIBUS2, CAN3 = PCAN_PCIBUS3, CAN4 = PCAN_PCIBUS4 };

enum BandRate : TPCANBaudrate {
  BAUD_1M = PCAN_BAUD_1M,
  BAUD_800K = PCAN_BAUD_800K,
  BAUD_500K = PCAN_BAUD_500K,
  BAUD_250K = PCAN_BAUD_250K,
  BAUD_125K = PCAN_BAUD_125K
};

class CanInterface {
 public:
  using Status = TPCANStatus;
  explicit CanInterface(const Channel channel, const BandRate bandRate);
  ~CanInterface();

  Status initialize();
  Status write();

 private:
  std::string errorMessage(Status status);
  const Channel channel_;
  const BandRate bandRate_;

  std::thread writeThread_;
  std::thread readThread_;
};
}  // namespace can
}  // namespace hopper