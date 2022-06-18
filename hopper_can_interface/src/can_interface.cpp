
#include "hopper_can_interface/can_interface.h"
#include "hopper_can_interface/libpcanfd.h"

void CanInterface::begin() {
  // fd_ = pcanfd_open();
  // err = pcanfd_set_init(fd_);
}

void CanInterface::setBaudRate(int CANBaudRate) {
  fd_.clock_Hz = CANBaudRate;
}

void CanInterface::write(CAN_message_t msg) {
  pcanfd_msg pcan_msg = convert_msg_to_pcan(msg);
  int err = pcanfd_send_msg(fd_, pcan_msg);
  // return err;
}

int CanInterface::read(CAN_message_t return_msg) {  // probably can't be void
  pcanfd_msg pcan_msg = convert_msg_to_pcan(return_msg);
  int err = pcanfd_recv_msg(fd_, pcan_msg);
  return err;
}

void CanInterface::close() {
  pcanfd_close(fd_);
}

pcanfd_msg CanInterface::convert_msg_to_pcan(CAN_message_t msg) {
  // first convert msg so pcanfd can interpret
  pcanfd_msg pcan_msg;
  pcan_msg.type = PCANFD_TYPE_CAN20_MSG;                     // CAN 2.0 vs CAN-FD
  pcan_msg.id = msg.id pcan_msg.data_len = uint16(msg.len);  // fingers crossed
  pcan_msg.buf = msg.data;
  // TODO: pcan_msg.timestamp ??
  return pcan_msg;
}