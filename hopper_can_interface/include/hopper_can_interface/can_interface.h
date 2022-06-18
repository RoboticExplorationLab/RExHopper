#pragma once
#include "hopper_can_interface/libpcanfd.h"  //#include <libpcanfd.h> 
#include <inttypes.h>
//namespace can_interface {

typedef struct CAN_message_t { // From FlexCAN_T4
  uint32_t id = 0;          // can identifier
  uint16_t timestamp = 0;   // FlexCAN time when message arrived
  uint8_t idhit = 0; // filter that id came from
  struct {
    bool extended = 0; // identifier is extended (29-bit)
    bool remote = 0;  // remote transmission request packet type
    bool overrun = 0; // message overrun
    bool reserved = 0;
  } flags;
  uint8_t len = 8;      // length of data
  uint8_t buf[8] = { 0 };       // data
  int8_t mb = 0;       // used to identify mailbox reception
  uint8_t bus = 0;      // used to identify where the message came from when events() is used.
  bool seq = 0;         // sequential frames
} CAN_message_t;


class CanInterface {  // Wrapper for interfacing PCAN and ODrive
  public:
    void begin();
    void setBaudRate(int CANBaudRate);
    void write(CAN_message_t msg);
    int read(CAN_message_t return_msg);
    void close();
  private:
    pcanfd_init fd_;
    pcanfd_msg convert_msg_to_pcan(CAN_message_t msg);
};
// }  // namespace can_interface