#include "hopper_mpc/cx5.h"

Cx5::Cx5() {
  // create the connection object with port and baud rate
  connection("COM3", 115200);

  // create the InertialNode, passing in the connection
  node(connection);

  // ping the Node
  // bool success = node.ping();  // test of whether it works
  try {
    if (!node.ping()) {
      throw 10;
    }
  } catch (char* excp) {
    cout << "Ping didn't work " << excp;
  }
  // "it is still useful to set the Node to an idle state between sampling sessions and when changing configurations."
  // put the Inertial Node into its idle state
  // node.setToIdle();

  // get all of the active channels for the GPS category on the Node
  // mscl::MipChannels activeChs = node.getActiveChannelFields(mscl::MipTypes::CLASS_GNSS);

  // configure a node
  mscl::MipChannels estFilterChs;
  // max hertz = 500
  estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION, mscl::SampleRate::Hertz(500)));

  // set the active channels for the different data classes on the Node
  node.setActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER, estFilterChs);

  // sampling
  // start sampling the active channels on the filter class of the Node
  node.enableDataStream(mscl::MipTypes::CLASS_ESTFILTER);
}

void Cx5::Collect() {
  // collecting data
  // get all the packets that have been collected, with a timeout of 500 milliseconds
  mscl::MipDataPackets packets = node.getDataPackets(500);  // does this require multi-threading?

  for (mscl::MipDataPacket packet : packets) {
    packet.descriptorSet();  // the descriptor set of the packet
    packet.timestamp();      // the PC time when this packet was received

    // get all of the points in the packet
    mscl::MipDataPoints points = packet.data();

    for (mscl::MipDataPoint dataPoint : points) {
      dataPoint.channelName();  // the name of the channel for this point
      dataPoint.storedAs();     // the ValueType that the data is stored as
      dataPoint.as_float();     // get the value as a float
    }
  }
}