#pragma once
#include "mscl/mscl.h"
// https://lord-microstrain.github.io/MSCL/Documentation/Getting%20Started/index.html#communicating-with-a-node-210

class Cx5 {  // The class

 public:  // Access specifier
  Cx5();  // constructor
  void Collect();

 private:
  mscl::Connection::Serial connection;
  mscl::InertialNode node;
};
