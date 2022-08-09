#include "hopper_mpc/leg.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/pid.h"

class Gait {  // The class

 public:                                                           // Access specifier
  Gait(Model model, double dt, Eigen::Matrix<double, 13, 1> X_f);  // constructor

  void Run();
  Model model;

 private:
  void U_Raibert(std::string state, std::string state_prev, Eigen::Matrix<double, 13, 1> X_in, Eigen::Matrix<double, 12, 1> x_ref);
  double dt_;  // timestep size
  double z_ref_;
  Eigen::Matrix<double, 13, 1> X_f_;  // final state
  Eigen::Matrix<double, 5, 1> u_;     // controls

  std::unique_ptr<Leg> legPtr_;
};