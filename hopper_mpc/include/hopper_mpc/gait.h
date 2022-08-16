#include "hopper_mpc/leg.h"
#include "hopper_mpc/model.h"
#include "hopper_mpc/pid.h"
#include "hopper_mpc/rwa.h"

class Gait {  // The class

 public:                                                           // Access specifier
  Gait(Model model, double dt, Eigen::Matrix<double, 13, 1> X_f);  // constructor
  void Run();
  Eigen::Matrix<double, 5, 1> u;  // controls
  Eigen::Vector3d pe_ref;         // end effector ref position
  Eigen::Vector3d pf_ref;         // footstep ref position
  Eigen::Quaterniond Q_base;
  Eigen::Quaterniond Q_z;
  Eigen::Quaterniond Q_ref;
  double z_ref;

 private:
  Eigen::Matrix<double, 5, 1> uRaibert(std::string state, std::string state_prev, Eigen::Matrix<double, 13, 1> X_in,
                                       Eigen::Matrix<double, 13, 1> X_ref);
  Eigen::Matrix<double, 5, 1> uKinInvVert(std::string state, std::string state_prev, Eigen::Matrix<double, 13, 1> X_in,
                                          Eigen::Matrix<double, 13, 1> X_ref);
  Eigen::Matrix<double, 5, 1> uKinInvStand(std::string state, std::string state_prev, Eigen::Matrix<double, 13, 1> X_in,
                                           Eigen::Matrix<double, 13, 1> X_ref);
  Model model_;
  double dt_;  // timestep size
  std::unique_ptr<Leg> legPtr_;
  std::unique_ptr<Rwa> rwaPtr_;
  std::unique_ptr<PID3> pid_dpPtr_;
};