#include "hopper_mpc/utils.hpp"
#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "rot_conv/rot_conv.h"

Utils::Utils(){};

Eigen::Vector3d Utils::QuatToEuler(const Eigen::Quaterniond Q) {
  // Euler ZYX Intrinsic, as specified by
  // https://dspace.mit.edu/bitstream/handle/1721.1/138000/convex_mpc_2fix.pdf?sequence=2&isAllowed=y
  // See also: https://github.com/AIS-Bonn/rot_conv_lib
  rot_conv::EulerAngles euler;
  rot_conv::EulerFromQuat(Q, euler);
  Eigen::Vector3d euler_eig;
  euler_eig << euler.roll, euler.pitch, euler.yaw;
  return euler_eig;
}

Eigen::Quaterniond Utils::EulerToQuat(const double roll, const double pitch, const double yaw) {
  // Euler ZYX Intrinsic
  Eigen::Quaterniond Q = rot_conv::QuatFromEuler(yaw, pitch, roll);
  return Q;
}

Eigen::Matrix3d Utils::Skew(Eigen::Vector3d vec) {
  Eigen::Matrix3d rst;
  rst.setZero();
  rst << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return rst;
};

Eigen::Matrix3d Utils::PseudoInverse(const Eigen::Matrix3d& mat) {
  // https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
  double epsilon = std::numeric_limits<double>::epsilon();
  // For a non-square matrix
  // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
  return svd.matrixV() *
         (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
         svd.matrixU().adjoint();
};

double Utils::WrapToPi(double a) {
  // wrap input angle bt 0 and pi
  return fmod(a + M_PI, 2 * M_PI) - M_PI;
};

double Utils::Clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

Eigen::Quaterniond Utils::VecToQuat(Eigen::Vector3d v1, Eigen::Vector3d v2) {
  // Conversion of line vector to quaternion rotation b/t it and a datum vector v1
  Eigen::Vector3d u1;
  u1 = v1.normalized();
  Eigen::Quaterniond Q;
  if ((v2.squaredNorm() == 0.0) || (isnan(v2.array()).any())) {
    Q.coeffs() << 0, 0, 0, 1;
  } else {
    Eigen::Vector3d u2;
    u2 = v2.normalized();
    if (u1.isApprox(-u2)) {
      Q.coeffs().block<3, 1>(0, 0).array() = (v1.cross(v2)).norm();
    } else {
      Eigen::Vector3d u_half;
      u_half = (u1 + u2).normalized();
      Q.w() = u1.transpose() * u_half;
      Q.coeffs().block<3, 1>(0, 0) = u1.cross(u_half);
      Q.normalize();
    }
  }
  return Q;
}

double Utils::AngleBetween(Eigen::Quaterniond Q1, Eigen::Quaterniond Q2) {
  // unsigned angle between two quaternions
  Eigen::Quaterniond Qd;
  Qd = Q1.conjugate() * Q2;
  return WrapToPi(2 * atan2(Qd.vec().norm(), Qd.w()));
}

Eigen::Quaterniond Utils::GenYawQuat(const double z_angle) {
  // create quaternion with only a yaw axis rotation (no pitch/roll, constrained to z-axis)
  // double z_angle = 2 * asin(Q.z());  // z-axis of body quaternion (only works assuming no pitch/roll)
  Eigen::Quaterniond Q_z;
  Q_z.w() = cos(z_angle / 2);
  Q_z.x() = 0;
  Q_z.y() = 0;
  Q_z.z() = sin(z_angle / 2);
  return Q_z;
}

Eigen::Quaterniond Utils::ExtractYawQuat(Eigen::Quaterniond Q) {
  // extract the yaw angle rotation of a quaternion
  Eigen::Vector3d v1;
  v1 << 1, 0, 0;                         // vector pointing straight forward
  Eigen::Vector3d v2 = Q.matrix() * v1;  // get directional vector
  // v2(2) = 0;                             // remove z-axis of the vector
  // Eigen::Quaterniond Qd = VecToQuat(v1, v2);
  double angle = atan2(v2(1), v2(0)) - atan2(v1(1), v1(0));  // signed angle in xy plane, cc positive
  // double angle = 2 * asin(Q.z());
  Eigen::Quaterniond Qd = GenYawQuat(angle);
  return Qd;
}

double Utils::ExtractX(Eigen::Quaterniond Q) {
  // extract the x (roll) angle rotation from a quaternion
  Eigen::Vector3d v1;
  v1 << 0, 0, 1;                                             // vector pointing straight up
  Eigen::Vector3d v2 = Q.matrix() * v1;                      // get directional vector
  double angle = atan2(v2(2), v2(1)) - atan2(v1(2), v1(1));  // signed angle in yz plane, cc positive
  return angle;
}

double Utils::ExtractZ(Eigen::Quaterniond Q) {
  // extract the z (yaw) angle rotation from a quaternion
  Eigen::Vector3d v1;
  v1 << 1, 0, 0;                                             // vector pointing straight forward
  Eigen::Vector3d v2 = Q.matrix() * v1;                      // get directional vector
  double angle = atan2(v2(1), v2(0)) - atan2(v1(1), v1(0));  // signed angle in xy plane, cc positive
  return angle;
}

double Utils::PolyFit(const std::vector<double>& t, const std::vector<double>& v, int k, double t_new) {
  // taken from https://towardsdatascience.com/least-square-polynomial-fitting-using-c-eigen-package-c0673728bd01
  // k = order of polynomial, for example k = 3 for cubic polynomial

  assert(t.size() == v.size());  // check to make sure inputs are correct
  size_t N = t.size();           // number of elements

  Eigen::MatrixXd T(N, k + 1);
  Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), N);  // initialize eigen vector from std::vector
  Eigen::VectorXd result;

  assert(N >= k + 1);
  // Populate the matrix
  for (size_t i = 0; i < N; ++i) {
    for (size_t j = 0; j < k + 1; ++j) {
      T(i, j) = pow(t.at(i), j);
    }
  }

  // Solve for linear least square fit
  result = T.householderQr().solve(V);
  std::vector<double> coeff;
  coeff.resize(k + 1);
  for (int k = 0; k < k + 1; k++) {
    coeff[k] = result[k];
  }

  // interpolate to get value at t_new
  double v_out = coeff[0] + coeff[1] * t_new + coeff[2] * (pow(t_new, 2)) + coeff[3] * (pow(t_new, 3));
  return v_out;
}
