//
// Created by shuoy on 10/19/21.
// Modified by bbokser starting on 06/26/22.
//
#include "hopper_mpc/utils.hpp"
#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"

Utils::Utils(){};

Eigen::Vector3d Utils::QuatToEuler(Eigen::Quaterniond quat) {
  Eigen::Vector3d rst;

  // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
  Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
  double x = coeff(0);
  double y = coeff(1);
  double z = coeff(2);
  double w = coeff(3);

  double y_sqr = y * y;

  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + y_sqr);

  rst[0] = atan2(t0, t1);

  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > +1.0 ? +1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  rst[1] = asin(t2);

  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (y_sqr + z * z);
  rst[2] = atan2(t3, t4);
  return rst;
};

Eigen::Quaterniond Utils::EulerToQuat(const double roll, const double pitch, const double yaw) {
  // taken from https://stackoverflow.com/questions/21412169/creating-a-rotation-matrix-with-pitch-yaw-roll-using-eigen
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q;
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

Eigen::Quaterniond Utils::VecToQuat(Eigen::Vector3d v2) {
  // Conversion of line vector to quaternion rotation b/t it and a datum vector v1
  //   v1 = np.array([0, 0, -1])  # datum vector, chosen as aligned with z-axis (representing leg direction)
  Eigen::Vector3d v1;
  v1 << 0, 0, -1;  // datum vector, chosen as aligned with z-axis (representing leg direction)
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
  Eigen::Quaterniond Qd;
  Qd = Q1.inverse() * Q2;

  return 2 * atan2(Qd.vec().norm(), Qd.w());
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