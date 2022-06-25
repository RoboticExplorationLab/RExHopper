//
// Created by shuoy on 10/19/21.
//

#include "hopper_mpc/utils.h"

Eigen::Vector3d Utils::quat_to_euler(Eigen::Quaterniond quat) {
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
}

template <typename T>
Eigen::Matrix<T, 3, 3> Utils::hat(Eigen::Matrix<T, 3, 1> vec) {
  Eigen::Matrix<T, 3, 3> rst;
  rst.setZero();
  rst << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return rst;
}
template Eigen::Matrix<Real<1, double>, 3, 3> Utils::hat(Eigen::Matrix<Real<1, double>, 3, 1> vec);

template <typename T>
Eigen::Matrix<T, 4, 4> Utils::L(Eigen::Quaternion<T> q) {
  Eigen::Matrix<T, 4, 4> rst;
  rst.setZero();
  Eigen::Matrix<T, 3, 1> vec = q.vec();
  rst.template block<1, 4>(0, 0) << q.w(), -q.vec().transpose();
  rst.template block<3, 1>(1, 0) << q.vec();
  rst.template block<3, 3>(1, 1) << q.w() * Eigen::Matrix<T, 3, 3>::Identity(3, 3) - hat(vec);
  return rst;
}
template Eigen::Matrix<Real<1, double>, 4, 4> Utils::L(Eigen::Quaternion<Real<1, double>> q);
template Eigen::Matrix<double, 4, 4> Utils::L(Eigen::Quaternion<double> q);

Eigen::Matrix<double, 4, 3> Utils::H() {
  Eigen::Matrix<double, 4, 3> mat;
  mat << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
  return mat;
}

Eigen::Quaterniond Utils::rho(Eigen::Vector3d phi) {
  double scalar = 1 / sqrt(1 + phi.norm());
  phi = scalar * phi;
  Eigen::Quaterniond q(scalar, phi(0), phi(1), phi(2));
  return q;
}

Eigen::Vector3d BezierUtils::get_foot_pos_curve(float t, Eigen::Vector3d foot_pos_start, Eigen::Vector3d foot_pos_final) {
  Eigen::Vector3d foot_pos_target;
  // X-axis
  std::vector<double> bezierX{foot_pos_start(0), foot_pos_start(0), foot_pos_final(0), foot_pos_final(0), foot_pos_final(0)};
  foot_pos_target(0) = bezier_curve(t, bezierX);

  // Y-axis
  std::vector<double> bezierY{foot_pos_start(1), foot_pos_start(1), foot_pos_final(1), foot_pos_final(1), foot_pos_final(1)};
  foot_pos_target(1) = bezier_curve(t, bezierY);

  // Z-axis
  std::vector<double> bezierZ{foot_pos_start(2), foot_pos_start(2), foot_pos_final(2), foot_pos_final(2), foot_pos_final(2)};
  bezierZ[1] += FOOT_SWING_CLEARANCE1;
  bezierZ[2] += FOOT_SWING_CLEARANCE2;
  foot_pos_target(2) = bezier_curve(t, bezierZ);

  return foot_pos_target;
}

double BezierUtils::bezier_curve(double t, const std::vector<double>& P) {
  std::vector<double> coefficients{1, 4, 6, 4, 1};
  double y = 0;
  for (int i = 0; i <= bezier_degree; i++) {
    y += coefficients[i] * std::pow(t, i) * std::pow(1 - t, bezier_degree - i) * P[i];
  }
  return y;
}

bool CubicSpineUtils::set_foot_pos_curve(Eigen::Vector3d foot_pos_start, Eigen::Vector3d foot_pos_final) {
  if (curve_constructed == true) {
    // a cubic spine curve is already constructed
    throw ERROR_CURVE_ALREADY_SET;
  } else {
    curve_constructed = true;
    std::vector<double> t;
    t.resize(num_pts);  // fix to be num_pts points
    std::vector<double> val;
    val.resize(num_pts);  // fix to be num_pts points

    // construct X curve
    for (int i = 0; i < num_pts; i++) {
      t[i] = i / double(num_pts - 1);
      val[i] = foot_pos_start(0) + t[i] * (foot_pos_final(0) - foot_pos_start(0));
    }
    //        for (double i: t)
    //            std::cout << i << ' ';
    //        std::cout << std::endl;

    sp_x = tk::spline(t, val, tk::spline::cspline, false, tk::spline::first_deriv, 0.0, tk::spline::first_deriv, 0.0);

    // construct Y curve
    for (int i = 0; i < num_pts; i++) {
      //            t[i] = i/double(num_pts-1);
      val[i] = foot_pos_start(1) + t[i] * (foot_pos_final(1) - foot_pos_start(1));
    }
    sp_y = tk::spline(t, val, tk::spline::cspline, false, tk::spline::first_deriv, 0.0, tk::spline::first_deriv, 0.0);

    // construct z curve
    for (int i = 0; i < num_pts; i++) {
      //            t[i] = i/double(num_pts-1);
      val[i] = foot_pos_start(2) + t[i] * (foot_pos_final(2) - foot_pos_start(2));
    }
    val[1] += FOOT_SWING_CLEARANCE2;
    sp_z = tk::spline(t, val, tk::spline::cspline, false, tk::spline::first_deriv, 0.0, tk::spline::first_deriv, 0.0);
  }
  return true;
}

Eigen::Vector3d CubicSpineUtils::get_foot_pos_curve(double t) {
  if (curve_constructed == true) {
    return Eigen::Vector3d(sp_x(t), sp_y(t), sp_z(t));
  } else {
    // must be something wrong, should avoid this from happening
    throw ERROR_CURVE_NOT_SET;
  }
}
Eigen::Vector3d CubicSpineUtils::get_foot_vel_curve(double t) {
  if (curve_constructed == true) {
    return Eigen::Vector3d(sp_x.deriv(1, t), sp_y.deriv(1, t), sp_z.deriv(1, t));
  } else {
    // must be something wrong, should avoid this from happening
    throw ERROR_CURVE_NOT_SET;
  }
}
Eigen::Vector3d CubicSpineUtils::get_foot_acc_curve(double t) {
  if (curve_constructed == true) {
    return Eigen::Vector3d(sp_x.deriv(2, t), sp_y.deriv(2, t), sp_z.deriv(2, t));
  } else {
    // must be something wrong, should avoid this from happening
    throw ERROR_CURVE_NOT_SET;
  }
}