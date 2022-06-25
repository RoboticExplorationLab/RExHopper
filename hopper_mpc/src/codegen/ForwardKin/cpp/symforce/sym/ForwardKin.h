// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     cpp_templates/function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <lcmtypes/sym/constants_t.hpp>

namespace sym {

/**
 * This function was autogenerated. Do not modify by hand.
 *
 * Args:
 *     q: Matrix41
 *     qd: Matrix41
 *     g: Matrix31
 *     constants: Values
 *
 * Outputs:
 *     fwd: Matrix31
 */
template <typename Scalar>
void Forwardkin(const Eigen::Matrix<Scalar, 4, 1>& q, const Eigen::Matrix<Scalar, 4, 1>& qd,
                const Eigen::Matrix<Scalar, 3, 1>& g, const sym::constants_t& constants,
                Eigen::Matrix<Scalar, 3, 1>* const fwd = nullptr) {
  // Total ops: 47

  // Input arrays

  // Intermediate terms (11)
  const Scalar _tmp0 = constants.L[2] * std::cos(q(2, 0));
  const Scalar _tmp1 = std::sin(q(0, 0));
  const Scalar _tmp2 = constants.L[2] * std::sin(q(2, 0));
  const Scalar _tmp3 = std::cos(q(0, 0));
  const Scalar _tmp4 = std::pow(Scalar(-_tmp0 + _tmp3 * constants.L[0]), Scalar(2)) +
                       std::pow(Scalar(_tmp1 * constants.L[0] - _tmp2), Scalar(2));
  const Scalar _tmp5 = (Scalar(1) / Scalar(2)) / std::sqrt(_tmp4);
  const Scalar _tmp6 = std::pow(constants.L[0], Scalar(2));
  const Scalar _tmp7 =
      -q(2, 0) +
      std::acos(_tmp5 *
                (-std::pow(_tmp1, Scalar(2)) * _tmp6 - std::pow(_tmp3, Scalar(2)) * _tmp6 + _tmp4 +
                 std::pow(constants.L[2], Scalar(2))) /
                constants.L[2]) +
      std::acos(
          _tmp5 *
          (_tmp4 - std::pow(constants.L[1], Scalar(2)) + std::pow(constants.L[3], Scalar(2))) /
          constants.L[3]);
  const Scalar _tmp8 = std::sin(_tmp7);
  const Scalar _tmp9 = _tmp8 * constants.L[5];
  const Scalar _tmp10 = constants.L[3] + constants.L[4];

  // Output terms (1)
  if (fwd != nullptr) {
    Eigen::Matrix<Scalar, 3, 1>& _fwd = (*fwd);

    _fwd(0, 0) = _tmp0 - _tmp10 * std::cos(_tmp7) + _tmp9;
    _fwd(1, 0) = 0;
    _fwd(2, 0) = _tmp10 * _tmp8 + _tmp2 + _tmp9;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym