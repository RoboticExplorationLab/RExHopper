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
 *     Ja: Matrix32
 */
template <typename Scalar>
void Jac(const Eigen::Matrix<Scalar, 4, 1>& q, const Eigen::Matrix<Scalar, 4, 1>& qd,
         const Eigen::Matrix<Scalar, 3, 1>& g, const sym::constants_t& constants,
         Eigen::Matrix<Scalar, 3, 2>* const Ja = nullptr) {
  // Total ops: 117

  // Input arrays

  // Intermediate terms (41)
  const Scalar _tmp0 = std::pow(constants.L[3], Scalar(2));
  const Scalar _tmp1 = std::sin(q(0, 0));
  const Scalar _tmp2 = _tmp1 * constants.L[0];
  const Scalar _tmp3 = std::sin(q(2, 0));
  const Scalar _tmp4 = _tmp3 * constants.L[2];
  const Scalar _tmp5 = _tmp2 - _tmp4;
  const Scalar _tmp6 = std::cos(q(0, 0));
  const Scalar _tmp7 = _tmp6 * constants.L[0];
  const Scalar _tmp8 = constants.L[2] * std::cos(q(2, 0));
  const Scalar _tmp9 = _tmp7 - _tmp8;
  const Scalar _tmp10 = std::pow(_tmp5, Scalar(2)) + std::pow(_tmp9, Scalar(2));
  const Scalar _tmp11 = _tmp0 + _tmp10 - std::pow(constants.L[1], Scalar(2));
  const Scalar _tmp12 = (Scalar(1) / Scalar(4)) / _tmp10;
  const Scalar _tmp13 = std::pow(Scalar(1 - std::pow(_tmp11, Scalar(2)) * _tmp12 / _tmp0),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp14 = Scalar(1.0) / (constants.L[3]);
  const Scalar _tmp15 = _tmp5 * _tmp7;
  const Scalar _tmp16 = (Scalar(1) / Scalar(2)) / std::sqrt(_tmp10);
  const Scalar _tmp17 = _tmp16 * (2 * _tmp15 - 2 * _tmp2 * _tmp9);
  const Scalar _tmp18 = (Scalar(1) / Scalar(2)) / (_tmp10 * std::sqrt(_tmp10));
  const Scalar _tmp19 = _tmp18 * (_tmp1 * _tmp9 * constants.L[0] - _tmp15);
  const Scalar _tmp20 = _tmp11 * _tmp14;
  const Scalar _tmp21 = std::pow(constants.L[2], Scalar(2));
  const Scalar _tmp22 = std::pow(constants.L[0], Scalar(2));
  const Scalar _tmp23 =
      -std::pow(_tmp1, Scalar(2)) * _tmp22 + _tmp10 + _tmp21 - _tmp22 * std::pow(_tmp6, Scalar(2));
  const Scalar _tmp24 = std::pow(Scalar(-_tmp12 * std::pow(_tmp23, Scalar(2)) / _tmp21 + 1),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp25 = Scalar(1.0) / (constants.L[2]);
  const Scalar _tmp26 = _tmp23 * _tmp25;
  const Scalar _tmp27 =
      -_tmp13 * (_tmp14 * _tmp17 + _tmp19 * _tmp20) - _tmp24 * (_tmp17 * _tmp25 + _tmp19 * _tmp26);
  const Scalar _tmp28 = _tmp14 * _tmp16;
  const Scalar _tmp29 = _tmp16 * _tmp25;
  const Scalar _tmp30 = -q(2, 0) + std::acos(_tmp11 * _tmp28) + std::acos(_tmp23 * _tmp29);
  const Scalar _tmp31 = std::cos(_tmp30);
  const Scalar _tmp32 = _tmp27 * _tmp31;
  const Scalar _tmp33 = _tmp32 * constants.L[5];
  const Scalar _tmp34 = constants.L[3] + constants.L[4];
  const Scalar _tmp35 = -_tmp34 * std::sin(_tmp30);
  const Scalar _tmp36 = _tmp5 * _tmp8;
  const Scalar _tmp37 = 2 * _tmp3 * _tmp9 * constants.L[2] - 2 * _tmp36;
  const Scalar _tmp38 = _tmp18 * (_tmp36 - _tmp4 * _tmp9);
  const Scalar _tmp39 = -_tmp13 * (_tmp20 * _tmp38 + _tmp28 * _tmp37) -
                        _tmp24 * (_tmp26 * _tmp38 + _tmp29 * _tmp37) - 1;
  const Scalar _tmp40 = _tmp31 * _tmp39;

  // Output terms (1)
  if (Ja != nullptr) {
    Eigen::Matrix<Scalar, 3, 2>& _Ja = (*Ja);

    _Ja(0, 0) = -_tmp27 * _tmp35 + _tmp33;
    _Ja(1, 0) = 0;
    _Ja(2, 0) = _tmp32 * _tmp34 + _tmp33;
    _Ja(0, 1) = _tmp31 * _tmp39 * constants.L[5] - _tmp35 * _tmp39 - _tmp4;
    _Ja(1, 1) = 0;
    _Ja(2, 1) = _tmp34 * _tmp40 + _tmp40 * constants.L[5] + _tmp8;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
