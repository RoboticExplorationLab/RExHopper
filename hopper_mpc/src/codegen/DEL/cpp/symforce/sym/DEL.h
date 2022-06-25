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
 *     M: Matrix44
 *     C: Matrix41
 *     G: Matrix41
 */
template <typename Scalar>
void Del(const Eigen::Matrix<Scalar, 4, 1>& q, const Eigen::Matrix<Scalar, 4, 1>& qd,
         const Eigen::Matrix<Scalar, 3, 1>& g, const sym::constants_t& constants,
         Eigen::Matrix<Scalar, 4, 4>* const M = nullptr,
         Eigen::Matrix<Scalar, 4, 1>* const C = nullptr,
         Eigen::Matrix<Scalar, 4, 1>* const G = nullptr) {
  // Total ops: 272

  // Input arrays

  // Intermediate terms (84)
  const Scalar _tmp0 = std::pow(constants.l_c0[2], Scalar(2));
  const Scalar _tmp1 = Scalar(1.0) * constants.m[0];
  const Scalar _tmp2 = _tmp0 * _tmp1;
  const Scalar _tmp3 = std::cos(q(0, 0));
  const Scalar _tmp4 = std::pow(constants.L[0], Scalar(2)) * constants.m[1];
  const Scalar _tmp5 = std::sin(q(0, 0));
  const Scalar _tmp6 = std::pow(_tmp5, Scalar(2));
  const Scalar _tmp7 = std::pow(constants.l_c0[0], Scalar(2));
  const Scalar _tmp8 = q(0, 0) + q(1, 0);
  const Scalar _tmp9 = std::sin(_tmp8);
  const Scalar _tmp10 = _tmp9 * constants.l_c1[0];
  const Scalar _tmp11 = constants.L[0] * constants.m[1];
  const Scalar _tmp12 = _tmp11 * _tmp5;
  const Scalar _tmp13 = _tmp10 * _tmp12;
  const Scalar _tmp14 = std::cos(_tmp8);
  const Scalar _tmp15 = _tmp14 * constants.l_c1[2];
  const Scalar _tmp16 = _tmp11 * _tmp3;
  const Scalar _tmp17 = _tmp15 * _tmp16;
  const Scalar _tmp18 = std::pow(constants.l_c1[0], Scalar(2)) * constants.m[1];
  const Scalar _tmp19 = std::pow(constants.l_c1[2], Scalar(2)) * constants.m[1];
  const Scalar _tmp20 = std::pow(_tmp14, Scalar(2)) * _tmp19 + _tmp18 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp21 = _tmp13 + _tmp17 + _tmp20;
  const Scalar _tmp22 = std::pow(constants.l_c2[2], Scalar(2));
  const Scalar _tmp23 = Scalar(1.0) * constants.m[2];
  const Scalar _tmp24 = _tmp22 * _tmp23;
  const Scalar _tmp25 = std::cos(q(2, 0));
  const Scalar _tmp26 = std::pow(constants.L[2], Scalar(2)) * constants.m[3];
  const Scalar _tmp27 = std::sin(q(2, 0));
  const Scalar _tmp28 = std::pow(_tmp27, Scalar(2));
  const Scalar _tmp29 = std::pow(constants.l_c2[0], Scalar(2));
  const Scalar _tmp30 = q(2, 0) + q(3, 0);
  const Scalar _tmp31 = std::sin(_tmp30);
  const Scalar _tmp32 = _tmp31 * constants.l_c3[0];
  const Scalar _tmp33 = constants.L[2] * constants.m[3];
  const Scalar _tmp34 = _tmp27 * _tmp33;
  const Scalar _tmp35 = _tmp32 * _tmp34;
  const Scalar _tmp36 = std::cos(_tmp30);
  const Scalar _tmp37 = _tmp36 * constants.l_c3[2];
  const Scalar _tmp38 = _tmp25 * _tmp33;
  const Scalar _tmp39 = _tmp37 * _tmp38;
  const Scalar _tmp40 = std::pow(constants.l_c3[0], Scalar(2)) * constants.m[3];
  const Scalar _tmp41 = std::pow(constants.l_c3[2], Scalar(2)) * constants.m[3];
  const Scalar _tmp42 = std::pow(_tmp31, Scalar(2)) * _tmp40 + std::pow(_tmp36, Scalar(2)) * _tmp41;
  const Scalar _tmp43 = _tmp35 + _tmp39 + _tmp42;
  const Scalar _tmp44 = std::pow(qd(0, 0), Scalar(2));
  const Scalar _tmp45 = Scalar(0.5) * _tmp44 * constants.m[0] * std::sin(2 * q(0, 0));
  const Scalar _tmp46 = _tmp12 * _tmp14 * constants.l_c1[0];
  const Scalar _tmp47 = qd(0, 0) * qd(1, 0);
  const Scalar _tmp48 = Scalar(2.0) * _tmp47;
  const Scalar _tmp49 = _tmp16 * _tmp9 * constants.l_c1[2];
  const Scalar _tmp50 = std::pow(qd(1, 0), Scalar(2));
  const Scalar _tmp51 = Scalar(1.0) * _tmp44;
  const Scalar _tmp52 = _tmp10 * _tmp16 * _tmp44;
  const Scalar _tmp53 = _tmp12 * _tmp15 * _tmp44;
  const Scalar _tmp54 = _tmp14 * _tmp9;
  const Scalar _tmp55 = _tmp18 * _tmp54;
  const Scalar _tmp56 = _tmp19 * _tmp54;
  const Scalar _tmp57 = _tmp44 * _tmp55;
  const Scalar _tmp58 = _tmp50 * _tmp55;
  const Scalar _tmp59 = _tmp44 * _tmp56;
  const Scalar _tmp60 = _tmp50 * _tmp56;
  const Scalar _tmp61 = 2 * _tmp47;
  const Scalar _tmp62 = std::pow(qd(2, 0), Scalar(2));
  const Scalar _tmp63 = Scalar(0.5) * _tmp62 * constants.m[2] * std::sin(2 * q(2, 0));
  const Scalar _tmp64 = _tmp34 * _tmp36 * constants.l_c3[0];
  const Scalar _tmp65 = qd(2, 0) * qd(3, 0);
  const Scalar _tmp66 = Scalar(2.0) * _tmp65;
  const Scalar _tmp67 = _tmp31 * _tmp38 * constants.l_c3[2];
  const Scalar _tmp68 = std::pow(qd(3, 0), Scalar(2));
  const Scalar _tmp69 = Scalar(1.0) * _tmp62;
  const Scalar _tmp70 = _tmp32 * _tmp38 * _tmp62;
  const Scalar _tmp71 = _tmp34 * _tmp37 * _tmp62;
  const Scalar _tmp72 = _tmp31 * _tmp36;
  const Scalar _tmp73 = _tmp40 * _tmp72;
  const Scalar _tmp74 = _tmp41 * _tmp72;
  const Scalar _tmp75 = _tmp62 * _tmp73;
  const Scalar _tmp76 = _tmp68 * _tmp73;
  const Scalar _tmp77 = _tmp62 * _tmp74;
  const Scalar _tmp78 = _tmp68 * _tmp74;
  const Scalar _tmp79 = 2 * _tmp65;
  const Scalar _tmp80 = _tmp5 * g(0, 0);
  const Scalar _tmp81 =
      _tmp10 * constants.m[1] * g(0, 0) - _tmp14 * constants.l_c1[2] * constants.m[1] * g(2, 0);
  const Scalar _tmp82 = _tmp27 * g(0, 0);
  const Scalar _tmp83 =
      _tmp32 * constants.m[3] * g(0, 0) - _tmp36 * constants.l_c3[2] * constants.m[3] * g(2, 0);

  // Output terms (3)
  if (M != nullptr) {
    Eigen::Matrix<Scalar, 4, 4>& _M = (*M);

    _M(0, 0) = _tmp1 * _tmp6 * _tmp7 + 2 * _tmp13 + 2 * _tmp17 - _tmp2 * _tmp6 + _tmp2 + _tmp20 +
               std::pow(_tmp3, Scalar(2)) * _tmp4 + _tmp4 * _tmp6 + Scalar(1.0) * constants.I[0];
    _M(1, 0) = _tmp21;
    _M(2, 0) = 0;
    _M(3, 0) = 0;
    _M(0, 1) = _tmp21;
    _M(1, 1) = _tmp20 + Scalar(1.0) * constants.I[1];
    _M(2, 1) = 0;
    _M(3, 1) = 0;
    _M(0, 2) = 0;
    _M(1, 2) = 0;
    _M(2, 2) = _tmp23 * _tmp28 * _tmp29 - _tmp24 * _tmp28 + _tmp24 +
               std::pow(_tmp25, Scalar(2)) * _tmp26 + _tmp26 * _tmp28 + 2 * _tmp35 + 2 * _tmp39 +
               _tmp42 + Scalar(1.0) * constants.I[2];
    _M(3, 2) = _tmp43;
    _M(0, 3) = 0;
    _M(1, 3) = 0;
    _M(2, 3) = _tmp43;
    _M(3, 3) = _tmp42 + Scalar(1.0) * constants.I[3];
  }

  if (C != nullptr) {
    Eigen::Matrix<Scalar, 4, 1>& _C = (*C);

    _C(0, 0) = -_tmp0 * _tmp45 + _tmp45 * _tmp7 + _tmp46 * _tmp48 + _tmp46 * _tmp50 +
               _tmp46 * _tmp51 - _tmp48 * _tmp49 + _tmp48 * _tmp55 - _tmp48 * _tmp56 -
               _tmp49 * _tmp50 - _tmp49 * _tmp51 + Scalar(1.0) * _tmp52 - Scalar(1.0) * _tmp53 +
               Scalar(1.0) * _tmp57 + Scalar(1.0) * _tmp58 - Scalar(1.0) * _tmp59 -
               Scalar(1.0) * _tmp60;
    _C(1, 0) =
        _tmp52 - _tmp53 + _tmp55 * _tmp61 - _tmp56 * _tmp61 + _tmp57 + _tmp58 - _tmp59 - _tmp60;
    _C(2, 0) = -_tmp22 * _tmp63 + _tmp29 * _tmp63 + _tmp64 * _tmp66 + _tmp64 * _tmp68 +
               _tmp64 * _tmp69 - _tmp66 * _tmp67 + _tmp66 * _tmp73 - _tmp66 * _tmp74 -
               _tmp67 * _tmp68 - _tmp67 * _tmp69 + Scalar(1.0) * _tmp70 - Scalar(1.0) * _tmp71 +
               Scalar(1.0) * _tmp75 + Scalar(1.0) * _tmp76 - Scalar(1.0) * _tmp77 -
               Scalar(1.0) * _tmp78;
    _C(3, 0) =
        _tmp70 - _tmp71 + _tmp73 * _tmp79 - _tmp74 * _tmp79 + _tmp75 + _tmp76 - _tmp77 - _tmp78;
  }

  if (G != nullptr) {
    Eigen::Matrix<Scalar, 4, 1>& _G = (*G);

    _G(0, 0) = -_tmp11 * _tmp80 + _tmp3 * constants.L[0] * constants.m[1] * g(2, 0) +
               _tmp3 * constants.l_c0[2] * constants.m[0] * g(2, 0) -
               _tmp80 * constants.l_c0[0] * constants.m[0] - _tmp81;
    _G(1, 0) = -_tmp81;
    _G(2, 0) = _tmp25 * constants.L[2] * constants.m[3] * g(2, 0) +
               _tmp25 * constants.l_c2[2] * constants.m[2] * g(2, 0) - _tmp33 * _tmp82 -
               _tmp82 * constants.l_c2[0] * constants.m[2] - _tmp83;
    _G(3, 0) = -_tmp83;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
