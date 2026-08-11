/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <cstdint>
#include <limits>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>

// CMake defines this macro for all GyPhysics targets. Keeping a default makes
// this header usable in a small standalone experiment as well.
#ifndef GY_PHYSICS_REAL_IS_DOUBLE
#define GY_PHYSICS_REAL_IS_DOUBLE 1
#endif

namespace gy::physics::math {

#if GY_PHYSICS_REAL_IS_DOUBLE == 1
using Real = double;
#elif GY_PHYSICS_REAL_IS_DOUBLE == 0
using Real = float;
#else
#error "GY_PHYSICS_REAL_IS_DOUBLE must be 0 or 1"
#endif

using Index = std::uint32_t;

using Vector2 = Eigen::Matrix<Real, 2, 1>;
using Vector3 = Eigen::Matrix<Real, 3, 1>;
using Vector4 = Eigen::Matrix<Real, 4, 1>;

using Matrix2 = Eigen::Matrix<Real, 2, 2>;
using Matrix3 = Eigen::Matrix<Real, 3, 3>;
using Matrix4 = Eigen::Matrix<Real, 4, 4>;

using Quaternion = Eigen::Quaternion<Real>;

inline constexpr Real kZero = static_cast<Real>(0);
inline constexpr Real kOne = static_cast<Real>(1);
inline constexpr Real kEpsilon = std::numeric_limits<Real>::epsilon();

static_assert(
    std::is_same_v<Real, float> || std::is_same_v<Real, double>,
    "GyPhysics Real must be float or double"
);

[[nodiscard]] inline constexpr const char* realTypeName() noexcept
{
    if constexpr (std::is_same_v<Real, double>) {
        return "double";
    }
    return "float";
}

} // namespace gy::physics::math
