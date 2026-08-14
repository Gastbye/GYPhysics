/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <cmath>
#include <stdexcept>

#include "gy/physics/math/MathTypes.h"

namespace gy::physics::math {

[[nodiscard]] inline Quaternion quaternionFromAxisAngle(
    const Real& angleRadians,
    const Vector3& axis)
{
    const Real axisNormSquared = axis.squaredNorm();
    if (axisNormSquared <= kEpsilon * kEpsilon) 
    {
        throw std::invalid_argument(
            "Rotation axis must have a non-zero length."
        );
    }

    const Vector3 unitAxis = axis / std::sqrt(axisNormSquared);
    const Real halfAngle = angleRadians / Real{2};
    const Real cosine = std::cos(halfAngle);
    const Real sine = std::sin(halfAngle);

    return Quaternion(
        cosine, 
        unitAxis.x() * sine,
        unitAxis.y() * sine, 
        unitAxis.z() * sine
    );
}

[[nodiscard]] inline Quaternion multiplyQuaternions(
    const Quaternion& q1,
    const Quaternion& q2) noexcept
{
    return q1 * q2;
}

[[nodiscard]] inline Real quaternionNormSquared(
    const Quaternion& quaternion) noexcept
{
    return quaternion.squaredNorm();
}

[[nodiscard]] inline Real quaternionNorm(
    const Quaternion& quaternion) noexcept
{
    return quaternion.norm();
}

[[nodiscard]] inline Quaternion normalizedQuaternion(
    const Quaternion& quaternion)
{
    if (quaternionNormSquared(quaternion) <= kEpsilon * kEpsilon)
    {
        throw std::invalid_argument(
            "Cannot normalize a near-zero quaternion.");
    }

    return quaternion.normalized();
}

[[nodiscard]] inline Quaternion conjugateQuaternion(
    const Quaternion& quaternion) noexcept
{
    return quaternion.conjugate();
}

[[nodiscard]] inline Quaternion inverseQuaternion(
    const Quaternion& quaternion)
{
    if (quaternionNormSquared(quaternion) <= kEpsilon * kEpsilon) 
    {
        throw std::invalid_argument(
            "Cannot invert a near-zero quaternion."
        );
    }

    return quaternion.inverse();
}

[[nodiscard]] inline Matrix3 quaternionToRotationMatrix(
    const Quaternion& quaternion)
{
    return normalizedQuaternion(quaternion).toRotationMatrix();
}

} // namespace gy::physics::math