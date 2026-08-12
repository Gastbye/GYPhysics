/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <cmath>

#include "gy/physics/math/MathTypes.h"

namespace gy::physics::math {

[[nodiscard]] inline Real dot(
    const Vector3& lhs,
    const Vector3& rhs) noexcept
{
    return lhs.dot(rhs);
}

[[nodiscard]] inline Vector3 cross(
    const Vector3& lhs,
    const Vector3& rhs) noexcept
{
    return lhs.cross(rhs);
}

[[nodiscard]] inline Real lengthSquared(
    const Vector3& vector) noexcept
{
    return vector.squaredNorm();
}

[[nodiscard]] inline Real length(
    const Vector3& vector) noexcept
{
    return vector.norm();
}

[[nodiscard]] inline Vector3 normalized(
    const Vector3& vector)
{
    const Real vectorLength = vector.norm();

    if (vectorLength <= kEpsilon) {
        throw std::invalid_argument(
            "Cannot normalize a zero-length vector"
        );
    }

    return vector / vectorLength;
}

[[nodiscard]] inline Vector3 multiply(
    const Matrix3& matrix,
    const Vector3& vector) noexcept
{
    return matrix * vector;
}

[[nodiscard]] inline Matrix3 multiply(
    const Matrix3& lhs,
    const Matrix3& rhs) noexcept
{
    return lhs * rhs;
}

[[nodiscard]] inline Matrix3 transpose(
    const Matrix3& matrix) noexcept
{
    return matrix.transpose();
}

// Rotation Matrix R, axis(n) and angleRadians(theta)：
//
//     R = cos(theta) I
//       + (1 - cos(theta)) n n^T
//       + sin(theta) [n]_x
//
// unit of angleRadians is radians.
[[nodiscard]] inline Matrix3 rotationMatrixFromAxisAngle(
    const Vector3& axis,
    Real angleRadians)
{
    const Vector3 unitAxis = normalized(axis);

    const Real x = unitAxis.x();
    const Real y = unitAxis.y();
    const Real z = unitAxis.z();

    const Real cosine = std::cos(angleRadians);
    const Real sine = std::sin(angleRadians);
    const Real oneMinusCosine = kOne - cosine;

    Matrix3 rotation;

    rotation(0, 0) = cosine + x * x * oneMinusCosine;
    rotation(0, 1) = x * y * oneMinusCosine - z * sine;
    rotation(0, 2) = x * z * oneMinusCosine + y * sine;

    rotation(1, 0) = y * x * oneMinusCosine + z * sine;
    rotation(1, 1) = cosine + y * y * oneMinusCosine;
    rotation(1, 2) = y * z * oneMinusCosine - x * sine;

    rotation(2, 0) = z * x * oneMinusCosine - y * sine;
    rotation(2, 1) = z * y * oneMinusCosine + x * sine;
    rotation(2, 2) = cosine + z * z * oneMinusCosine;

    return rotation;
}

// local vector is transferred into world coordinate system.
// Vector has no position, therefore no translation is performed.
[[nodiscard]] inline Vector3 localToWorldVector(
    const Vector3& localVector,
    const Matrix3& rotationLocalToWorld) noexcept
{
    return rotationLocalToWorld * localVector;
}

// world vector is transferred into local coordinate system.
// For valid rotation matrix R, R^{-1} = R^T。
[[nodiscard]] inline Vector3 worldToLocalVector(
    const Vector3& worldVector,
    const Matrix3& rotationLocalToWorld) noexcept
{
    return rotationLocalToWorld.transpose() * worldVector;
}

// point in local coordinate system is transferred into world coordinate system. 
//
//     p_world = origin_world + R_world_local * p_local
[[nodiscard]] inline Vector3 localToWorldPoint(
    const Vector3& localPoint,
    const Vector3& localOriginInWorld,
    const Matrix3& rotationLocalToWorld) noexcept
{
    return localOriginInWorld
        + rotationLocalToWorld * localPoint;
}

// point in world coordinate system is transferred into local coordinate system. 
//
//     p_local = R_world_local^T
//             * (p_world - origin_world)
[[nodiscard]] inline Vector3 worldToLocalPoint(
    const Vector3& worldPoint,
    const Vector3& localOriginInWorld,
    const Matrix3& rotationLocalToWorld) noexcept
{
    return rotationLocalToWorld.transpose()
        * (worldPoint - localOriginInWorld);
}

} // namespace gy::physics::math