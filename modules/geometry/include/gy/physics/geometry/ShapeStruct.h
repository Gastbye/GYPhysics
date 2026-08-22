/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include "gy/physics/math/MathTypes.h"

namespace gy::physics::geometry {

struct SolidGeometryProperties
{
    math::Real volume{math::kZero};

    // Geometric centroid of the shape: local coordinates.
    math::Vector3 centroid{math::Vector3::Zero()};

    // Unit-density inertia tensor about the geometric centroid: local coordinates.
    math::Matrix3 unitDensityInertiaAtCentroid{math::Matrix3::Zero()};
};

struct ShapeId
{
    math::Index value{math::InvalidIndex};

    constexpr ShapeId() noexcept = default;

    explicit constexpr ShapeId(math::Index index) noexcept
        : value(index)
    {
    }

    [[nodiscard]] constexpr bool isValid() const noexcept
    {
        return math::isValidIndex(value);
    }

    friend constexpr bool operator==(
        ShapeId lhs,
        ShapeId rhs) noexcept
    {
        return lhs.value == rhs.value;
    }

    friend constexpr bool operator!=(
        ShapeId lhs,
        ShapeId rhs) noexcept
    {
        return !(lhs == rhs);
    }
};

} // namespace gy::physics::geometry
