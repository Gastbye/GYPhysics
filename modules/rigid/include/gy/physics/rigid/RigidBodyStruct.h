/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include "gy/physics/math/MathTypes.h"
#include "gy/physics/mechanics/Mechanics.h"
#include "gy/physics/geometry/ShapeStruct.h"
#include "gy/physics/material/MaterialStruct.h"

namespace gy::physics::rigid {

struct RigidBodyState
{
    // World-space position of the center of mass.
    math::Vector3 position{math::Vector3::Zero()};

    // Rotation from the body-local coordinate system to world space.
    math::Quaternion orientation{math::Quaternion::Identity()};

    // Center-of-mass linear velocity expressed in world-space axes.
    math::Vector3 linearVelocity{math::Vector3::Zero()};

    // Angular velocity expressed in world-space axes.
    math::Vector3 angularVelocity{math::Vector3::Zero()};
};

struct RigidBodyDesc
{
    mechanics::MassProperties massProperties{};
    RigidBodyState initialState{};
};

struct BodyId
{
    math::Index value{math::InvalidIndex};

    constexpr BodyId() noexcept = default;

    explicit constexpr BodyId(math::Index index) noexcept
        : value(index)
    {
    }

    [[nodiscard]] constexpr bool isValid() const noexcept
    {
        return math::isValidIndex(value);
    }

    friend constexpr bool operator==(
        BodyId lhs,
        BodyId rhs) noexcept
    {
        return lhs.value == rhs.value;
    }

    friend constexpr bool operator!=(
        BodyId lhs,
        BodyId rhs) noexcept
    {
        return !(lhs == rhs);
    }
};

struct RigidBodyShapeAttachment
{
    geometry::ShapeId shapeId{};
    material::MaterialId materialId{};

    // Position relative to the actor reference frame.
    math::Vector3 localPosition{
        math::Vector3::Zero()
    };

    // Orientation from the shape frame to the actor frame.
    math::Quaternion localOrientation{
        math::Quaternion::Identity()
    };
};

} // namespace gy::physics::rigid
