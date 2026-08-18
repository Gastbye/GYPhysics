/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include "gy/physics/math/MathTypes.h"
#include "gy/physics/mechanics/Mechanics.h"

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

class RigidBody
{
public:
    explicit RigidBody(const RigidBodyDesc& desc = RigidBodyDesc{});

    void setMassProperties(const mechanics::MassProperties& massProperties);

    [[nodiscard]] math::Real mass() const noexcept;
    [[nodiscard]] math::Real inverseMass() const noexcept;

    [[nodiscard]] const math::Vector3&
    centerOfMassLocalPosition() const noexcept;

    [[nodiscard]] const math::Matrix3&
    inertiaTensorLocalAtCenterOfMass() const noexcept;

    [[nodiscard]] const math::Matrix3&
    inverseInertiaTensorLocalAtCenterOfMass() const noexcept;

    [[nodiscard]] const RigidBodyState& state() const noexcept;
    void setState(const RigidBodyState& state);

    // Accumulated force and torque are expressed in world-space axes.
    void addForce(const math::Vector3& force) noexcept;
    void addTorque(const math::Vector3& torque) noexcept;
    void clearForceAndTorque() noexcept;

    [[nodiscard]] const math::Vector3&
    accumulatedForce() const noexcept;

    [[nodiscard]] const math::Vector3&
    accumulatedTorque() const noexcept;

    [[nodiscard]] math::Vector3
    worldCenterOfMass() const noexcept;

    [[nodiscard]] math::Vector3
    bodyOriginWorldPosition() const noexcept;

    [[nodiscard]] math::Matrix3
    inertiaTensorWorldAtCenterOfMass() const noexcept;

    [[nodiscard]] math::Matrix3
    inverseInertiaTensorWorldAtCenterOfMass() const noexcept;

private:
    math::Real mass_{math::kOne};
    math::Real inverseMass_{math::kOne};

    math::Vector3 centerOfMassLocalPosition_{
        math::Vector3::Zero()
    };

    math::Matrix3 inertiaTensorLocalAtCenterOfMass_{
        math::Matrix3::Identity()
    };

    math::Matrix3 inverseInertiaTensorLocalAtCenterOfMass_{
        math::Matrix3::Identity()
    };

    RigidBodyState state_{};
    math::Vector3 accumulatedForce_{math::Vector3::Zero()};
    math::Vector3 accumulatedTorque_{math::Vector3::Zero()};
};

} // namespace gy::physics::rigid
