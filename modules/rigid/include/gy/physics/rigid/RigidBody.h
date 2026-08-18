/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include "gy/physics/rigid/RigidBodyStruct.h"
#include "gy/physics/math/MathTypes.h"
#include "gy/physics/mechanics/Mechanics.h"

namespace gy::physics::rigid {

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

    [[nodiscard]] RigidBodyState& state() noexcept;
    [[nodiscard]] const RigidBodyState& state() const noexcept;
    void setState(const RigidBodyState& state);

    // Accumulated force and torque are expressed in world-space axes.
    void addForce(const math::Vector3& force) noexcept;
    void addForce(const math::Vector3& force, const math::Vector3& position) noexcept;
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
