/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include "gy/physics/math/MathTypes.h"

namespace gy::physics::rigid {

struct RigidBodyState
{
    math::Vector3 position{math::Vector3::Zero()};
    math::Quaternion orientation{math::Quaternion::Identity()};
    math::Vector3 linearVelocity{math::Vector3::Zero()};
    math::Vector3 angularVelocity{math::Vector3::Zero()};
};

struct RigidBodyDesc
{
    math::Real mass{math::kOne};
    math::Matrix3 inertiaTensor{math::Matrix3::Identity()};
    RigidBodyState initialState{};
};

class RigidBody
{
public:
    explicit RigidBody(const RigidBodyDesc& desc = RigidBodyDesc{});

    math::Real mass() const noexcept;
    math::Real inverseMass() const noexcept;
    void setMass(math::Real mass);

    const math::Matrix3& inertiaTensor() const noexcept;
    const math::Matrix3& inverseInertiaTensor() const noexcept;
    void setInertiaTensor(const math::Matrix3& inertiaTensor);

    const RigidBodyState& state() const noexcept;
    RigidBodyState& state() noexcept;
    void setState(const RigidBodyState& state);

    void addForce(const math::Vector3& force) noexcept;
    void addTorque(const math::Vector3& torque) noexcept;
    void clearForceAndTorque() noexcept;

    const math::Vector3& accumulatedForce() const noexcept;
    const math::Vector3& accumulatedTorque() const noexcept;

private:
    math::Real mass_{math::kOne};
    math::Real inverseMass_{math::kOne};

    math::Matrix3 inertiaTensor_{math::Matrix3::Identity()};
    math::Matrix3 inverseInertiaTensor_{math::Matrix3::Identity()};

    RigidBodyState state_{};
    math::Vector3 accumulatedForce_{math::Vector3::Zero()};
    math::Vector3 accumulatedTorque_{math::Vector3::Zero()};
};

} // namespace gy::physics::rigid
