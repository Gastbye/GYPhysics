#include "gy/physics/rigid/RigidBody.h"

#include <stdexcept>

#include <Eigen/LU>

namespace gy::physics::rigid {

RigidBody::RigidBody(const RigidBodyDesc& desc)
{
    setMass(desc.mass);
    setInertiaTensor(desc.inertiaTensor);
    setState(desc.initialState);
}

math::Real RigidBody::mass() const noexcept
{
    return mass_;
}

math::Real RigidBody::inverseMass() const noexcept
{
    return inverseMass_;
}

void RigidBody::setMass(math::Real mass)
{
    if (mass <= math::kZero) {
        throw std::invalid_argument("RigidBody mass must be greater than zero");
    }

    mass_ = mass;
    inverseMass_ = math::kOne / mass_;
}

const math::Matrix3& RigidBody::inertiaTensor() const noexcept
{
    return inertiaTensor_;
}

const math::Matrix3& RigidBody::inverseInertiaTensor() const noexcept
{
    return inverseInertiaTensor_;
}

void RigidBody::setInertiaTensor(const math::Matrix3& inertiaTensor)
{
    const Eigen::FullPivLU<math::Matrix3> decomposition(inertiaTensor);
    if (!decomposition.isInvertible()) {
        throw std::invalid_argument("RigidBody inertia tensor must be invertible");
    }

    inertiaTensor_ = inertiaTensor;
    inverseInertiaTensor_ = decomposition.inverse();
}

const RigidBodyState& RigidBody::state() const noexcept
{
    return state_;
}

RigidBodyState& RigidBody::state() noexcept
{
    return state_;
}

void RigidBody::setState(const RigidBodyState& state)
{
    state_ = state;
    state_.orientation.normalize();
}

void RigidBody::addForce(const math::Vector3& force) noexcept
{
    accumulatedForce_ += force;
}

void RigidBody::addTorque(const math::Vector3& torque) noexcept
{
    accumulatedTorque_ += torque;
}

void RigidBody::clearForceAndTorque() noexcept
{
    accumulatedForce_.setZero();
    accumulatedTorque_.setZero();
}

const math::Vector3& RigidBody::accumulatedForce() const noexcept
{
    return accumulatedForce_;
}

const math::Vector3& RigidBody::accumulatedTorque() const noexcept
{
    return accumulatedTorque_;
}

} // namespace gy::physics::rigid
