#include "gy/physics/rigid/RigidBody.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace gy::physics::rigid {

namespace {

[[nodiscard]] constexpr math::Real validationFactor() noexcept
{
    return math::Real{100};
}

[[nodiscard]] math::Real matrixTolerance(
    const math::Matrix3& matrix) noexcept
{
    const math::Real scale = std::max(
        math::kOne,
        matrix.cwiseAbs().maxCoeff()
    );
    return validationFactor() * std::numeric_limits<math::Real>::epsilon()
        * scale;
}

[[nodiscard]] RigidBodyState validatedState(const RigidBodyState& state)
{
    if (!state.orientation.coeffs().allFinite()) {
        throw std::invalid_argument(
            "RigidBody orientation must contain only finite values"
        );
    }

    const math::Real largestCoefficient =
        state.orientation.coeffs().cwiseAbs().maxCoeff();
    if (!(largestCoefficient > math::kZero)) {
        throw std::invalid_argument(
            "RigidBody orientation must have non-zero norm"
        );
    }

    RigidBodyState result = state;
    result.orientation.coeffs() /= largestCoefficient;
    const math::Real scaledNorm = result.orientation.norm();
    if (!std::isfinite(scaledNorm) || !(scaledNorm > math::kZero)) {
        throw std::invalid_argument(
            "RigidBody orientation must have finite non-zero norm"
        );
    }
    result.orientation.coeffs() /= scaledNorm;
    return result;
}

} // namespace

RigidBody::RigidBody(const RigidBodyDesc& desc)
{
    setMassProperties(desc.massProperties);
    setState(desc.initialState);
}

void RigidBody::setMassProperties(
    const mechanics::MassProperties& massProperties)
{
    if (!std::isfinite(massProperties.mass)
        || !(massProperties.mass > math::kZero)) {
        throw std::invalid_argument(
            "RigidBody mass must be finite and greater than zero"
        );
    }

    if (!massProperties.centerOfMassLocalPosition.allFinite()) {
        throw std::invalid_argument(
            "RigidBody local center of mass must contain only finite values"
        );
    }

    const math::Matrix3& inputInertia =
        massProperties.inertiaTensorLocalAtCenterOfMass;
    if (!inputInertia.allFinite()) {
        throw std::invalid_argument(
            "RigidBody inertia tensor must contain only finite values"
        );
    }

    const math::Real tolerance = matrixTolerance(inputInertia);
    if ((inputInertia - inputInertia.transpose()).cwiseAbs().maxCoeff()
        > tolerance) {
        throw std::invalid_argument(
            "RigidBody inertia tensor must be symmetric"
        );
    }

    const math::Matrix3 inertia =
        math::Real{0.5} * (inputInertia + inputInertia.transpose());
    const Eigen::SelfAdjointEigenSolver<math::Matrix3> eigenSolver(
        inertia,
        Eigen::EigenvaluesOnly
    );
    if (eigenSolver.info() != Eigen::Success
        || !eigenSolver.eigenvalues().allFinite()) {
        throw std::invalid_argument(
            "RigidBody inertia tensor eigenvalues could not be computed"
        );
    }

    const auto& principalMoments = eigenSolver.eigenvalues();
    if (!(principalMoments[0] > tolerance)) {
        throw std::invalid_argument(
            "RigidBody inertia tensor must be positive definite"
        );
    }
    if (principalMoments[2]
        > principalMoments[0] + principalMoments[1] + tolerance) {
        throw std::invalid_argument(
            "RigidBody inertia tensor violates the principal-moment "
            "triangle inequality"
        );
    }

    const Eigen::LLT<math::Matrix3> decomposition(inertia);
    if (decomposition.info() != Eigen::Success) {
        throw std::invalid_argument(
            "RigidBody inertia tensor must be invertible"
        );
    }

    const math::Real inverseMass = math::kOne / massProperties.mass;
    math::Matrix3 inverseInertia =
        decomposition.solve(math::Matrix3::Identity());
    inverseInertia = math::Real{0.5}
        * (inverseInertia + inverseInertia.transpose());
    if (!std::isfinite(inverseMass) || !inverseInertia.allFinite()) {
        throw std::invalid_argument(
            "RigidBody inverse mass properties must be finite"
        );
    }

    // Commit only after every validation and derived calculation succeeds.
    mass_ = massProperties.mass;
    inverseMass_ = inverseMass;
    centerOfMassLocalPosition_ = massProperties.centerOfMassLocalPosition;
    inertiaTensorLocalAtCenterOfMass_ = inertia;
    inverseInertiaTensorLocalAtCenterOfMass_ = inverseInertia;
}

math::Real RigidBody::mass() const noexcept
{
    return mass_;
}

math::Real RigidBody::inverseMass() const noexcept
{
    return inverseMass_;
}

const math::Vector3&
RigidBody::centerOfMassLocalPosition() const noexcept
{
    return centerOfMassLocalPosition_;
}

const math::Matrix3&
RigidBody::inertiaTensorLocalAtCenterOfMass() const noexcept
{
    return inertiaTensorLocalAtCenterOfMass_;
}

const math::Matrix3&
RigidBody::inverseInertiaTensorLocalAtCenterOfMass() const noexcept
{
    return inverseInertiaTensorLocalAtCenterOfMass_;
}

RigidBodyState& RigidBody::state() noexcept
{
    return state_;
}

const RigidBodyState& RigidBody::state() const noexcept
{
    return state_;
}

void RigidBody::setState(const RigidBodyState& state)
{
    state_ = validatedState(state);
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

math::Vector3 RigidBody::worldCenterOfMass() const noexcept
{
    return state_.position;
}

math::Vector3 RigidBody::bodyOriginWorldPosition() const noexcept
{
    const math::Matrix3 rotationMatrix =
        state_.orientation.toRotationMatrix();

    return state_.position
        - rotationMatrix * centerOfMassLocalPosition_;
}

math::Matrix3 RigidBody::inertiaTensorWorldAtCenterOfMass() const noexcept
{
    const math::Matrix3 rotationMatrix =
        state_.orientation.toRotationMatrix();
    return rotationMatrix
        * inertiaTensorLocalAtCenterOfMass_
        * rotationMatrix.transpose();
}

math::Matrix3 RigidBody::inverseInertiaTensorWorldAtCenterOfMass() const noexcept
{
    const math::Matrix3 rotationMatrix =
        state_.orientation.toRotationMatrix();
    return rotationMatrix
        * inverseInertiaTensorLocalAtCenterOfMass_
        * rotationMatrix.transpose();
}
} // namespace gy::physics::rigid
