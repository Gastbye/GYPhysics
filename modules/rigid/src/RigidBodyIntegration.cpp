#include "gy/physics/rigid/RigidBodyIntegration.h"

#include <cmath>
#include <stdexcept>

#include "gy/physics/rigid/RigidBody.h"
#include "gy/physics/rigid/RigidBodyStruct.h"

namespace gy::physics::rigid {

void integrateVelocities(
    RigidBody& body,
    math::Real timeStep)
{
    if (!std::isfinite(timeStep)
        || timeStep <= math::kZero) {
        throw std::invalid_argument(
            "Rigid-body time step must be finite and positive."
        );
    }

    RigidBodyState& state = body.state();

    const math::Vector3 linearAcceleration =
        body.inverseMass()
        * body.accumulatedForce();

    state.linearVelocity +=
        linearAcceleration * timeStep;

    const math::Matrix3 inertiaWorld =
        body.inertiaTensorWorldAtCenterOfMass();

    const math::Matrix3 inverseInertiaWorld =
        body.inverseInertiaTensorWorldAtCenterOfMass();

    const math::Vector3 angularMomentumWorld =
        inertiaWorld * state.angularVelocity;

    const math::Vector3 gyroscopicTorque =
        state.angularVelocity.cross(
            angularMomentumWorld
        );

    const math::Vector3 angularAcceleration =
        inverseInertiaWorld
        * (body.accumulatedTorque()
           - gyroscopicTorque);

    state.angularVelocity +=
        angularAcceleration * timeStep;

    // const math::Vector3 angularAcceleration =
    //     body.inverseInertiaTensorWorldAtCenterOfMass()
    //     * body.accumulatedTorque();
    // state.angularVelocity +=
    //     angularAcceleration * timeStep;

    body.clearForceAndTorque();
}

void integrateOrientation(
    RigidBodyState& state,
    math::Real timeStep)
{
    const math::Vector3 rotationVector =
        state.angularVelocity * timeStep;

    const math::Real rotationAngle =
        rotationVector.norm();

    if (rotationAngle == math::kZero) {
        return;
    }

    const math::Vector3 rotationAxis =
        rotationVector / rotationAngle;

    const math::Real halfAngle =
        math::Real{0.5} * rotationAngle;

    const math::Real sinHalfAngle =
        std::sin(halfAngle);

    const math::Quaternion deltaOrientation(
        std::cos(halfAngle),
        rotationAxis.x() * sinHalfAngle,
        rotationAxis.y() * sinHalfAngle,
        rotationAxis.z() * sinHalfAngle
    );

    state.orientation =
        deltaOrientation * state.orientation;

    state.orientation.normalize();
}
    
void integratePose(
    RigidBody& body,
    math::Real timeStep)
{
    if (!std::isfinite(timeStep)
        || timeStep <= math::kZero) {
        throw std::invalid_argument(
            "Rigid-body time step must be finite and positive."
        );
    }

    RigidBodyState& state = body.state();

    state.position +=
        state.linearVelocity * timeStep;

    integrateOrientation(state, timeStep);
}

void integrateRigidBody(
    RigidBody& body,
    math::Real timeStep)
{
    integrateVelocities(body, timeStep);
    integratePose(body, timeStep);
}

} // namespace gy::physics::rigid