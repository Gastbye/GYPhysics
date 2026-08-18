#include "gy/physics/rigid/RigidBodyIntegration.h"

#include <cmath>
#include <stdexcept>

#include "gy/physics/rigid/RigidBody.h"
#include "gy/physics/rigid/RigidBodyStruct.h"

namespace gy::physics::rigid {

void integrateRigidBody(
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

    state.position +=
        state.linearVelocity * timeStep;

    const math::Vector3 angularAcceleration =
        body.inverseInertiaTensorWorldAtCenterOfMass()
        * body.accumulatedTorque();

    state.angularVelocity +=
        angularAcceleration * timeStep;

    body.clearForceAndTorque();
}

} // namespace gy::physics::rigid