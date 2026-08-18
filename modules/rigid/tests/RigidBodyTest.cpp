#include <cmath>
#include <iostream>

#include "gy/physics/rigid/RigidBody.h"

int main()
{
    namespace math = gy::physics::math;
    namespace rigid = gy::physics::rigid;

    rigid::RigidBodyDesc desc;
    desc.massProperties.mass = static_cast<math::Real>(2);
    desc.massProperties.inertiaTensorLocalAtCenterOfMass =
        static_cast<math::Real>(3)
        * math::Matrix3::Identity();
    desc.initialState.position = math::Vector3(
        static_cast<math::Real>(1),
        static_cast<math::Real>(2),
        static_cast<math::Real>(3)
    );

    rigid::RigidBody body(desc);
    const math::Real tolerance = static_cast<math::Real>(100) * math::kEpsilon;

    if (std::abs(body.mass() - static_cast<math::Real>(2)) > tolerance) {
        std::cerr << "RigidBody mass test failed.\n";
        return 1;
    }

    if (std::abs(body.inverseMass() - static_cast<math::Real>(0.5))
        > tolerance) {
        std::cerr << "RigidBody inverse-mass test failed.\n";
        return 1;
    }

    if (!body.state().position.isApprox(desc.initialState.position, tolerance)) {
        std::cerr << "RigidBody initial-state test failed.\n";
        return 1;
    }

    const math::Matrix3 expectedInverseInertia =
        (static_cast<math::Real>(1) / static_cast<math::Real>(3))
        * math::Matrix3::Identity();
    if (!body.inverseInertiaTensor().isApprox(
            expectedInverseInertia,
            tolerance)) {
        std::cerr << "RigidBody inverse-inertia test failed.\n";
        return 1;
    }

    const math::Vector3 force(
        static_cast<math::Real>(4),
        static_cast<math::Real>(5),
        static_cast<math::Real>(6)
    );
    body.addForce(force);
    if (!body.accumulatedForce().isApprox(force, tolerance)) {
        std::cerr << "RigidBody force-accumulation test failed.\n";
        return 1;
    }

    body.clearForceAndTorque();
    if (!body.accumulatedForce().isZero(tolerance)) {
        std::cerr << "RigidBody force-clear test failed.\n";
        return 1;
    }

    std::cout << "GyPhysics Rigid test passed with Real="
              << math::realTypeName() << '\n';
    return 0;
}
