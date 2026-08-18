#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "gy/physics/rigid/RigidBody.h"

namespace {

namespace math = gy::physics::math;
namespace mechanics = gy::physics::mechanics;
namespace rigid = gy::physics::rigid;

[[nodiscard]] constexpr math::Real testTolerance() noexcept
{
#if GY_PHYSICS_REAL_IS_DOUBLE == 1
    return math::Real{1.0e-10};
#else
    return math::Real{2.0e-4F};
#endif
}

void requireNear(
    math::Real actual,
    math::Real expected,
    const std::string& message)
{
    const math::Real scale = std::max(math::kOne, std::abs(expected));
    if (std::abs(actual - expected) > testTolerance() * scale) {
        throw std::runtime_error(message);
    }
}

void requireNear(
    const math::Vector3& actual,
    const math::Vector3& expected,
    const std::string& message)
{
    const math::Real scale = std::max(math::kOne, expected.norm());
    if ((actual - expected).norm() > testTolerance() * scale) {
        throw std::runtime_error(message);
    }
}

void requireNear(
    const math::Matrix3& actual,
    const math::Matrix3& expected,
    const std::string& message)
{
    const math::Real scale = std::max(math::kOne, expected.norm());
    if ((actual - expected).norm() > testTolerance() * scale) {
        throw std::runtime_error(message);
    }
}

template<typename Exception, typename Function>
void requireThrows(Function&& function, const std::string& message)
{
    try {
        std::forward<Function>(function)();
    } catch (const Exception&) {
        return;
    } catch (...) {
        throw std::runtime_error(message + " (wrong exception type)");
    }

    throw std::runtime_error(message + " (no exception)");
}

[[nodiscard]] math::Matrix3 diagonalInertia(
    math::Real x,
    math::Real y,
    math::Real z)
{
    return math::Vector3(x, y, z).asDiagonal();
}

[[nodiscard]] mechanics::MassProperties validMassProperties()
{
    mechanics::MassProperties properties;
    properties.mass = math::Real{2};
    properties.centerOfMassLocalPosition = math::Vector3(
        math::Real{1}, math::Real{2}, math::Real{3}
    );
    properties.inertiaTensorLocalAtCenterOfMass = diagonalInertia(
        math::Real{2}, math::Real{3}, math::Real{4}
    );
    return properties;
}

[[nodiscard]] math::Quaternion zRotation(math::Real angle)
{
    return math::Quaternion(
        Eigen::AngleAxis<math::Real>(angle, math::Vector3::UnitZ())
    );
}

void testDefaults()
{
    const rigid::RigidBody body;

    requireNear(body.mass(), math::kOne, "Default mass is incorrect.");
    requireNear(body.inverseMass(), math::kOne,
                "Default inverse mass is incorrect.");
    requireNear(body.centerOfMassLocalPosition(), math::Vector3::Zero(),
                "Default local center of mass is incorrect.");
    requireNear(body.inertiaTensorLocalAtCenterOfMass(),
                math::Matrix3::Identity(),
                "Default local inertia tensor is incorrect.");
    requireNear(body.inverseInertiaTensorLocalAtCenterOfMass(),
                math::Matrix3::Identity(),
                "Default inverse local inertia tensor is incorrect.");
    requireNear(body.worldCenterOfMass(), math::Vector3::Zero(),
                "Default world center of mass is incorrect.");
    requireNear(body.bodyOriginWorldPosition(), math::Vector3::Zero(),
                "Default body origin is incorrect.");
    requireNear(body.inertiaTensorWorldAtCenterOfMass(),
                math::Matrix3::Identity(),
                "Default world inertia tensor is incorrect.");
}

void testConstructionFromMassProperties()
{
    rigid::RigidBodyDesc desc;
    desc.massProperties = validMassProperties();
    desc.initialState.position = math::Vector3(
        math::Real{10}, math::Real{20}, math::Real{30}
    );
    const rigid::RigidBody body(desc);

    requireNear(body.mass(), math::Real{2},
                "RigidBody did not read mass properties.");
    requireNear(body.inverseMass(), math::Real{0.5},
                "Mass two must produce inverse mass one half.");
    requireNear(body.centerOfMassLocalPosition(),
                desc.massProperties.centerOfMassLocalPosition,
                "RigidBody did not preserve the local center of mass.");
    requireNear(body.inertiaTensorLocalAtCenterOfMass(),
                desc.massProperties.inertiaTensorLocalAtCenterOfMass,
                "RigidBody did not preserve the local inertia tensor.");
    requireNear(body.inverseInertiaTensorLocalAtCenterOfMass(),
                diagonalInertia(
                    math::Real{0.5},
                    math::Real{1} / math::Real{3},
                    math::Real{0.25}
                ),
                "RigidBody computed the wrong inverse inertia tensor.");
    requireNear(body.worldCenterOfMass(), desc.initialState.position,
                "World center of mass must equal state.position.");
}

void testDerivedPositions()
{
    rigid::RigidBodyDesc desc;
    desc.massProperties = validMassProperties();
    desc.massProperties.centerOfMassLocalPosition = math::Vector3(
        math::Real{1}, math::Real{2}, math::Real{0}
    );
    desc.initialState.position = math::Vector3(
        math::Real{10}, math::Real{20}, math::Real{30}
    );

    rigid::RigidBody body(desc);
    requireNear(body.bodyOriginWorldPosition(),
                math::Vector3(
                    math::Real{9}, math::Real{18}, math::Real{30}
                ),
                "Body origin is incorrect for identity orientation.");

    rigid::RigidBodyState rotatedState = body.state();
    const math::Real halfPi = std::acos(math::Real{-1}) / math::Real{2};
    rotatedState.orientation = zRotation(halfPi);
    body.setState(rotatedState);

    requireNear(body.worldCenterOfMass(), rotatedState.position,
                "Rotation must not change state.position semantics.");
    requireNear(body.bodyOriginWorldPosition(),
                math::Vector3(
                    math::Real{12}, math::Real{19}, math::Real{30}
                ),
                "Body origin is incorrect for a 90-degree Z rotation.");
}

void testWorldInertia()
{
    rigid::RigidBodyDesc desc;
    desc.massProperties = validMassProperties();
    rigid::RigidBody body(desc);

    requireNear(body.inertiaTensorWorldAtCenterOfMass(),
                desc.massProperties.inertiaTensorLocalAtCenterOfMass,
                "Identity rotation changed the inertia tensor.");

    rigid::RigidBodyState state = body.state();
    const math::Real halfPi = std::acos(math::Real{-1}) / math::Real{2};
    state.orientation = zRotation(halfPi);
    body.setState(state);

    requireNear(body.inertiaTensorWorldAtCenterOfMass(),
                diagonalInertia(math::Real{3}, math::Real{2}, math::Real{4}),
                "Z rotation did not exchange the X/Y principal moments.");
    const math::Matrix3 inertiaTimesInverse =
        body.inertiaTensorWorldAtCenterOfMass()
        * body.inverseInertiaTensorWorldAtCenterOfMass();
    requireNear(
        inertiaTimesInverse,
        math::Matrix3::Identity(),
        "World inertia multiplied by its cached rotated inverse is not identity."
    );
}

void testSetMassPropertiesAndStrongExceptionGuarantee()
{
    rigid::RigidBody body;
    const mechanics::MassProperties replacement = validMassProperties();
    body.setMassProperties(replacement);

    requireNear(body.mass(), replacement.mass,
                "setMassProperties did not update mass.");
    requireNear(body.inverseMass(), math::Real{0.5},
                "setMassProperties did not update inverse mass.");
    requireNear(body.centerOfMassLocalPosition(),
                replacement.centerOfMassLocalPosition,
                "setMassProperties did not update the local center of mass.");
    requireNear(body.inertiaTensorLocalAtCenterOfMass(),
                replacement.inertiaTensorLocalAtCenterOfMass,
                "setMassProperties did not update local inertia.");

    mechanics::MassProperties invalid = replacement;
    invalid.mass = math::Real{-1};
    requireThrows<std::invalid_argument>(
        [&body, &invalid] { body.setMassProperties(invalid); },
        "setMassProperties must reject a negative mass."
    );
    requireNear(body.mass(), replacement.mass,
                "A rejected update changed the stored mass.");
    requireNear(body.centerOfMassLocalPosition(),
                replacement.centerOfMassLocalPosition,
                "A rejected update changed the stored center of mass.");
    requireNear(body.inertiaTensorLocalAtCenterOfMass(),
                replacement.inertiaTensorLocalAtCenterOfMass,
                "A rejected update changed the stored inertia tensor.");
}

void testInvalidMassProperties()
{
    const auto requireRejected = [](
        const mechanics::MassProperties& properties,
        const std::string& message) {
        requireThrows<std::invalid_argument>(
            [&properties] {
                rigid::RigidBodyDesc desc;
                desc.massProperties = properties;
                static_cast<void>(rigid::RigidBody(desc));
            },
            message
        );
    };

    mechanics::MassProperties properties = validMassProperties();
    properties.mass = math::kZero;
    requireRejected(properties, "Zero mass must be rejected.");
    properties.mass = math::Real{-1};
    requireRejected(properties, "Negative mass must be rejected.");
    properties.mass = std::numeric_limits<math::Real>::infinity();
    requireRejected(properties, "Infinite mass must be rejected.");
    properties.mass = std::numeric_limits<math::Real>::quiet_NaN();
    requireRejected(properties, "NaN mass must be rejected.");

    properties = validMassProperties();
    properties.centerOfMassLocalPosition.x() =
        std::numeric_limits<math::Real>::quiet_NaN();
    requireRejected(properties, "A NaN center of mass must be rejected.");
    properties = validMassProperties();
    properties.centerOfMassLocalPosition.z() =
        std::numeric_limits<math::Real>::infinity();
    requireRejected(properties, "An infinite center of mass must be rejected.");

    properties = validMassProperties();
    properties.inertiaTensorLocalAtCenterOfMass(0, 0) =
        std::numeric_limits<math::Real>::quiet_NaN();
    requireRejected(properties, "A NaN inertia tensor must be rejected.");
    properties = validMassProperties();
    properties.inertiaTensorLocalAtCenterOfMass(1, 1) =
        std::numeric_limits<math::Real>::infinity();
    requireRejected(properties, "An infinite inertia tensor must be rejected.");

    properties = validMassProperties();
    properties.inertiaTensorLocalAtCenterOfMass(0, 1) = math::Real{0.25};
    requireRejected(properties, "A non-symmetric inertia tensor must be rejected.");
    properties = validMassProperties();
    properties.inertiaTensorLocalAtCenterOfMass = diagonalInertia(
        math::Real{1}, math::Real{1}, math::kZero
    );
    requireRejected(properties, "A singular inertia tensor must be rejected.");
    properties = validMassProperties();
    properties.inertiaTensorLocalAtCenterOfMass = diagonalInertia(
        math::Real{1}, math::Real{1}, math::Real{-1}
    );
    requireRejected(properties, "An indefinite inertia tensor must be rejected.");
    properties = validMassProperties();
    properties.inertiaTensorLocalAtCenterOfMass = diagonalInertia(
        math::Real{1}, math::Real{1}, math::Real{3}
    );
    requireRejected(
        properties,
        "Physically invalid principal moments must be rejected."
    );
}

void testOrientationValidationAndNormalization()
{
    rigid::RigidBodyDesc desc;
    desc.initialState.orientation = math::Quaternion(
        math::Real{2}, math::kZero, math::kZero, math::kZero
    );
    rigid::RigidBody body(desc);
    requireNear(body.state().orientation.norm(), math::kOne,
                "Construction did not normalize orientation.");
    requireNear(body.state().orientation.toRotationMatrix(),
                math::Matrix3::Identity(),
                "Normalized construction orientation changed rotation.");

    rigid::RigidBodyState state = body.state();
    const math::Real halfPi = std::acos(math::Real{-1}) / math::Real{2};
    state.orientation = zRotation(halfPi);
    state.orientation.coeffs() *= math::Real{5};
    body.setState(state);
    requireNear(body.state().orientation.norm(), math::kOne,
                "setState did not normalize orientation.");
    requireNear(body.state().orientation.toRotationMatrix(),
                zRotation(halfPi).toRotationMatrix(),
                "setState normalization changed the represented rotation.");

    const rigid::RigidBodyState validState = body.state();
    rigid::RigidBodyState invalidState = validState;
    invalidState.orientation.coeffs().setZero();
    requireThrows<std::invalid_argument>(
        [&body, &invalidState] { body.setState(invalidState); },
        "A zero quaternion must be rejected."
    );
    requireNear(body.state().orientation.toRotationMatrix(),
                validState.orientation.toRotationMatrix(),
                "A rejected zero quaternion changed the body state.");

    invalidState = validState;
    invalidState.orientation.coeffs()[0] =
        std::numeric_limits<math::Real>::quiet_NaN();
    requireThrows<std::invalid_argument>(
        [&body, &invalidState] { body.setState(invalidState); },
        "A NaN quaternion must be rejected."
    );
    invalidState = validState;
    invalidState.orientation.coeffs()[1] =
        std::numeric_limits<math::Real>::infinity();
    requireThrows<std::invalid_argument>(
        [&body, &invalidState] { body.setState(invalidState); },
        "An infinite quaternion must be rejected."
    );
}

void testForceAndTorqueAccumulation()
{
    rigid::RigidBody body;
    const math::Vector3 firstForce(
        math::Real{1}, math::Real{2}, math::Real{3}
    );
    const math::Vector3 secondForce(
        math::Real{4}, math::Real{5}, math::Real{6}
    );
    const math::Vector3 firstTorque(
        math::Real{7}, math::Real{8}, math::Real{9}
    );
    const math::Vector3 secondTorque(
        math::Real{1}, math::Real{3}, math::Real{5}
    );

    body.addForce(firstForce);
    body.addForce(secondForce);
    body.addTorque(firstTorque);
    body.addTorque(secondTorque);
    requireNear(body.accumulatedForce(), firstForce + secondForce,
                "Forces did not accumulate.");
    requireNear(body.accumulatedTorque(), firstTorque + secondTorque,
                "Torques did not accumulate.");

    body.clearForceAndTorque();
    requireNear(body.accumulatedForce(), math::Vector3::Zero(),
                "Force accumulator did not clear.");
    requireNear(body.accumulatedTorque(), math::Vector3::Zero(),
                "Torque accumulator did not clear.");
}

} // namespace

int main()
{
    try {
        testDefaults();
        testConstructionFromMassProperties();
        testDerivedPositions();
        testWorldInertia();
        testSetMassPropertiesAndStrongExceptionGuarantee();
        testInvalidMassProperties();
        testOrientationValidationAndNormalization();
        testForceAndTorqueAccumulation();
    } catch (const std::exception& error) {
        std::cerr << "RigidBody test failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "GyPhysics RigidBody tests passed with Real="
              << math::realTypeName() << '\n';
    return 0;
}
