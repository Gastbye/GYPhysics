#include <cmath>
#include <iostream>
#include <stdexcept>

#include "gy/physics/math/QuaternionOperators.h"

namespace {

namespace math = gy::physics::math;

[[nodiscard]] constexpr math::Real testTolerance() noexcept
{
#if GY_PHYSICS_REAL_IS_DOUBLE == 1
    return math::Real{1.0e-12};
#else
    return math::Real{1.0e-5};
#endif
}

[[nodiscard]] bool nearlyEqual(
    math::Real lhs,
    math::Real rhs,
    math::Real tolerance = testTolerance()) noexcept
{
    return std::abs(lhs - rhs) <= tolerance;
}

[[nodiscard]] bool isIdentityQuaternion(
    const math::Quaternion& quaternion,
    math::Real tolerance = testTolerance()) noexcept
{
    return nearlyEqual(quaternion.w(), math::kOne, tolerance)
        && nearlyEqual(quaternion.x(), math::kZero, tolerance)
        && nearlyEqual(quaternion.y(), math::kZero, tolerance)
        && nearlyEqual(quaternion.z(), math::kZero, tolerance);
}

} // namespace

int main()
{
    const math::Real tolerance = testTolerance();
    const math::Real pi = std::acos(math::Real{-1});

    const math::Vector3 xAxis = math::Vector3::UnitX();
    const math::Vector3 yAxis = math::Vector3::UnitY();
    const math::Vector3 zAxis = math::Vector3::UnitZ();

    // Axis-angle construction: +90 degrees about Z maps +X to +Y.
    const math::Quaternion rotateZ90 = math::quaternionFromAxisAngle(
        pi / math::Real{2},
        zAxis
    );

    if (!nearlyEqual(math::quaternionNorm(rotateZ90), math::kOne)) {
        std::cerr << "Axis-angle unit-norm test failed.\n";
        return 1;
    }

    const math::Matrix3 rotationZ90 =
        math::quaternionToRotationMatrix(rotateZ90);
    if (!(rotationZ90 * xAxis).isApprox(yAxis, tolerance)) {
        std::cerr << "Quaternion vector-rotation test failed.\n";
        return 1;
    }

    // A valid quaternion rotation matrix must be orthogonal with det(R) = 1.
    if (!(rotationZ90.transpose() * rotationZ90)
            .isApprox(math::Matrix3::Identity(), tolerance)) {
        std::cerr << "Quaternion rotation-matrix orthogonality test failed.\n";
        return 1;
    }

    if (!nearlyEqual(rotationZ90.determinant(), math::kOne)) {
        std::cerr << "Quaternion rotation-matrix determinant test failed.\n";
        return 1;
    }

    // Multiplication order: qSecond * qFirst means first qFirst, then qSecond.
    const math::Quaternion rotateX90 = math::quaternionFromAxisAngle(
        pi / math::Real{2},
        xAxis
    );
    const math::Quaternion combined = math::multiplyQuaternions(
        rotateZ90,
        rotateX90
    );
    const math::Matrix3 expectedCombined =
        math::quaternionToRotationMatrix(rotateZ90)
        * math::quaternionToRotationMatrix(rotateX90);

    if (!math::quaternionToRotationMatrix(combined)
            .isApprox(expectedCombined, tolerance)) {
        std::cerr << "Quaternion multiplication-order test failed.\n";
        return 1;
    }

    // Normalization must produce a unit quaternion without changing rotation.
    const math::Quaternion unnormalized(
        math::Real{2},
        math::Real{-1},
        math::Real{3},
        math::Real{0.5}
    );
    const math::Quaternion normalized =
        math::normalizedQuaternion(unnormalized);
    if (!nearlyEqual(math::quaternionNorm(normalized), math::kOne)) {
        std::cerr << "Quaternion normalization test failed.\n";
        return 1;
    }

    // q * conjugate(q) = (|q|^2, 0, 0, 0).
    const math::Quaternion conjugateProduct = math::multiplyQuaternions(
        unnormalized,
        math::conjugateQuaternion(unnormalized)
    );
    if (!nearlyEqual(
            conjugateProduct.w(),
            math::quaternionNormSquared(unnormalized))
        || !nearlyEqual(conjugateProduct.x(), math::kZero)
        || !nearlyEqual(conjugateProduct.y(), math::kZero)
        || !nearlyEqual(conjugateProduct.z(), math::kZero)) {
        std::cerr << "Quaternion conjugate test failed.\n";
        return 1;
    }

    // q * inverse(q) = identity, including for a non-unit quaternion.
    const math::Quaternion inverseProduct = math::multiplyQuaternions(
        unnormalized,
        math::inverseQuaternion(unnormalized)
    );
    if (!isIdentityQuaternion(inverseProduct)) {
        std::cerr << "Quaternion inverse test failed.\n";
        return 1;
    }

    // q and -q represent the same rotation.
    const math::Quaternion negativeRotateZ90(
        -rotateZ90.w(),
        -rotateZ90.x(),
        -rotateZ90.y(),
        -rotateZ90.z()
    );
    if (!math::quaternionToRotationMatrix(negativeRotateZ90)
            .isApprox(rotationZ90, tolerance)) {
        std::cerr << "Quaternion sign-equivalence test failed.\n";
        return 1;
    }

    // The axis-angle API must normalize a non-unit axis.
    const math::Quaternion rotateAroundScaledAxis =
        math::quaternionFromAxisAngle(
            pi / math::Real{2},
            math::Real{3} * zAxis
        );
    if (!math::quaternionToRotationMatrix(rotateAroundScaledAxis)
            .isApprox(rotationZ90, tolerance)) {
        std::cerr << "Non-unit rotation-axis test failed.\n";
        return 1;
    }

    // Matrix conversion must be safe even if numerical drift scales q.
    const math::Quaternion scaledQuaternion(
        math::Real{4} * rotateZ90.w(),
        math::Real{4} * rotateZ90.x(),
        math::Real{4} * rotateZ90.y(),
        math::Real{4} * rotateZ90.z()
    );
    if (!math::quaternionToRotationMatrix(scaledQuaternion)
            .isApprox(rotationZ90, tolerance)) {
        std::cerr << "Scaled-quaternion matrix-conversion test failed.\n";
        return 1;
    }

    bool rejectedZeroAxis = false;
    try {
        static_cast<void>(math::quaternionFromAxisAngle(
            math::kOne,
            math::Vector3::Zero()
        ));
    } catch (const std::invalid_argument&) {
        rejectedZeroAxis = true;
    }

    if (!rejectedZeroAxis) {
        std::cerr << "Zero rotation-axis validation test failed.\n";
        return 1;
    }

    bool rejectedZeroNormalization = false;
    try {
        static_cast<void>(math::normalizedQuaternion(
            math::Quaternion(math::kZero, math::kZero, math::kZero, math::kZero)
        ));
    } catch (const std::invalid_argument&) {
        rejectedZeroNormalization = true;
    }

    if (!rejectedZeroNormalization) {
        std::cerr << "Zero-quaternion normalization validation test failed.\n";
        return 1;
    }

    bool rejectedZeroInverse = false;
    try {
        static_cast<void>(math::inverseQuaternion(
            math::Quaternion(math::kZero, math::kZero, math::kZero, math::kZero)
        ));
    } catch (const std::invalid_argument&) {
        rejectedZeroInverse = true;
    }

    if (!rejectedZeroInverse) {
        std::cerr << "Zero-quaternion inverse validation test failed.\n";
        return 1;
    }

    std::cout << "GyPhysics QuaternionOperators test passed with Real="
              << math::realTypeName() << '\n';
    return 0;
}