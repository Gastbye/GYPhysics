/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "gy/physics/math/MathOperators.h"

namespace {

namespace math = gy::physics::math;

[[nodiscard]] constexpr math::Real testTolerance() noexcept
{
#if GY_PHYSICS_REAL_IS_DOUBLE == 1
    return static_cast<math::Real>(1.0e-12);
#else
    return static_cast<math::Real>(1.0e-5);
#endif
}

[[nodiscard]] bool nearlyEqual(
    math::Real lhs,
    math::Real rhs,
    math::Real tolerance = testTolerance()) noexcept
{
    return std::abs(lhs - rhs) <= tolerance;
}

int reportFailure(const char* message)
{
    std::cerr << "[FAILED] " << message << '\n';
    return 1;
}

} // namespace

int main()
{
    const math::Real tolerance = testTolerance();

    const math::Vector3 x = math::Vector3::UnitX();
    const math::Vector3 y = math::Vector3::UnitY();
    const math::Vector3 z = math::Vector3::UnitZ();

    /*
     * Test 1: 点积
     *
     * e_x · e_y = 0
     * e_x · e_x = 1
     */
    if (!nearlyEqual(math::dot(x, y), math::kZero)) {
        return reportFailure(
            "Dot product of orthogonal vectors should be zero."
        );
    }

    if (!nearlyEqual(math::dot(x, x), math::kOne)) {
        return reportFailure(
            "Dot product of a unit vector with itself should be one."
        );
    }

    /*
     * Test 2: 叉积方向
     *
     * e_x × e_y = e_z
     * e_y × e_x = -e_z
     */
    if (!math::cross(x, y).isApprox(z, tolerance)) {
        return reportFailure(
            "Cross product x cross y should be z."
        );
    }

    if (!math::cross(y, x).isApprox(-z, tolerance)) {
        return reportFailure(
            "Cross product y cross x should be negative z."
        );
    }

    /*
     * Test 3: 向量长度
     */
    const math::Vector3 vector(
        static_cast<math::Real>(2),
        static_cast<math::Real>(-3),
        static_cast<math::Real>(6)
    );

    // 2^2 + (-3)^2 + 6^2 = 49
    if (!nearlyEqual(
            math::lengthSquared(vector),
            static_cast<math::Real>(49))) {
        return reportFailure(
            "Squared vector length should be 49."
        );
    }

    if (!nearlyEqual(
            math::length(vector),
            static_cast<math::Real>(7))) {
        return reportFailure(
            "Vector length should be 7."
        );
    }

    /*
     * Test 4: 向量归一化
     */
    const math::Vector3 unitVector = math::normalized(vector);

    if (!nearlyEqual(math::length(unitVector), math::kOne)) {
        return reportFailure(
            "Normalized vector length should be one."
        );
    }

    /*
     * Test 5: 零向量不能归一化
     */
    bool zeroVectorRejected = false;

    try {
        static_cast<void>(
            math::normalized(math::Vector3::Zero())
        );
    } catch (const std::invalid_argument&) {
        zeroVectorRejected = true;
    }

    if (!zeroVectorRejected) {
        return reportFailure(
            "Normalizing a zero vector should throw invalid_argument."
        );
    }

    /*
     * Test 6: 构造旋转矩阵
     *
     * 绕世界坐标系 z 轴旋转 90 度：
     *
     *     e_x -> e_y
     */
    const math::Real pi =
        std::acos(static_cast<math::Real>(-1));

    const math::Real halfPi =
        pi / static_cast<math::Real>(2);

    const math::Matrix3 rotation =
        math::rotationMatrixFromAxisAngle(z, halfPi);

    const math::Vector3 rotatedX =
        math::localToWorldVector(x, rotation);

    if (!rotatedX.isApprox(y, tolerance)) {
        return reportFailure(
            "A 90-degree rotation around z should map x to y."
        );
    }

    /*
     * Test 7: 旋转矩阵的正交性
     *
     *     R^T R = I
     */
    const math::Matrix3 rotationTranspose =
        math::transpose(rotation);

    const math::Matrix3 orthogonality =
        math::multiply(rotationTranspose, rotation);

    if (!orthogonality.isApprox(
            math::Matrix3::Identity(),
            tolerance)) {
        return reportFailure(
            "Rotation matrix should satisfy transpose(R) * R = I."
        );
    }

    /*
     * Test 8: 矩阵乘向量接口
     */
    const math::Vector3 multipliedVector =
        math::multiply(rotation, x);

    if (!multipliedVector.isApprox(y, tolerance)) {
        return reportFailure(
            "Matrix-vector multiplication test failed."
        );
    }

    /*
     * Test 9: 向量局部坐标 -> 世界坐标 -> 局部坐标
     */
    const math::Vector3 localVector(
        static_cast<math::Real>(2),
        static_cast<math::Real>(1),
        static_cast<math::Real>(-3)
    );

    const math::Vector3 worldVector =
        math::localToWorldVector(
            localVector,
            rotation
        );

    const math::Vector3 recoveredLocalVector =
        math::worldToLocalVector(
            worldVector,
            rotation
        );

    if (!recoveredLocalVector.isApprox(
            localVector,
            tolerance)) {
        return reportFailure(
            "Vector local-world-local round-trip failed."
        );
    }

    /*
     * Test 10: 旋转不改变向量长度
     *
     *     ||R v|| = ||v||
     */
    if (!nearlyEqual(
            math::length(localVector),
            math::length(worldVector))) {
        return reportFailure(
            "Rotation should preserve vector length."
        );
    }

    /*
     * Test 11: 旋转不改变两个向量的点积
     *
     *     (R a) · (R b) = a · b
     */
    const math::Vector3 localA(
        static_cast<math::Real>(1),
        static_cast<math::Real>(2),
        static_cast<math::Real>(3)
    );

    const math::Vector3 localB(
        static_cast<math::Real>(-2),
        static_cast<math::Real>(4),
        static_cast<math::Real>(1)
    );

    const math::Vector3 worldA =
        math::localToWorldVector(localA, rotation);

    const math::Vector3 worldB =
        math::localToWorldVector(localB, rotation);

    if (!nearlyEqual(
            math::dot(localA, localB),
            math::dot(worldA, worldB))) {
        return reportFailure(
            "Rotation should preserve dot products."
        );
    }

    /*
     * Test 12: 局部点转换到世界坐标
     *
     * 局部点：
     *
     *     p_local = (2, 1, -3)
     *
     * 绕 z 轴旋转90度：
     *
     *     R p_local = (-1, 2, -3)
     *
     * 局部坐标系原点在世界坐标：
     *
     *     origin_world = (10, -2, 1)
     *
     * 所以：
     *
     *     p_world = (9, 0, -2)
     */
    const math::Vector3 localOriginInWorld(
        static_cast<math::Real>(10),
        static_cast<math::Real>(-2),
        static_cast<math::Real>(1)
    );

    const math::Vector3 worldPoint =
        math::localToWorldPoint(
            localVector,
            localOriginInWorld,
            rotation
        );

    const math::Vector3 expectedWorldPoint(
        static_cast<math::Real>(9),
        static_cast<math::Real>(0),
        static_cast<math::Real>(-2)
    );

    if (!worldPoint.isApprox(
            expectedWorldPoint,
            tolerance)) {
        return reportFailure(
            "Known local-to-world point transformation failed."
        );
    }

    /*
     * Test 13: 点的局部 -> 世界 -> 局部往返变换
     */
    const math::Vector3 recoveredLocalPoint =
        math::worldToLocalPoint(
            worldPoint,
            localOriginInWorld,
            rotation
        );

    if (!recoveredLocalPoint.isApprox(
            localVector,
            tolerance)) {
        return reportFailure(
            "Point local-world-local round-trip failed."
        );
    }

    /*
     * Test 14: 零长度旋转轴必须报错
     */
    bool zeroAxisRejected = false;

    try {
        static_cast<void>(
            math::rotationMatrixFromAxisAngle(
                math::Vector3::Zero(),
                halfPi
            )
        );
    } catch (const std::invalid_argument&) {
        zeroAxisRejected = true;
    }

    if (!zeroAxisRejected) {
        return reportFailure(
            "A zero-length rotation axis should be rejected."
        );
    }

    std::cout
        << "GyPhysics MathOperators tests passed with Real="
        << math::realTypeName()
        << '\n';

    return 0;
}