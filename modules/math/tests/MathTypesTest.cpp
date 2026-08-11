#include <cmath>
#include <iostream>
#include <type_traits>

#include "gy/physics/math/MathTypes.h"

namespace {

bool nearlyEqual(
    gy::physics::math::Real lhs,
    gy::physics::math::Real rhs,
    gy::physics::math::Real tolerance)
{
    return std::abs(lhs - rhs) <= tolerance;
}

} // namespace

int main()
{
    namespace math = gy::physics::math;

#if GY_PHYSICS_REAL_IS_DOUBLE == 1
    static_assert(std::is_same_v<math::Real, double>);
#else
    static_assert(std::is_same_v<math::Real, float>);
#endif

    const math::Vector3 x = math::Vector3::UnitX();
    const math::Vector3 y = math::Vector3::UnitY();
    const math::Vector3 z = x.cross(y);

    const math::Real tolerance = static_cast<math::Real>(100) * math::kEpsilon;
    if (!z.isApprox(math::Vector3::UnitZ(), tolerance)) {
        std::cerr << "Vector3 cross-product test failed.\n";
        return 1;
    }

    const math::Matrix3 identity = math::Matrix3::Identity();
    const math::Vector3 value(
        static_cast<math::Real>(1),
        static_cast<math::Real>(2),
        static_cast<math::Real>(3)
    );
    if (!(identity * value).isApprox(value, tolerance)) {
        std::cerr << "Matrix3 identity test failed.\n";
        return 1;
    }

    const math::Quaternion rotation = math::Quaternion::Identity();
    if (!(rotation * value).isApprox(value, tolerance)) {
        std::cerr << "Quaternion identity test failed.\n";
        return 1;
    }

    if (!nearlyEqual(
            value.norm(),
            std::sqrt(static_cast<math::Real>(14)),
            tolerance)) {
        std::cerr << "Vector3 norm test failed.\n";
        return 1;
    }

    std::cout << "GyPhysics Math test passed with Real="
              << math::realTypeName() << '\n';
    return 0;
}
