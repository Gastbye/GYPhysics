#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "gy/physics/material/Material.h"
#include "gy/physics/material/MaterialStruct.h"

namespace {

namespace material = gy::physics::material;
namespace math = gy::physics::math;

void require(bool condition, const std::string& message)
{
    if (!condition) {
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

void testMaterialId()
{
    static_assert(!std::is_convertible_v<math::Index, material::MaterialId>);
    static_assert(!std::is_convertible_v<material::MaterialId, math::Index>);

    const material::MaterialId invalid;
    const material::MaterialId first{math::Index{0}};
    const material::MaterialId sameFirst{math::Index{0}};
    const material::MaterialId second{math::Index{1}};

    require(!invalid.isValid(), "A default MaterialId must be invalid.");
    require(first.isValid(), "A non-sentinel MaterialId must be valid.");
    require(first == sameFirst, "Equal MaterialIds must compare equal.");
    require(first != second, "Different MaterialIds must compare unequal.");
}

void testValidMaterial()
{
    static_assert(std::is_same_v<
        decltype(std::declval<const material::Material&>().density()),
        math::Real
    >);
    static_assert(noexcept(
        std::declval<const material::Material&>().name()
    ));
    static_assert(noexcept(
        std::declval<const material::Material&>().density()
    ));

    material::Material steel("steel");
    require(steel.name() == "steel", "Material name getter is incorrect.");
    require(
        steel.density() == math::kZero,
        "A new material density must initially be unset."
    );

    steel.setDensity(math::Real{7850});
    require(
        steel.density() == math::Real{7850},
        "Material density getter is incorrect."
    );
}

void testInvalidDensities()
{
    const auto requireInvalidDensity = [](math::Real density) {
        material::Material invalid("invalid");
        requireThrows<std::invalid_argument>(
            [&invalid, density] { invalid.setDensity(density); },
            "Invalid material density must be rejected."
        );
    };

    requireInvalidDensity(math::Real{0});
    requireInvalidDensity(math::Real{-1});
    requireInvalidDensity(std::numeric_limits<math::Real>::quiet_NaN());
    requireInvalidDensity(std::numeric_limits<math::Real>::infinity());
    requireInvalidDensity(-std::numeric_limits<math::Real>::infinity());

    material::Material materialWithDensity("valid");
    materialWithDensity.setDensity(math::Real{10});
    requireThrows<std::invalid_argument>(
        [&materialWithDensity] {
            materialWithDensity.setDensity(math::Real{0});
        },
        "An invalid update must be rejected."
    );
    require(
        materialWithDensity.density() == math::Real{10},
        "A rejected density update must preserve the previous value."
    );
}

} // namespace

int main()
{
    try {
        testMaterialId();
        testValidMaterial();
        testInvalidDensities();
    } catch (const std::exception& error) {
        std::cerr << "Material test failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "GyPhysics Material tests passed with Real="
              << math::realTypeName() << '\n';
    return 0;
}
