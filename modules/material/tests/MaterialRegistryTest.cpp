#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "gy/physics/material/MaterialRegistry.h"

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

[[nodiscard]] material::Material makeMaterial(
    std::string name,
    math::Real density)
{
    material::Material result(std::move(name));
    result.setDensity(density);
    return result;
}

void testAddAndLookup()
{
    static_assert(noexcept(
        std::declval<const material::MaterialRegistry&>().contains(
            material::MaterialId{}
        )
    ));

    material::MaterialRegistry registry;
    require(registry.empty(), "A new registry must be empty.");
    require(registry.size() == std::size_t{0}, "Initial size must be zero.");

    const material::MaterialId steelId = registry.add(
        makeMaterial("steel", math::Real{7850})
    );
    const material::MaterialId aluminumId = registry.add(
        makeMaterial("aluminum", math::Real{2700})
    );

    require(steelId.isValid(), "A registered ID must be valid.");
    require(aluminumId.isValid(), "A registered ID must be valid.");
    require(steelId != aluminumId, "Registered IDs must be unique.");
    require(steelId == steelId, "An ID must compare equal to itself.");
    require(!registry.empty(), "A populated registry must not be empty.");
    require(registry.size() == std::size_t{2}, "Registry size is incorrect.");
    require(registry.contains(steelId), "Registry must contain steel ID.");
    require(registry.get(steelId).name() == "steel", "Wrong material name.");
    require(
        registry.get(steelId).density() == math::Real{7850},
        "Wrong material density."
    );

    const material::MaterialRegistry& constRegistry = registry;
    require(
        constRegistry.get(aluminumId).name() == "aluminum",
        "A const registry must support lookup."
    );
}

void testInvalidLookup()
{
    material::MaterialRegistry registry;
    const material::MaterialId invalid;
    const material::MaterialId outOfRange{math::Index{1234}};

    requireThrows<std::invalid_argument>(
        [&registry] {
            static_cast<void>(
                registry.add(material::Material("density-not-set"))
            );
        },
        "Registering a material without density must throw invalid_argument."
    );

    require(!registry.contains(invalid), "Default ID must not be contained.");
    require(!registry.contains(outOfRange), "Out-of-range ID must not exist.");
    requireThrows<std::out_of_range>(
        [&registry, invalid] { static_cast<void>(registry.get(invalid)); },
        "Getting a default ID must throw out_of_range."
    );
    requireThrows<std::out_of_range>(
        [&registry, outOfRange] {
            static_cast<void>(registry.get(outOfRange));
        },
        "Getting an out-of-range ID must throw out_of_range."
    );
}

void testIdsSurviveGrowth()
{
    material::MaterialRegistry registry;
    const material::MaterialId firstId = registry.add(
        makeMaterial("first", math::Real{1})
    );

    constexpr std::size_t materialCount = 4096;
    for (std::size_t index = 0; index < materialCount; ++index) {
        static_cast<void>(registry.add(makeMaterial(
            "bulk-" + std::to_string(index),
            math::Real{2}
        )));
    }

    require(
        registry.get(firstId).name() == "first",
        "Registry growth changed the meaning of an existing ID."
    );
    require(
        registry.get(firstId).density() == math::Real{1},
        "Registry growth changed an existing material."
    );
}

void testRemovalKeepsIdsStable()
{
    material::MaterialRegistry registry;
    const material::MaterialId firstId = registry.add(
        makeMaterial("first", math::Real{1})
    );
    const material::MaterialId removedId = registry.add(
        makeMaterial("removed", math::Real{2})
    );
    const material::MaterialId lastId = registry.add(
        makeMaterial("last", math::Real{3})
    );
    const math::Index lastIdValue = lastId.value;

    registry.remove(removedId);
    require(registry.size() == std::size_t{2}, "Removal must reduce size.");
    require(!registry.contains(removedId), "Removed ID must not be contained.");
    require(registry.get(firstId).name() == "first", "First ID changed.");
    require(registry.get(lastId).name() == "last", "Last ID changed.");
    require(
        lastId.value == lastIdValue,
        "Removing another material changed the numeric value of the last ID."
    );
    requireThrows<std::out_of_range>(
        [&registry, removedId] {
            static_cast<void>(registry.get(removedId));
        },
        "A removed ID must not be accessible."
    );
    requireThrows<std::out_of_range>(
        [&registry, removedId] { registry.remove(removedId); },
        "Repeated removal must throw out_of_range."
    );

    const material::MaterialId newId = registry.add(
        makeMaterial("new", math::Real{4})
    );
    require(newId.isValid(), "A material added after removal needs a valid ID.");
    require(newId != removedId, "Removed slots must not be reused.");
    require(
        newId.value > lastId.value,
        "A new material must be appended after all existing slots."
    );
    require(!registry.contains(removedId), "An old ID must stay invalid.");
    require(registry.get(newId).name() == "new", "New ID lookup failed.");
}

} // namespace

int main()
{
    try {
        testAddAndLookup();
        testInvalidLookup();
        testIdsSurviveGrowth();
        testRemovalKeepsIdsStable();
    } catch (const std::exception& error) {
        std::cerr << "MaterialRegistry test failed: "
                  << error.what() << '\n';
        return 1;
    }

    std::cout << "GyPhysics MaterialRegistry tests passed with Real="
              << math::realTypeName() << '\n';
    return 0;
}
