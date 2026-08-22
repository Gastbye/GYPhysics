#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "gy/physics/geometry/ShapeRegistry.h"
#include "gy/physics/geometry/SphereShape.h"
#include "gy/physics/geometry/TriMeshShape.h"

namespace {

namespace geometry = gy::physics::geometry;
namespace math = gy::physics::math;

[[nodiscard]] constexpr math::Real testTolerance() noexcept
{
#if GY_PHYSICS_REAL_IS_DOUBLE == 1
    return math::Real{1.0e-12};
#else
    return math::Real{1.0e-5F};
#endif
}

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(
    math::Real actual,
    math::Real expected,
    const std::string& message)
{
    const math::Real scale = std::max(math::Real{1}, std::abs(expected));
    if (std::abs(actual - expected) > testTolerance() * scale) {
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

[[nodiscard]] math::Vector3 vertex(
    math::Real x,
    math::Real y,
    math::Real z)
{
    return math::Vector3(x, y, z);
}

[[nodiscard]] math::IntV3 triangle(
    math::Index i0,
    math::Index i1,
    math::Index i2)
{
    return math::IntV3(i0, i1, i2);
}

[[nodiscard]] geometry::TriMeshData::VertexContainer tetrahedronVertices()
{
    return {
        vertex(math::Real{0}, math::Real{0}, math::Real{0}),
        vertex(math::Real{1}, math::Real{0}, math::Real{0}),
        vertex(math::Real{0}, math::Real{1}, math::Real{0}),
        vertex(math::Real{0}, math::Real{0}, math::Real{1})
    };
}

[[nodiscard]] geometry::TriMeshData::TriangleContainer tetrahedronTriangles()
{
    return {
        triangle(1, 2, 3),
        triangle(0, 2, 1),
        triangle(0, 1, 3),
        triangle(0, 3, 2)
    };
}

void testDefaultStateAndInvalidAccess()
{
    static_assert(noexcept(
        std::declval<const geometry::ShapeRegistry&>().contains(
            geometry::ShapeId{}
        )
    ));
    static_assert(std::is_same_v<
        decltype(std::declval<const geometry::ShapeRegistry&>().get(
            geometry::ShapeId{}
        )),
        const geometry::Shape&
    >);
    static_assert(!std::is_convertible_v<math::Index, geometry::ShapeId>);

    geometry::ShapeRegistry registry;
    const geometry::ShapeId invalidId;
    const geometry::ShapeId outOfRangeId{math::Index{1234}};

    require(!invalidId.isValid(), "A default ShapeId must be invalid.");
    require(invalidId.value == math::InvalidIndex,
            "A default ShapeId must use InvalidIndex.");
    require(registry.empty(), "A new registry must be empty.");
    require(registry.size() == std::size_t{0},
            "A new registry size must be zero.");
    require(!registry.contains(invalidId),
            "A registry cannot contain the default ShapeId.");
    require(!registry.contains(outOfRangeId),
            "A registry cannot contain an out-of-range ShapeId.");
    requireThrows<std::out_of_range>(
        [&registry, invalidId] {
            static_cast<void>(registry.get(invalidId));
        },
        "Getting the default ShapeId must throw."
    );
    requireThrows<std::out_of_range>(
        [&registry, outOfRangeId] {
            static_cast<void>(registry.get(outOfRangeId));
        },
        "Getting an out-of-range ShapeId must throw."
    );
    requireThrows<std::out_of_range>(
        [&registry, invalidId] { registry.remove(invalidId); },
        "Removing the default ShapeId must throw."
    );
    requireThrows<std::out_of_range>(
        [&registry, outOfRangeId] { registry.remove(outOfRangeId); },
        "Removing an out-of-range ShapeId must throw."
    );
}

void testSphereOwnershipAndConstLookup()
{
    geometry::ShapeRegistry registry;
    auto sphere = std::make_unique<geometry::SphereShape>(math::Real{2});
    const geometry::ShapeId sphereId = registry.add(std::move(sphere));

    require(sphere == nullptr, "ShapeRegistry must take unique ownership.");
    require(sphereId.isValid(), "A registered ShapeId must be valid.");
    require(registry.contains(sphereId),
            "Registry must contain the registered sphere.");
    require(registry.size() == std::size_t{1},
            "Adding a sphere must increase the active size.");
    require(!registry.empty(), "A populated registry cannot be empty.");

    const geometry::ShapeRegistry& constRegistry = registry;
    const geometry::Shape& baseShape = constRegistry.get(sphereId);
    require(baseShape.type() == geometry::ShapeType::Sphere,
            "Registered sphere has the wrong ShapeType.");
    const auto* storedSphere =
        dynamic_cast<const geometry::SphereShape*>(&baseShape);
    require(storedSphere != nullptr,
            "ShapeRegistry sliced the registered SphereShape.");
    requireNear(storedSphere->radius(), math::Real{2},
                "Registered sphere radius changed.");
    requireNear(
        storedSphere->volume(),
        math::Real{32} * std::acos(math::Real{-1}) / math::Real{3},
        "Registered sphere volume changed."
    );
}

void testTriMeshShapeAndInputPreservation()
{
    geometry::ShapeRegistry registry;
    const geometry::ShapeId sphereId = registry.add(
        std::make_unique<geometry::SphereShape>(math::Real{1})
    );
    const auto inputVertices = tetrahedronVertices();
    const auto inputTriangles = tetrahedronTriangles();
    const std::shared_ptr<const geometry::TriMeshData> meshData =
        std::make_shared<geometry::TriMeshData>(
            inputVertices, inputTriangles);
    auto meshShape = std::make_unique<geometry::TriMeshShape>(meshData);
    const geometry::ShapeId meshId = registry.add(std::move(meshShape));

    require(meshShape == nullptr, "Registry must own the TriMeshShape.");
    require(meshId.isValid(), "TriMeshShape needs a valid ShapeId.");
    require(meshId != sphereId,
            "Different registered shapes must receive different IDs.");
    const geometry::Shape& baseShape = registry.get(meshId);
    require(baseShape.type() == geometry::ShapeType::TriangleMesh,
            "Registered mesh has the wrong ShapeType.");
    const auto* storedMesh =
        dynamic_cast<const geometry::TriMeshShape*>(&baseShape);
    require(storedMesh != nullptr,
            "ShapeRegistry sliced the registered TriMeshShape.");
    require(storedMesh->dataPointer().get() == meshData.get(),
            "TriMeshShape no longer references the original mesh data.");
    require(storedMesh->data().vertexCount() == inputVertices.size(),
            "Registered mesh vertex count changed.");
    requireNear(storedMesh->volume(), math::Real{1} / math::Real{6},
                "Registered tetrahedron volume changed.");

    for (std::size_t index = 0; index < inputVertices.size(); ++index) {
        require((storedMesh->data().vertices()[index].array()
                 == inputVertices[index].array()).all(),
                "Registering a TriMeshShape modified its vertices.");
    }
}

void testNullShapeIsRejected()
{
    geometry::ShapeRegistry registry;
    requireThrows<std::invalid_argument>(
        [&registry] { static_cast<void>(registry.add(nullptr)); },
        "Adding a null shape must throw invalid_argument."
    );
    require(registry.empty(), "A rejected null shape changed empty().");
    require(registry.size() == std::size_t{0},
            "A rejected null shape changed size().");
}

void testIdsSurviveGrowth()
{
    geometry::ShapeRegistry registry;
    const geometry::ShapeId firstId = registry.add(
        std::make_unique<geometry::SphereShape>(math::Real{0.5})
    );

    constexpr std::size_t additionalShapeCount = 4096;
    for (std::size_t index = 0;
         index < additionalShapeCount;
         ++index) {
        static_cast<void>(registry.add(
            std::make_unique<geometry::SphereShape>(math::Real{2})
        ));
    }

    const auto* firstSphere = dynamic_cast<const geometry::SphereShape*>(
        &registry.get(firstId));
    require(firstSphere != nullptr,
            "Registry growth changed the first shape's dynamic type.");
    requireNear(firstSphere->radius(), math::Real{0.5},
                "Registry growth changed the first ShapeId meaning.");
    require(registry.size() == additionalShapeCount + std::size_t{1},
            "Registry growth produced the wrong active size.");
}

void testRemovalKeepsIdsStableAndSlotsAreNotReused()
{
    geometry::ShapeRegistry registry;
    const geometry::ShapeId firstId = registry.add(
        std::make_unique<geometry::SphereShape>(math::Real{1})
    );
    const geometry::ShapeId removedId = registry.add(
        std::make_unique<geometry::SphereShape>(math::Real{2})
    );
    const geometry::ShapeId lastId = registry.add(
        std::make_unique<geometry::SphereShape>(math::Real{3})
    );
    const math::Index lastIdValue = lastId.value;

    registry.remove(removedId);
    require(registry.size() == std::size_t{2},
            "Removing a shape must reduce the active size.");
    require(!registry.contains(removedId),
            "A removed ShapeId must no longer be contained.");
    require(registry.contains(firstId), "Removing B invalidated A.");
    require(registry.contains(lastId), "Removing B invalidated C.");
    require(lastId.value == lastIdValue,
            "Removing B changed C's numeric ShapeId.");

    const auto* firstSphere = dynamic_cast<const geometry::SphereShape*>(
        &registry.get(firstId));
    const auto* lastSphere = dynamic_cast<const geometry::SphereShape*>(
        &registry.get(lastId));
    require(firstSphere != nullptr && firstSphere->radius() == math::Real{1},
            "A no longer resolves to the original shape.");
    require(lastSphere != nullptr && lastSphere->radius() == math::Real{3},
            "C no longer resolves to the original shape.");
    requireThrows<std::out_of_range>(
        [&registry, removedId] {
            static_cast<void>(registry.get(removedId));
        },
        "Getting a removed ShapeId must throw."
    );
    requireThrows<std::out_of_range>(
        [&registry, removedId] { registry.remove(removedId); },
        "Repeated removal must throw without changing size."
    );
    require(registry.size() == std::size_t{2},
            "Repeated removal changed the active size.");

    const geometry::ShapeId newId = registry.add(
        std::make_unique<geometry::SphereShape>(math::Real{4})
    );
    require(newId != removedId, "A removed slot must not be reused.");
    require(newId.value > lastId.value,
            "New shapes must be appended after all issued slots.");
    require(!registry.contains(removedId),
            "An old ShapeId became valid again after another addition.");
    require(registry.contains(newId), "The appended shape is unavailable.");
    require(registry.size() == std::size_t{3},
            "Appending after removal produced the wrong active size.");
}

} // namespace

int main()
{
    try {
        testDefaultStateAndInvalidAccess();
        testSphereOwnershipAndConstLookup();
        testTriMeshShapeAndInputPreservation();
        testNullShapeIsRejected();
        testIdsSurviveGrowth();
        testRemovalKeepsIdsStableAndSlotsAreNotReused();
    } catch (const std::exception& error) {
        std::cerr << "ShapeRegistry test failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "GyPhysics ShapeRegistry tests passed with Real="
              << math::realTypeName() << '\n';
    return 0;
}
