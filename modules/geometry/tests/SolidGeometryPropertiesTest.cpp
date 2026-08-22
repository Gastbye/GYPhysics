#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "gy/physics/geometry/TriMeshData.h"

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

void requireVectorNear(
    const math::Vector3& actual,
    const math::Vector3& expected,
    const std::string& message)
{
    const math::Real scale = std::max(
        math::Real{1}, expected.cwiseAbs().maxCoeff());
    if ((actual - expected).cwiseAbs().maxCoeff()
        > testTolerance() * scale) {
        throw std::runtime_error(message);
    }
}

void requireMatrixNear(
    const math::Matrix3& actual,
    const math::Matrix3& expected,
    const std::string& message)
{
    const math::Real scale = std::max(
        math::Real{1}, expected.cwiseAbs().maxCoeff());
    if ((actual - expected).cwiseAbs().maxCoeff()
        > testTolerance() * scale) {
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

[[nodiscard]] geometry::TriMeshData::VertexContainer tetrahedronVertices(
    const math::Vector3& translation = math::Vector3::Zero())
{
    return {
        translation + vertex(math::Real{0}, math::Real{0}, math::Real{0}),
        translation + vertex(math::Real{1}, math::Real{0}, math::Real{0}),
        translation + vertex(math::Real{0}, math::Real{1}, math::Real{0}),
        translation + vertex(math::Real{0}, math::Real{0}, math::Real{1})
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

[[nodiscard]] math::Matrix3 expectedTetrahedronInertia()
{
    const math::Real diagonal = math::Real{1} / math::Real{80};
    const math::Real offDiagonal = math::Real{1} / math::Real{480};
    math::Matrix3 expected;
    expected << diagonal, offDiagonal, offDiagonal,
                offDiagonal, diagonal, offDiagonal,
                offDiagonal, offDiagonal, diagonal;
    return expected;
}

void testUnitTetrahedron()
{
    const geometry::TriMeshData mesh(
        tetrahedronVertices(), tetrahedronTriangles());
    require(mesh.hasSolidGeometryProperties(),
            "A unit tetrahedron must have solid geometry properties.");
    require(mesh.hasVolume(), "A unit tetrahedron must have volume.");
    requireNear(mesh.volume(), math::Real{1} / math::Real{6},
                "Unit tetrahedron volume is incorrect.");
    requireVectorNear(
        mesh.centroid(),
        vertex(math::Real{1} / math::Real{4},
               math::Real{1} / math::Real{4},
               math::Real{1} / math::Real{4}),
        "Unit tetrahedron centroid is incorrect."
    );
    requireMatrixNear(
        mesh.unitDensityInertiaAtCentroid(),
        expectedTetrahedronInertia(),
        "Unit tetrahedron centroidal inertia is incorrect."
    );
    requireMatrixNear(
        mesh.unitDensityInertiaAtCentroid(),
        mesh.unitDensityInertiaAtCentroid().transpose(),
        "Unit tetrahedron inertia must be symmetric."
    );
    require(&mesh.solidGeometryProperties().centroid == &mesh.centroid(),
            "TriMeshData getters must expose the authoritative cache.");
}

void testTranslatedTetrahedronAndInputPreservation()
{
    const math::Vector3 translation =
        vertex(math::Real{10}, math::Real{-3}, math::Real{5});
    const auto inputVertices = tetrahedronVertices(translation);
    const auto inputTriangles = tetrahedronTriangles();
    const geometry::TriMeshData mesh(inputVertices, inputTriangles);

    requireNear(mesh.volume(), math::Real{1} / math::Real{6},
                "Translation changed tetrahedron volume.");
    requireVectorNear(
        mesh.centroid(),
        translation + math::Vector3::Constant(math::Real{1} / math::Real{4}),
        "Translated tetrahedron centroid is incorrect."
    );
    requireMatrixNear(
        mesh.unitDensityInertiaAtCentroid(),
        expectedTetrahedronInertia(),
        "Translation changed centroidal inertia."
    );

    require(mesh.vertices().size() == inputVertices.size(),
            "TriMeshData changed the vertex count.");
    for (std::size_t index = 0; index < inputVertices.size(); ++index) {
        require((mesh.vertices()[index].array()
                 == inputVertices[index].array()).all(),
                "Geometry-property integration modified an input vertex.");
    }
    require(mesh.triangles().size() == inputTriangles.size(),
            "TriMeshData changed the triangle count.");
    for (std::size_t index = 0; index < inputTriangles.size(); ++index) {
        require((mesh.triangles()[index].array()
                 == inputTriangles[index].array()).all(),
                "Geometry-property integration modified triangle winding.");
    }
}

void testReversedTetrahedron()
{
    auto reversedTriangles = tetrahedronTriangles();
    for (math::IntV3& face : reversedTriangles) {
        std::swap(face[1], face[2]);
    }
    const geometry::TriMeshData mesh(
        tetrahedronVertices(), std::move(reversedTriangles));

    requireNear(mesh.volume(), math::Real{1} / math::Real{6},
                "Reversed tetrahedron volume is incorrect.");
    requireVectorNear(
        mesh.centroid(),
        math::Vector3::Constant(math::Real{1} / math::Real{4}),
        "Reversed tetrahedron centroid is incorrect."
    );
    requireMatrixNear(
        mesh.unitDensityInertiaAtCentroid(),
        expectedTetrahedronInertia(),
        "Reversed tetrahedron inertia is incorrect."
    );
}

void testCuboid()
{
    const geometry::TriMeshData mesh(
        {
            vertex(math::Real{-1}, math::Real{-2}, math::Real{-3}),
            vertex(math::Real{1}, math::Real{-2}, math::Real{-3}),
            vertex(math::Real{1}, math::Real{2}, math::Real{-3}),
            vertex(math::Real{-1}, math::Real{2}, math::Real{-3}),
            vertex(math::Real{-1}, math::Real{-2}, math::Real{3}),
            vertex(math::Real{1}, math::Real{-2}, math::Real{3}),
            vertex(math::Real{1}, math::Real{2}, math::Real{3}),
            vertex(math::Real{-1}, math::Real{2}, math::Real{3})
        },
        {
            triangle(0, 2, 1), triangle(0, 3, 2),
            triangle(4, 5, 6), triangle(4, 6, 7),
            triangle(0, 1, 5), triangle(0, 5, 4),
            triangle(3, 7, 6), triangle(3, 6, 2),
            triangle(0, 4, 7), triangle(0, 7, 3),
            triangle(1, 2, 6), triangle(1, 6, 5)
        }
    );

    math::Matrix3 expectedInertia = math::Matrix3::Zero();
    expectedInertia(0, 0) = math::Real{208};
    expectedInertia(1, 1) = math::Real{160};
    expectedInertia(2, 2) = math::Real{80};
    requireNear(mesh.volume(), math::Real{48},
                "Cuboid volume is incorrect.");
    requireVectorNear(mesh.centroid(), math::Vector3::Zero(),
                      "Cuboid centroid is incorrect.");
    requireMatrixNear(mesh.unitDensityInertiaAtCentroid(), expectedInertia,
                      "Cuboid centroidal inertia is incorrect.");
}

void testOpenMeshHasNoSolidProperties()
{
    const geometry::TriMeshData mesh(
        {
            vertex(math::Real{0}, math::Real{0}, math::Real{0}),
            vertex(math::Real{1}, math::Real{0}, math::Real{0}),
            vertex(math::Real{0}, math::Real{1}, math::Real{0})
        },
        {triangle(0, 1, 2)}
    );
    require(!mesh.hasSolidGeometryProperties(),
            "An open mesh cannot have solid geometry properties.");
    require(!mesh.hasVolume(), "An open mesh cannot have volume.");
    requireThrows<std::logic_error>(
        [&mesh] { static_cast<void>(mesh.solidGeometryProperties()); },
        "Open mesh solidGeometryProperties() must throw."
    );
    requireThrows<std::logic_error>(
        [&mesh] { static_cast<void>(mesh.volume()); },
        "Open mesh volume() must throw."
    );
    requireThrows<std::logic_error>(
        [&mesh] { static_cast<void>(mesh.centroid()); },
        "Open mesh centroid() must throw."
    );
    requireThrows<std::logic_error>(
        [&mesh] {
            static_cast<void>(mesh.unitDensityInertiaAtCentroid());
        },
        "Open mesh inertia getter must throw."
    );
}

} // namespace

int main()
{
    try {
        testUnitTetrahedron();
        testTranslatedTetrahedronAndInputPreservation();
        testReversedTetrahedron();
        testCuboid();
        testOpenMeshHasNoSolidProperties();
    } catch (const std::exception& error) {
        std::cerr << "Solid geometry-properties test failed: "
                  << error.what() << '\n';
        return 1;
    }

    std::cout << "GyPhysics solid geometry-properties tests passed with Real="
              << math::realTypeName() << '\n';
    return 0;
}
