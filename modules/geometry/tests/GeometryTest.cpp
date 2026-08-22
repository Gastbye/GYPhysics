#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "gy/physics/geometry/SphereShape.h"
#include "gy/physics/geometry/TriMeshData.h"
#include "gy/physics/geometry/TriMeshShape.h"

namespace {

namespace geometry = gy::physics::geometry;
namespace math = gy::physics::math;

#if GY_PHYSICS_REAL_IS_DOUBLE == 1
constexpr math::Real kTolerance = math::Real{1.0e-12};
#else
constexpr math::Real kTolerance = math::Real{1.0e-5F};
#endif

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
    if (std::abs(actual - expected) > kTolerance) {
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

void testSphereShape()
{
    const math::Real pi = std::acos(math::Real{-1});
    const geometry::SphereShape unitSphere(math::Real{1});
    require(unitSphere.type() == geometry::ShapeType::Sphere,
            "SphereShape has the wrong shape type.");
    requireNear(unitSphere.radius(), math::Real{1},
                "SphereShape did not cache its radius.");
    requireNear(unitSphere.volume(), math::Real{4} * pi / math::Real{3},
                "Unit sphere volume is incorrect.");

    const geometry::SphereShape radiusTwo(math::Real{2});
    requireNear(radiusTwo.volume(), math::Real{32} * pi / math::Real{3},
                "Radius-two sphere volume is incorrect.");

    requireThrows<std::invalid_argument>(
        [] { geometry::SphereShape sphere(math::Real{0}); },
        "A zero sphere radius must be rejected."
    );
    requireThrows<std::invalid_argument>(
        [] { geometry::SphereShape sphere(math::Real{-1}); },
        "A negative sphere radius must be rejected."
    );
    requireThrows<std::invalid_argument>(
        [] {
            geometry::SphereShape sphere(
                std::numeric_limits<math::Real>::quiet_NaN()
            );
        },
        "A NaN sphere radius must be rejected."
    );
    requireThrows<std::invalid_argument>(
        [] {
            geometry::SphereShape sphere(
                std::numeric_limits<math::Real>::infinity()
            );
        },
        "An infinite sphere radius must be rejected."
    );
}

void testOpenTriangleMesh()
{
    geometry::TriMeshData mesh(
        {
            vertex(math::Real{0}, math::Real{0}, math::Real{0}),
            vertex(math::Real{1}, math::Real{0}, math::Real{0}),
            vertex(math::Real{0}, math::Real{1}, math::Real{0})
        },
        {triangle(0, 1, 2)}
    );

    require(mesh.vertexCount() == std::size_t{3},
            "Open mesh vertex count is incorrect.");
    require(mesh.triangleCount() == std::size_t{1},
            "Open mesh triangle count is incorrect.");
    require(mesh.boundaryEdgeCount() == std::size_t{3},
            "Open mesh boundary-edge count is incorrect.");
    require(mesh.nonManifoldEdgeCount() == std::size_t{0},
            "Open mesh has an unexpected non-manifold edge.");
    require(mesh.topology().boundaryEdgeCount == std::size_t{3},
            "The compatibility topology view is incorrect.");
    require(mesh.isManifold(), "An open triangle should be manifold.");
    require(!mesh.isClosed(), "An open triangle cannot be closed.");
    require(!mesh.hasVolume(), "An open triangle cannot have volume.");
    requireThrows<std::logic_error>(
        [&mesh] { static_cast<void>(mesh.volume()); },
        "Open mesh volume access must fail."
    );
}

void testClosedTetrahedron()
{
    const geometry::TriMeshData mesh(
        tetrahedronVertices(),
        tetrahedronTriangles()
    );
    require(mesh.vertexCount() == std::size_t{4},
            "Tetrahedron vertex count is incorrect.");
    require(mesh.triangleCount() == std::size_t{4},
            "Tetrahedron triangle count is incorrect.");
    require(mesh.boundaryEdgeCount() == std::size_t{0},
            "Tetrahedron has an unexpected boundary edge.");
    require(mesh.nonManifoldEdgeCount() == std::size_t{0},
            "Tetrahedron has an unexpected non-manifold edge.");
    require(mesh.isManifold(), "Tetrahedron should be manifold.");
    require(mesh.isConsistentlyOriented(),
            "Tetrahedron faces should be consistently oriented.");
    require(mesh.isClosed(), "Tetrahedron should be closed.");
    require(mesh.hasVolume(), "Tetrahedron should have volume.");
    requireNear(mesh.volume(), math::Real{1} / math::Real{6},
                "Tetrahedron volume is incorrect.");
    require(mesh.volumeOptional().has_value(),
            "Tetrahedron optional volume is missing.");
}

void testReversedTetrahedron()
{
    auto triangles = tetrahedronTriangles();
    for (math::IntV3& face : triangles) {
        std::swap(face[1], face[2]);
    }
    const geometry::TriMeshData mesh(tetrahedronVertices(), std::move(triangles));
    require(mesh.isClosed(), "Reversed tetrahedron should remain closed.");
    require(mesh.isConsistentlyOriented(),
            "Reversed tetrahedron should remain consistently oriented.");
    requireNear(mesh.volume(), math::Real{1} / math::Real{6},
                "Reversed tetrahedron volume must remain positive.");
}

void testLocalOrientationError()
{
    auto triangles = tetrahedronTriangles();
    std::swap(triangles.front()[1], triangles.front()[2]);
    const geometry::TriMeshData mesh(tetrahedronVertices(), std::move(triangles));
    require(!mesh.isConsistentlyOriented(),
            "A locally reversed face must be detected.");
    require(!mesh.isClosed(),
            "An inconsistently oriented mesh must not represent a closed solid.");
    require(!mesh.hasVolume(),
            "An inconsistently oriented mesh must not have volume.");
}

void testClosedZeroVolumeMesh()
{
    requireThrows<std::runtime_error>(
        [] {
            geometry::TriMeshData mesh(
                {
                    vertex(math::Real{0}, math::Real{0}, math::Real{0}),
                    vertex(math::Real{1}, math::Real{0}, math::Real{0}),
                    vertex(math::Real{0}, math::Real{1}, math::Real{0}),
                    vertex(math::Real{1}, math::Real{1}, math::Real{0})
                },
                tetrahedronTriangles()
            );
        },
        "A topologically closed mesh with near-zero volume must be rejected."
    );
}

void testInvalidMeshes()
{
    const auto vertices = tetrahedronVertices();
    requireThrows<std::invalid_argument>(
        [] {
            geometry::TriMeshData mesh({}, {triangle(0, 1, 2)});
        },
        "An empty vertex array must be rejected."
    );
    requireThrows<std::invalid_argument>(
        [vertices] {
            geometry::TriMeshData mesh(vertices, {});
        },
        "An empty triangle array must be rejected."
    );
    requireThrows<std::invalid_argument>(
        [vertices] {
            geometry::TriMeshData mesh(vertices, {triangle(0, 1, 9)});
        },
        "An out-of-range triangle index must be rejected."
    );
    requireThrows<std::invalid_argument>(
        [vertices] {
            geometry::TriMeshData mesh(vertices, {triangle(0, 0, 1)});
        },
        "Repeated triangle indices must be rejected."
    );
    requireThrows<std::invalid_argument>(
        [] {
            geometry::TriMeshData mesh(
                {
                    vertex(math::Real{0}, math::Real{0}, math::Real{0}),
                    vertex(math::Real{1}, math::Real{0}, math::Real{0}),
                    vertex(math::Real{2}, math::Real{0}, math::Real{0})
                },
                {triangle(0, 1, 2)}
            );
        },
        "A collinear triangle must be rejected."
    );
    requireThrows<std::invalid_argument>(
        [] {
            geometry::TriMeshData mesh(
                {
                    vertex(math::Real{0}, math::Real{0}, math::Real{0}),
                    vertex(
                        std::numeric_limits<math::Real>::quiet_NaN(),
                        math::Real{0},
                        math::Real{0}
                    ),
                    vertex(math::Real{0}, math::Real{1}, math::Real{0})
                },
                {triangle(0, 1, 2)}
            );
        },
        "A non-finite vertex coordinate must be rejected."
    );
}

void testNonManifoldEdge()
{
    const geometry::TriMeshData mesh(
        {
            vertex(math::Real{0}, math::Real{0}, math::Real{0}),
            vertex(math::Real{1}, math::Real{0}, math::Real{0}),
            vertex(math::Real{0}, math::Real{1}, math::Real{0}),
            vertex(math::Real{0}, math::Real{0}, math::Real{1}),
            vertex(math::Real{0}, math::Real{-1}, math::Real{0})
        },
        {
            triangle(0, 1, 2),
            triangle(1, 0, 3),
            triangle(0, 1, 4)
        }
    );
    require(mesh.nonManifoldEdgeCount() > std::size_t{0},
            "An edge used by three faces must be non-manifold.");
    require(!mesh.isManifold(), "The non-manifold mesh was not detected.");
    require(!mesh.isClosed(), "A non-manifold mesh cannot be closed.");
    require(!mesh.hasVolume(), "A non-manifold mesh cannot have volume.");
}

void testTriMeshShape()
{
    const std::shared_ptr<const geometry::TriMeshData> data =
        std::make_shared<geometry::TriMeshData>(
            tetrahedronVertices(),
            tetrahedronTriangles()
        );
    const geometry::TriMeshShape first(data);
    const geometry::TriMeshShape second(data);

    require(first.type() == geometry::ShapeType::TriangleMesh,
            "TriMeshShape has the wrong shape type.");
    require(&first.data() == data.get(),
            "TriMeshShape did not expose the shared mesh data.");
    require(first.dataPointer().get() == data.get(),
            "TriMeshShape did not retain the original shared pointer.");
    require(second.dataPointer().get() == first.dataPointer().get(),
            "Two TriMeshShape instances should share one TriMeshData object.");
    require(first.isClosed(), "TriMeshShape did not forward isClosed().");
    require(first.hasVolume(), "TriMeshShape did not forward hasVolume().");
    require(first.hasSolidGeometryProperties(),
            "TriMeshShape did not forward hasSolidGeometryProperties().");
    requireNear(first.volume(), math::Real{1} / math::Real{6},
                "TriMeshShape did not forward volume().");
    requireNear(first.centroid().x(), math::Real{1} / math::Real{4},
                "TriMeshShape did not forward centroid().");
    require(first.unitDensityInertiaAtCentroid().allFinite(),
            "TriMeshShape did not forward the inertia tensor.");

    requireThrows<std::invalid_argument>(
        [] {
            const std::shared_ptr<const geometry::TriMeshData> nullData;
            geometry::TriMeshShape shape(nullData);
        },
        "TriMeshShape must reject a null data pointer."
    );
}

} // namespace

int main()
{
    try {
        testSphereShape();
        testOpenTriangleMesh();
        testClosedTetrahedron();
        testReversedTetrahedron();
        testLocalOrientationError();
        testClosedZeroVolumeMesh();
        testInvalidMeshes();
        testNonManifoldEdge();
        testTriMeshShape();
    } catch (const std::exception& error) {
        std::cerr << "Geometry test failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "GyPhysics Geometry tests passed with Real="
              << math::realTypeName() << '\n';
    return 0;
}
