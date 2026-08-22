#include <cstddef>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "gy/physics/rigid/RigidBodyRegistry.h"
#include "gy/physics/geometry/ShapeRegistry.h"
#include "gy/physics/geometry/TriMeshShape.h"

namespace {

namespace geometry = gy::physics::geometry;
namespace math = gy::physics::math;
namespace rigid = gy::physics::rigid;

struct RigidBodyInstance
{
    rigid::BodyId bodyId;
    geometry::ShapeId shapeId;
};

[[nodiscard]] std::shared_ptr<const geometry::TriMeshData>
makeTetrahedron()
{
    geometry::TriMeshData::VertexContainer vertices{
        math::Vector3(math::Real{0}, math::Real{0}, math::Real{0}),
        math::Vector3(math::Real{1}, math::Real{0}, math::Real{0}),
        math::Vector3(math::Real{0}, math::Real{1}, math::Real{0}),
        math::Vector3(math::Real{0}, math::Real{0}, math::Real{1})
    };
    geometry::TriMeshData::TriangleContainer triangles{
        math::IntV3(math::Index{1}, math::Index{2}, math::Index{3}),
        math::IntV3(math::Index{0}, math::Index{2}, math::Index{1}),
        math::IntV3(math::Index{0}, math::Index{1}, math::Index{3}),
        math::IntV3(math::Index{0}, math::Index{3}, math::Index{2})
    };

    return std::make_shared<const geometry::TriMeshData>(
        std::move(vertices), std::move(triangles));
}

} // namespace

int main()
{
    constexpr std::size_t bodyCount = 20;
    constexpr std::size_t bodiesPerRow = 5;

    geometry::ShapeRegistry shapeRegistry;
    rigid::RigidBodyRegistry bodyRegistry;

    const std::shared_ptr<const geometry::TriMeshData> tetrahedron =
        makeTetrahedron();
    const geometry::ShapeId tetrahedronShapeId = shapeRegistry.add(
        std::make_unique<geometry::TriMeshShape>(tetrahedron)
    );

    std::vector<RigidBodyInstance> instances;
    instances.reserve(bodyCount);

    const math::Real bodyMass = math::Real{1};
    const math::Real density = bodyMass / tetrahedron->volume();

    for (std::size_t index = 0; index < bodyCount; ++index) {
        rigid::RigidBodyDesc desc;
        desc.massProperties.mass = bodyMass;
        desc.massProperties.centerOfMassLocalPosition =
            tetrahedron->centroid();
        desc.massProperties.inertiaTensorLocalAtCenterOfMass =
            density * tetrahedron->unitDensityInertiaAtCentroid();
        desc.initialState.position = math::Vector3(
            math::Real{2} * static_cast<math::Real>(index % bodiesPerRow),
            math::Real{2} * static_cast<math::Real>(int(index / bodiesPerRow)),
            math::Real{0}
        );

        const rigid::BodyId bodyId = bodyRegistry.add(rigid::RigidBody(desc));
        instances.push_back(RigidBodyInstance{bodyId, tetrahedronShapeId});
    }

    std::cout << "Created " << bodyRegistry.size()
              << " rigid bodies sharing tetrahedron shape "
              << tetrahedronShapeId.value << "\n";

    for (const RigidBodyInstance& instance : instances) {
        const rigid::RigidBody& body = bodyRegistry.get(instance.bodyId);
        std::cout << "body " << instance.bodyId.value
                  << ", shape " << instance.shapeId.value
                  << ", position "
                  << body.state().position.transpose() << '\n';
    }

    return 0;
}
