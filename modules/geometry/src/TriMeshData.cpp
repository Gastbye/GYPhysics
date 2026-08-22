#include "gy/physics/geometry/TriMeshData.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <Eigen/Eigenvalues>

namespace gy::physics::geometry {
namespace {

[[nodiscard]] math::Real integrateSquare(
    math::Real signedVolume,
    math::Real a,
    math::Real b,
    math::Real c) noexcept
{
    return signedVolume / math::Real{10}
        * (a * a + b * b + c * c + a * b + a * c + b * c);
}

[[nodiscard]] math::Real integrateProduct(
    math::Real signedVolume,
    math::Real ax,
    math::Real ay,
    math::Real bx,
    math::Real by,
    math::Real cx,
    math::Real cy) noexcept
{
    return signedVolume / math::Real{20}
        * (math::Real{2} * (ax * ay + bx * by + cx * cy)
           + ax * by + ay * bx
           + ax * cy + ay * cx
           + bx * cy + by * cx);
}

[[nodiscard]] math::Matrix3 tetrahedronInertiaAtOrigin(
    math::Real signedVolume,
    const math::Vector3& a,
    const math::Vector3& b,
    const math::Vector3& c) noexcept
{
    const math::Real integralXX = integrateSquare(
        signedVolume, a.x(), b.x(), c.x());
    const math::Real integralYY = integrateSquare(
        signedVolume, a.y(), b.y(), c.y());
    const math::Real integralZZ = integrateSquare(
        signedVolume, a.z(), b.z(), c.z());
    const math::Real integralXY = integrateProduct(
        signedVolume,
        a.x(), a.y(), b.x(), b.y(), c.x(), c.y());
    const math::Real integralXZ = integrateProduct(
        signedVolume,
        a.x(), a.z(), b.x(), b.z(), c.x(), c.z());
    const math::Real integralYZ = integrateProduct(
        signedVolume,
        a.y(), a.z(), b.y(), b.z(), c.y(), c.z());

    math::Matrix3 inertia;
    inertia(0, 0) = integralYY + integralZZ;
    inertia(1, 1) = integralXX + integralZZ;
    inertia(2, 2) = integralXX + integralYY;
    inertia(0, 1) = -integralXY;
    inertia(1, 0) = -integralXY;
    inertia(0, 2) = -integralXZ;
    inertia(2, 0) = -integralXZ;
    inertia(1, 2) = -integralYZ;
    inertia(2, 1) = -integralYZ;
    return inertia;
}

[[nodiscard]] math::Real validationFactor() noexcept
{
    return math::Real{128} * std::numeric_limits<math::Real>::epsilon();
}

void validateSolidGeometryProperties(
    const SolidGeometryProperties& properties,
    math::Real characteristicLength)
{
    if (!std::isfinite(properties.volume)
        || !properties.centroid.allFinite()
        || !properties.unitDensityInertiaAtCentroid.allFinite()) {
        throw std::runtime_error(
            "Triangle mesh geometry integration produced non-finite values."
        );
    }

    const math::Matrix3& inertia =
        properties.unitDensityInertiaAtCentroid;
    const math::Real inertiaScale = std::max(
        properties.volume * characteristicLength * characteristicLength,
        inertia.cwiseAbs().maxCoeff()
    );
    const math::Real tolerance = validationFactor() * inertiaScale;
    if ((inertia - inertia.transpose()).cwiseAbs().maxCoeff() > tolerance) {
        throw std::runtime_error(
            "Triangle mesh inertia tensor is not sufficiently symmetric."
        );
    }

    Eigen::SelfAdjointEigenSolver<math::Matrix3> eigenSolver(inertia);
    if (eigenSolver.info() != Eigen::Success) {
        throw std::runtime_error(
            "Triangle mesh inertia eigenvalue validation failed."
        );
    }
    if (eigenSolver.eigenvalues().minCoeff() < -tolerance) {
        throw std::runtime_error(
            "Triangle mesh inertia has a significantly negative principal value."
        );
    }
}

[[nodiscard]] SolidGeometryProperties computeSolidGeometryProperties(
    const TriMeshData::VertexContainer& vertices,
    const TriMeshData::TriangleContainer& triangles)
{
    const math::Vector3 referencePoint = vertices.front();
    math::Vector3 boundsMinimum = referencePoint;
    math::Vector3 boundsMaximum = referencePoint;
    for (const math::Vector3& vertex : vertices) {
        boundsMinimum = boundsMinimum.cwiseMin(vertex);
        boundsMaximum = boundsMaximum.cwiseMax(vertex);
    }
    const math::Real characteristicLength =
        (boundsMaximum - boundsMinimum).norm();
    const math::Real volumeTolerance = validationFactor()
        * characteristicLength * characteristicLength * characteristicLength;

    math::Real totalSignedVolume = math::kZero;
    math::Vector3 firstMoment = math::Vector3::Zero();
    math::Matrix3 inertiaAtReference = math::Matrix3::Zero();

    for (const math::IntV3& triangle : triangles) {
        const math::Vector3 a =
            vertices[static_cast<std::size_t>(triangle[0])] - referencePoint;
        const math::Vector3 b =
            vertices[static_cast<std::size_t>(triangle[1])] - referencePoint;
        const math::Vector3 c =
            vertices[static_cast<std::size_t>(triangle[2])] - referencePoint;
        const math::Real signedVolume =
            a.dot(b.cross(c)) / math::Real{6};

        totalSignedVolume += signedVolume;
        firstMoment += signedVolume * (a + b + c) / math::Real{4};
        inertiaAtReference += tetrahedronInertiaAtOrigin(
            signedVolume, a, b, c);
    }

    if (totalSignedVolume < math::kZero) {
        totalSignedVolume = -totalSignedVolume;
        firstMoment = -firstMoment;
        inertiaAtReference = -inertiaAtReference;
    }
    if (!std::isfinite(characteristicLength)
        || !std::isfinite(totalSignedVolume)
        || totalSignedVolume <= volumeTolerance) {
        throw std::runtime_error(
            "Triangle mesh signed volume is not a finite, significant solid volume."
        );
    }

    const math::Vector3 localCentroid = firstMoment / totalSignedVolume;
    const math::Vector3 globalCentroid = referencePoint + localCentroid;
    const math::Matrix3 parallelAxisTerm = totalSignedVolume
        * (localCentroid.squaredNorm() * math::Matrix3::Identity()
           - localCentroid * localCentroid.transpose());
    const math::Matrix3 inertiaAtCentroid =
        inertiaAtReference - parallelAxisTerm;
    const math::Matrix3 symmetricInertia = math::Real{0.5}
        * (inertiaAtCentroid + inertiaAtCentroid.transpose());

    SolidGeometryProperties properties;
    properties.volume = totalSignedVolume;
    properties.centroid = globalCentroid;
    properties.unitDensityInertiaAtCentroid = symmetricInertia;
    validateSolidGeometryProperties(properties, characteristicLength);
    return properties;
}

struct EdgeKey
{
    math::Index low{};
    math::Index high{};

    [[nodiscard]] bool operator==(const EdgeKey& other) const noexcept
    {
        return low == other.low && high == other.high;
    }
};

struct EdgeKeyHash
{
    [[nodiscard]] std::size_t operator()(const EdgeKey& edge) const noexcept
    {
        const std::size_t lowHash = std::hash<math::Index>{}(edge.low);
        const std::size_t highHash = std::hash<math::Index>{}(edge.high);
        return lowHash ^ (highHash << 1U);
    }
};

struct EdgeInfo
{
    std::size_t useCount{};
    std::size_t lowToHighCount{};
};

[[nodiscard]] EdgeKey makeEdgeKey(math::Index from, math::Index to) noexcept
{
    return EdgeKey{std::min(from, to), std::max(from, to)};
}

[[noreturn]] void throwTriangleError(
    std::size_t triangleIndex,
    const std::string& reason)
{
    std::ostringstream message;
    message << "Triangle " << triangleIndex << ' ' << reason;
    throw std::invalid_argument(message.str());
}

} // namespace

TriMeshData::TriMeshData(
    VertexContainer vertices,
    TriangleContainer triangles)
    : vertices_(std::move(vertices)),
      triangles_(std::move(triangles))
{
    if (vertices_.empty()) {
        throw std::invalid_argument(
            "Triangle mesh must contain at least one vertex."
        );
    }
    if (triangles_.empty()) {
        throw std::invalid_argument(
            "Triangle mesh must contain at least one triangle."
        );
    }

    for (std::size_t vertexIndex = 0;
         vertexIndex < vertices_.size();
         ++vertexIndex) {
        const math::Vector3& vertex = vertices_[vertexIndex];
        if (!vertex.allFinite()) {
            std::ostringstream message;
            message << "Vertex " << vertexIndex
                    << " contains a non-finite coordinate.";
            throw std::invalid_argument(message.str());
        }
    }

    std::unordered_map<EdgeKey, EdgeInfo, EdgeKeyHash> edges;
    edges.reserve(triangles_.size() * std::size_t{3});

    const math::Real degeneracyFactor =
        math::Real{64} * std::numeric_limits<math::Real>::epsilon();

    for (std::size_t triangleIndex = 0;
         triangleIndex < triangles_.size();
         ++triangleIndex) {
        const math::IntV3& triangle = triangles_[triangleIndex];
        const math::Index i0 = triangle[0];
        const math::Index i1 = triangle[1];
        const math::Index i2 = triangle[2];

        const math::Index indices[3] = {i0, i1, i2};
        for (const math::Index index : indices) {
            const std::size_t vertexIndex = static_cast<std::size_t>(index);
            if (vertexIndex >= vertices_.size()) {
                std::ostringstream reason;
                reason << "references out-of-range vertex index " << index
                       << ".";
                throwTriangleError(triangleIndex, reason.str());
            }
        }
        if (i0 == i1 || i1 == i2 || i2 == i0) {
            throwTriangleError(triangleIndex, "contains repeated indices.");
        }

        const math::Vector3& a = vertices_[static_cast<std::size_t>(i0)];
        const math::Vector3& b = vertices_[static_cast<std::size_t>(i1)];
        const math::Vector3& c = vertices_[static_cast<std::size_t>(i2)];
        const math::Vector3 ab = b - a;
        const math::Vector3 bc = c - b;
        const math::Vector3 ca = a - c;
        const math::Vector3 doubledArea = ab.cross(c - a);
        if (!ab.allFinite() || !bc.allFinite() || !ca.allFinite()
            || !doubledArea.allFinite()) {
            throwTriangleError(
                triangleIndex,
                "cannot be analyzed without floating-point overflow."
            );
        }

        const math::Real maxEdgeSquared = std::max(
            {ab.squaredNorm(), bc.squaredNorm(), ca.squaredNorm()}
        );
        const math::Real doubledAreaNorm = doubledArea.norm();
        if (!std::isfinite(maxEdgeSquared)
            || !std::isfinite(doubledAreaNorm)
            || doubledAreaNorm <= degeneracyFactor * maxEdgeSquared) {
            throwTriangleError(
                triangleIndex,
                "is degenerate or has area too close to zero."
            );
        }

        const math::Index edgeStarts[3] = {i0, i1, i2};
        const math::Index edgeEnds[3] = {i1, i2, i0};
        for (std::size_t localEdge = 0; localEdge < std::size_t{3}; ++localEdge) {
            const math::Index from = edgeStarts[localEdge];
            const math::Index to = edgeEnds[localEdge];
            const EdgeKey key = makeEdgeKey(from, to);
            EdgeInfo& info = edges[key];
            ++info.useCount;
            if (from == key.low) {
                ++info.lowToHighCount;
            }
        }

    }

    topology_.consistentlyOriented = true;
    for (const auto& edgeEntry : edges) {
        const EdgeInfo& info = edgeEntry.second;
        if (info.useCount == std::size_t{1}) {
            ++topology_.boundaryEdgeCount;
        } else if (info.useCount >= std::size_t{3}) {
            ++topology_.nonManifoldEdgeCount;
        } else if (info.lowToHighCount != std::size_t{1}) {
            topology_.consistentlyOriented = false;
        }
    }

    manifold_ = topology_.nonManifoldEdgeCount == std::size_t{0};
    closed_ = topology_.canRepresentSolid();

    // This topology analysis intentionally does not detect intersections
    // between triangles. A topologically closed mesh can still self-intersect.
    if (closed_) {
        solidGeometryProperties_ = computeSolidGeometryProperties(
            vertices_, triangles_);
    }
}

const SolidGeometryProperties&
TriMeshData::solidGeometryProperties() const
{
    if (!solidGeometryProperties_) {
        throw std::logic_error(
            "Triangle mesh does not have cached solid geometry properties."
        );
    }
    return *solidGeometryProperties_;
}

math::Real TriMeshData::volume() const
{
    return solidGeometryProperties().volume;
}

const math::Vector3& TriMeshData::centroid() const
{
    return solidGeometryProperties().centroid;
}

const math::Matrix3& TriMeshData::unitDensityInertiaAtCentroid() const
{
    return solidGeometryProperties().unitDensityInertiaAtCentroid;
}

} // namespace gy::physics::geometry
