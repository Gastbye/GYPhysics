/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "gy/physics/math/MathTypes.h"
#include "gy/physics/shape/ShapeStruct.h"

namespace gy::physics::geometry {

struct MeshTopologyInfo
{
    std::size_t boundaryEdgeCount{};
    std::size_t nonManifoldEdgeCount{};
    bool consistentlyOriented{false};

    [[nodiscard]] bool isWatertight() const noexcept
    {
        return boundaryEdgeCount == std::size_t{0}
            && nonManifoldEdgeCount == std::size_t{0};
    }

    [[nodiscard]] bool canRepresentSolid() const noexcept
    {
        return isWatertight() && consistentlyOriented;
    }
};

class TriMeshData
{
public:
    using VertexContainer = std::vector<math::Vector3>;
    using TriangleContainer = std::vector<math::IntV3>;

    TriMeshData(
        VertexContainer vertices,
        TriangleContainer triangles);

    [[nodiscard]] const VertexContainer& vertices() const noexcept
    {
        return vertices_;
    }

    [[nodiscard]] const TriangleContainer& triangles() const noexcept
    {
        return triangles_;
    }

    [[nodiscard]] const MeshTopologyInfo& topology() const noexcept
    {
        return topology_;
    }

    [[nodiscard]] std::size_t vertexCount() const noexcept
    {
        return vertices_.size();
    }

    [[nodiscard]] std::size_t triangleCount() const noexcept
    {
        return triangles_.size();
    }

    [[nodiscard]] bool isClosed() const noexcept
    {
        return closed_;
    }

    [[nodiscard]] bool isManifold() const noexcept
    {
        return manifold_;
    }

    [[nodiscard]] bool isConsistentlyOriented() const noexcept
    {
        return topology_.consistentlyOriented;
    }

    [[nodiscard]] std::size_t boundaryEdgeCount() const noexcept
    {
        return topology_.boundaryEdgeCount;
    }

    [[nodiscard]] std::size_t nonManifoldEdgeCount() const noexcept
    {
        return topology_.nonManifoldEdgeCount;
    }

    [[nodiscard]] bool hasSolidGeometryProperties() const noexcept
    {
        return solidGeometryProperties_.has_value();
    }

    [[nodiscard]] const SolidGeometryProperties&
    solidGeometryProperties() const;

    [[nodiscard]] bool hasVolume() const noexcept
    {
        return hasSolidGeometryProperties();
    }

    [[nodiscard]] math::Real volume() const;
    [[nodiscard]] const math::Vector3& centroid() const;
    [[nodiscard]] const math::Matrix3&
    unitDensityInertiaAtCentroid() const;

    [[nodiscard]] std::optional<math::Real>
    volumeOptional() const noexcept
    {
        if (!solidGeometryProperties_) {
            return std::nullopt;
        }
        return solidGeometryProperties_->volume;
    }

private:
    VertexContainer vertices_;
    TriangleContainer triangles_;

    bool closed_{false};
    bool manifold_{false};
    MeshTopologyInfo topology_{};

    std::optional<SolidGeometryProperties> solidGeometryProperties_;
};

} // namespace gy::physics::geometry
