/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <memory>
#include <stdexcept>
#include <utility>

#include "gy/physics/geometry/Shape.h"
#include "gy/physics/geometry/TriMeshData.h"

namespace gy::physics::geometry {

class TriMeshShape final : public Shape
{
public:
    explicit TriMeshShape(
        std::shared_ptr<const TriMeshData> data)
        : Shape(ShapeType::TriangleMesh, ShapeExtend{}),
          data_(std::move(data))
    {
        if (!data_) {
            throw std::invalid_argument(
                "Triangle mesh data must not be null."
            );
        }
    }

    [[nodiscard]] const TriMeshData& data() const noexcept
    {
        return *data_;
    }

    [[nodiscard]] const std::shared_ptr<const TriMeshData>&
    dataPointer() const noexcept
    {
        return data_;
    }

    [[nodiscard]] bool isClosed() const noexcept
    {
        return data_->isClosed();
    }

    [[nodiscard]] bool hasVolume() const noexcept
    {
        return data_->hasVolume();
    }

    [[nodiscard]] bool hasSolidGeometryProperties() const noexcept
    {
        return data_->hasSolidGeometryProperties();
    }

    [[nodiscard]] const SolidGeometryProperties&
    solidGeometryProperties() const
    {
        return data_->solidGeometryProperties();
    }

    [[nodiscard]] math::Real volume() const
    {
        return data_->volume();
    }

    [[nodiscard]] const math::Vector3& centroid() const
    {
        return data_->centroid();
    }

    [[nodiscard]] const math::Matrix3&
    unitDensityInertiaAtCentroid() const
    {
        return data_->unitDensityInertiaAtCentroid();
    }

    // Compatibility with the accessor name used by the initial draft.
    [[nodiscard]] const TriMeshData& mesh() const noexcept
    {
        return data();
    }

private:
    std::shared_ptr<const TriMeshData> data_;
};

} // namespace gy::physics::geometry
