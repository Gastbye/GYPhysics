/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <cmath>
#include <stdexcept>

#include "gy/physics/math/MathTypes.h"
#include "gy/physics/geometry/Shape.h"

namespace gy::physics::geometry {

class SphereShape final : public Shape
{
public:
    explicit SphereShape(math::Real radius)
        : Shape(ShapeType::Sphere, ShapeExtend{}),
          radius_(validatedRadius(radius)),
          volume_(math::Real{4} / math::Real{3}
              * std::acos(math::Real{-1})
              * radius_ * radius_ * radius_)
    {
    }

    [[nodiscard]] math::Real radius() const noexcept
    {
        return radius_;
    }

    [[nodiscard]] math::Real volume() const noexcept
    {
        return volume_;
    }

    [[nodiscard]] math::Vector3 centroid() const noexcept
    {
        return math::Vector3::Zero();
    }

private:
    [[nodiscard]] static math::Real validatedRadius(math::Real radius)
    {
        if (!std::isfinite(radius) || radius <= math::Real{0}) {
            throw std::invalid_argument(
                "Sphere radius must be finite and greater than zero."
            );
        }
        return radius;
    }

    math::Real radius_;
    math::Real volume_;
};

} // namespace gy::physics::geometry
