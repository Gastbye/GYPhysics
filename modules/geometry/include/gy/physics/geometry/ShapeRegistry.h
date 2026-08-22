/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "gy/physics/geometry/Shape.h"
#include "gy/physics/geometry/ShapeStruct.h"

namespace gy::physics::geometry {

class ShapeRegistry
{
public:
    [[nodiscard]] ShapeId add(std::unique_ptr<const Shape> shape);

    [[nodiscard]] const Shape&
    get(ShapeId id) const;

    [[nodiscard]] bool contains(
        ShapeId id) const noexcept;

    void remove(ShapeId id);

    [[nodiscard]] std::size_t size() const noexcept;
    [[nodiscard]] bool empty() const noexcept;

private:
    std::vector<std::unique_ptr<const Shape>> shapes_;
    std::size_t size_{0};
};

} // namespace gy::physics::geometry
