/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "gy/physics/shape/Shape.h"
#include "gy/physics/shape/ShapeStruct.h"

namespace gy::physics::geometry {

class ShapeRegistry
{
public:
    // ShapeId remains stable across additions.
    [[nodiscard]] ShapeId add(std::unique_ptr<const Shape> shape);

    // References returned by get() must not be retained across registry
    // mutations.
    [[nodiscard]] const Shape&
    get(ShapeId id) const;

    [[nodiscard]] bool contains(
        ShapeId id) const noexcept;

    void remove(ShapeId id);

    // Returns the number of currently registered shapes, excluding removed
    // slots.
    [[nodiscard]] std::size_t size() const noexcept;
    [[nodiscard]] bool empty() const noexcept;

private:
    // Slots are never erased or reused: every issued ID keeps one meaning for
    // the lifetime of this registry.
    std::vector<std::unique_ptr<const Shape>> shapes_;
    std::size_t size_{0};
};

} // namespace gy::physics::geometry
