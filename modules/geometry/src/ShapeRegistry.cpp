#include "gy/physics/geometry/ShapeRegistry.h"

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>

namespace gy::physics::geometry {

ShapeId ShapeRegistry::add(std::unique_ptr<const Shape> shape)
{
    if (!shape) {
        throw std::invalid_argument(
            "ShapeRegistry::add: shape must not be null"
        );
    }

    if (shapes_.size()
        >= static_cast<std::size_t>(math::InvalidIndex)) {
        throw std::length_error("ShapeRegistry has exhausted shape IDs");
    }

    const ShapeId id{
        static_cast<math::Index>(shapes_.size())
    };
    shapes_.emplace_back(std::move(shape));
    ++size_;
    return id;
}

const Shape& ShapeRegistry::get(ShapeId id) const
{
    if (!contains(id)) {
        throw std::out_of_range("ShapeRegistry::get: invalid shape ID");
    }
    return *shapes_[static_cast<std::size_t>(id.value)];
}

bool ShapeRegistry::contains(ShapeId id) const noexcept
{
    if (!id.isValid()) {
        return false;
    }

    const std::size_t index = static_cast<std::size_t>(id.value);
    return index < shapes_.size() && shapes_[index] != nullptr;
}

void ShapeRegistry::remove(ShapeId id)
{
    if (!contains(id)) {
        throw std::out_of_range(
            "ShapeRegistry::remove: invalid shape ID"
        );
    }

    shapes_[static_cast<std::size_t>(id.value)].reset();
    --size_;
}

std::size_t ShapeRegistry::size() const noexcept
{
    return size_;
}

bool ShapeRegistry::empty() const noexcept
{
    return size_ == 0;
}

} // namespace gy::physics::geometry
