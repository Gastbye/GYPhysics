#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

#include "gy/physics/material/MaterialRegistry.h"

namespace gy::physics::material {

MaterialId MaterialRegistry::add(Material material)
{
    if (!std::isfinite(material.density())
        || material.density() <= math::kZero) {
        throw std::invalid_argument(
            "MaterialRegistry::add: material density is not set"
        );
    }

    if (materials_.size()
        >= static_cast<std::size_t>(math::InvalidIndex)) {
        throw std::length_error("MaterialRegistry has exhausted material IDs");
    }

    const MaterialId id{
        static_cast<math::Index>(materials_.size())
    };
    materials_.emplace_back(std::move(material));
    ++size_;
    return id;
}

const Material& MaterialRegistry::get(MaterialId id) const
{
    if (!contains(id)) {
        throw std::out_of_range("MaterialRegistry::get: invalid material ID");
    }
    return *materials_[static_cast<std::size_t>(id.value)];
}

bool MaterialRegistry::contains(MaterialId id) const noexcept
{
    if (!id.isValid()) {
        return false;
    }

    const std::size_t index = static_cast<std::size_t>(id.value);
    return index < materials_.size() && materials_[index].has_value();
}

void MaterialRegistry::remove(MaterialId id)
{
    if (!contains(id)) {
        throw std::out_of_range(
            "MaterialRegistry::remove: invalid material ID"
        );
    }

    materials_[static_cast<std::size_t>(id.value)].reset();
    --size_;
}

std::size_t MaterialRegistry::size() const noexcept
{
    return size_;
}

bool MaterialRegistry::empty() const noexcept
{
    return size_ == 0;
}

} // namespace gy::physics::material
