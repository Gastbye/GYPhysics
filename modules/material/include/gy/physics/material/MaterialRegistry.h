/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "gy/physics/material/Material.h"
#include "gy/physics/material/MaterialStruct.h"

namespace gy::physics::material {

class MaterialRegistry
{
public:
    // MaterialId remains stable across additions.
    [[nodiscard]] MaterialId add(Material material);

    // References returned by get() must not be retained across registry
    // mutations.
    [[nodiscard]] const Material&
    get(MaterialId id) const;

    [[nodiscard]] bool contains(
        MaterialId id) const noexcept;

    void remove(MaterialId id);

    // Returns the number of occupied slots, excluding removed materials.
    [[nodiscard]] std::size_t size() const noexcept;
    [[nodiscard]] bool empty() const noexcept;

private:
    // Slots are never erased or reused: every issued ID keeps one meaning for
    // the lifetime of this registry.
    std::vector<std::optional<Material>> materials_;
    std::size_t size_{0};
};

} // namespace gy::physics::material
