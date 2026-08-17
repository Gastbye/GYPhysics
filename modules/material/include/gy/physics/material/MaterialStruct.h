/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include "gy/physics/math/MathTypes.h"

namespace gy::physics::material {

struct MaterialId
{
    math::Index value{math::InvalidIndex};

    constexpr MaterialId() noexcept = default;

    explicit constexpr MaterialId(math::Index index) noexcept
        : value(index)
    {
    }

    [[nodiscard]] constexpr bool isValid() const noexcept
    {
        return math::isValidIndex(value);
    }

    friend constexpr bool operator==(
        MaterialId lhs,
        MaterialId rhs) noexcept
    {
        return lhs.value == rhs.value;
    }

    friend constexpr bool operator!=(
        MaterialId lhs,
        MaterialId rhs) noexcept
    {
        return !(lhs == rhs);
    }
};

} // namespace gy::physics::material
