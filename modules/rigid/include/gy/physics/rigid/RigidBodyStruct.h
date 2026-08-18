/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include "gy/physics/math/MathTypes.h"

namespace gy::physics::rigid {

struct BodyId
{
    math::Index value{math::InvalidIndex};

    constexpr BodyId() noexcept = default;

    explicit constexpr BodyId(math::Index index) noexcept
        : value(index)
    {
    }

    [[nodiscard]] constexpr bool isValid() const noexcept
    {
        return math::isValidIndex(value);
    }

    friend constexpr bool operator==(
        BodyId lhs,
        BodyId rhs) noexcept
    {
        return lhs.value == rhs.value;
    }

    friend constexpr bool operator!=(
        BodyId lhs,
        BodyId rhs) noexcept
    {
        return !(lhs == rhs);
    }
};

} // namespace gy::physics::rigid
