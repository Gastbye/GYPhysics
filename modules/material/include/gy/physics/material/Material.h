/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <string>

#include "gy/physics/math/MathTypes.h"

namespace gy::physics::material {

class Material
{
public:
    explicit Material(std::string name);

    [[nodiscard]] const std::string& name() const noexcept;
    [[nodiscard]] math::Real density() const noexcept;
    void setDensity(math::Real density);

private:
    std::string name_;
    math::Real density_{math::kZero};
};

} // namespace gy::physics::material
