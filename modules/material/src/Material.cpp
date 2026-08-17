#include <cmath>
#include <stdexcept>
#include <utility>

#include "gy/physics/material/Material.h"

namespace gy::physics::material {

Material::Material(std::string name)
    : name_(std::move(name))
{
}

const std::string& Material::name() const noexcept
{
    return name_;
}

math::Real Material::density() const noexcept
{
    return density_;
}

void Material::setDensity(math::Real density)
{
    if (!std::isfinite(density) || density <= math::kZero) {
        throw std::invalid_argument(
            "Material density must be finite and greater than zero"
        );
    }
    density_ = density;
}

} // namespace gy::physics::material
