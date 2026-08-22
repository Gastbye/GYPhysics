/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <cstdint>

namespace gy::physics::geometry {

enum class ShapeType : std::uint8_t
{
    None = 0,
    Sphere = 1,
    TriangleMesh = 2, TriMesh = TriangleMesh,
    Box = 3,
};

struct ShapeExtend
{
    ShapeExtend() = default;
    ShapeExtend(bool bounded) : isBounded(bounded) {}

    bool isBounded = true;
};

class Shape
{
public:
    virtual ~Shape() = default;

    [[nodiscard]] ShapeType type() const noexcept
    {
        return type_;
    }

    [[nodiscard]] ShapeExtend extend() const noexcept
    {
        return extend_;
    }

    [[nodiscard]] bool isBounded() const noexcept
    {
        return extend_.isBounded;
    }

protected:
    explicit Shape(ShapeType type, ShapeExtend extend)
        : type_(type), extend_(extend)
    {
    }

private:
    ShapeType type_ = ShapeType::None;
    ShapeExtend extend_;
};

} // namespace gy::physics::geometry
