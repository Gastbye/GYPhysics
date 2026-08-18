/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "gy/physics/rigid/RigidBody.h"
#include "gy/physics/rigid/RigidBodyStruct.h"

namespace gy::physics::rigid {

class RigidBodyRegistry
{
public:
    [[nodiscard]] BodyId add(RigidBody body);

    [[nodiscard]] RigidBody& get(BodyId id);

    [[nodiscard]] const RigidBody&
    get(BodyId id) const;

    [[nodiscard]] bool contains(
        BodyId id) const noexcept;

    // Returns true only when the body exists and is active.
    [[nodiscard]] bool isActive(
        BodyId id) const noexcept;

    // An inactive body remains registered and can still be accessed.
    void setActive(BodyId id, bool active);

    void remove(BodyId id);

    // Number of registered, non-removed bodies.
    [[nodiscard]] std::size_t size() const noexcept;

    // Number of active bodies.
    [[nodiscard]] std::size_t activeCount() const noexcept;

    [[nodiscard]] bool empty() const noexcept;

    /*
     * Do not add or remove bodies while executing the callback.
     * References passed to the callback must not be retained across
     * modifications of the registry.
     */
    template<typename Function>
    void forEachActive(Function&& function)
    {
        for (std::size_t index = 0;
             index < bodies_.size();
             ++index) {
            if (!bodies_[index].has_value()
                || !activeFlags_[index]) {
                continue;
            }

            function(
                BodyId{
                    static_cast<math::Index>(index)
                },
                bodies_[index].value()
            );
        }
    }

    template<typename Function>
    void forEachActive(Function&& function) const
    {
        for (std::size_t index = 0;
             index < bodies_.size();
             ++index) {
            if (!bodies_[index].has_value()
                || !activeFlags_[index]) {
                continue;
            }

            function(
                BodyId{
                    static_cast<math::Index>(index)
                },
                static_cast<const RigidBody&>(
                    bodies_[index].value()
                )
            );
        }
    }

private:
    [[nodiscard]] std::size_t checkedIndex(
        BodyId id) const;

    // An empty optional represents a removed slot.
    std::vector<std::optional<RigidBody>> bodies_;

    // true means that the existing body participates in simulation.
    // activeFlags_.size() must always equal bodies_.size().
    std::vector<bool> activeFlags_;

    std::size_t bodyCount_{0};
    std::size_t activeBodyCount_{0};
};

} // namespace gy::physics::rigid