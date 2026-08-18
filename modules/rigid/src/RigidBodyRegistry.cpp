#include "gy/physics/rigid/RigidBodyRegistry.h"

#include <stdexcept>
#include <utility>

namespace gy::physics::rigid {

BodyId RigidBodyRegistry::add(RigidBody body)
{
    if (bodies_.size()
        >= static_cast<std::size_t>(
            math::InvalidIndex
        )) {
        throw std::overflow_error(
            "RigidBodyRegistry exhausted all BodyId values"
        );
    }

    const BodyId id{
        static_cast<math::Index>(bodies_.size())
    };

    /*
     * Insert the body first. If inserting the active flag fails,
     * remove the newly inserted body to preserve the invariant
     * activeFlags_.size() == bodies_.size().
     */
    bodies_.emplace_back(std::move(body));

    try {
        activeFlags_.push_back(true);
    }
    catch (...) {
        bodies_.pop_back();
        throw;
    }

    ++bodyCount_;
    ++activeBodyCount_;

    return id;
}

RigidBody& RigidBodyRegistry::get(BodyId id)
{
    return bodies_[checkedIndex(id)].value();
}

const RigidBody& RigidBodyRegistry::get(
    BodyId id) const
{
    return bodies_[checkedIndex(id)].value();
}

bool RigidBodyRegistry::contains(
    BodyId id) const noexcept
{
    if (!id.isValid()) {
        return false;
    }

    const std::size_t index =
        static_cast<std::size_t>(id.value);

    return index < bodies_.size()
        && bodies_[index].has_value();
}

bool RigidBodyRegistry::isActive(
    BodyId id) const noexcept
{
    if (!contains(id)) {
        return false;
    }

    const std::size_t index =
        static_cast<std::size_t>(id.value);

    return activeFlags_[index];
}

void RigidBodyRegistry::setActive(
    BodyId id,
    bool active)
{
    const std::size_t index = checkedIndex(id);
    const bool wasActive = activeFlags_[index];

    if (wasActive == active) {
        return;
    }

    activeFlags_[index] = active;

    if (active) {
        ++activeBodyCount_;
    }
    else {
        --activeBodyCount_;
    }
}

void RigidBodyRegistry::remove(BodyId id)
{
    const std::size_t index = checkedIndex(id);

    if (activeFlags_[index]) {
        activeFlags_[index] = false;
        --activeBodyCount_;
    }

    bodies_[index].reset();
    --bodyCount_;
}

std::size_t RigidBodyRegistry::size() const noexcept
{
    return bodyCount_;
}

std::size_t
RigidBodyRegistry::activeCount() const noexcept
{
    return activeBodyCount_;
}

bool RigidBodyRegistry::empty() const noexcept
{
    return bodyCount_ == 0;
}

std::size_t RigidBodyRegistry::checkedIndex(
    BodyId id) const
{
    if (!contains(id)) {
        throw std::out_of_range(
            "RigidBodyRegistry contains no body "
            "for the given BodyId"
        );
    }

    return static_cast<std::size_t>(id.value);
}

} // namespace gy::physics::rigid