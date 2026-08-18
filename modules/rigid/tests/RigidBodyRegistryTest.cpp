#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "gy/physics/rigid/RigidBodyRegistry.h"

namespace {

namespace math = gy::physics::math;
namespace rigid = gy::physics::rigid;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template<typename Exception, typename Function>
void requireThrows(Function&& function, const std::string& message)
{
    try {
        std::forward<Function>(function)();
    } catch (const Exception&) {
        return;
    } catch (...) {
        throw std::runtime_error(message + " (wrong exception type)");
    }

    throw std::runtime_error(message + " (no exception)");
}

[[nodiscard]] rigid::RigidBody makeBody(
    math::Real mass,
    const math::Vector3& position)
{
    rigid::RigidBodyDesc desc;
    desc.massProperties.mass = mass;
    desc.initialState.position = position;
    return rigid::RigidBody(desc);
}

void requireMissingIdOperationsThrow(
    rigid::RigidBodyRegistry& registry,
    rigid::BodyId id,
    const std::string& description)
{
    require(!registry.contains(id),
            description + " must not be contained.");
    requireThrows<std::out_of_range>(
        [&registry, id] { static_cast<void>(registry.get(id)); },
        description + " get() must throw."
    );
    const rigid::RigidBodyRegistry& constRegistry = registry;
    requireThrows<std::out_of_range>(
        [&constRegistry, id] {
            static_cast<void>(constRegistry.get(id));
        },
        description + " const get() must throw."
    );
    requireThrows<std::out_of_range>(
        [&constRegistry, id] {
            static_cast<void>(constRegistry.isActive(id));
        },
        description + " isActive() must throw."
    );
    requireThrows<std::out_of_range>(
        [&registry, id] { registry.setActive(id, false); },
        description + " setActive() must throw."
    );
    requireThrows<std::out_of_range>(
        [&registry, id] { registry.remove(id); },
        description + " remove() must throw."
    );
}

void testDefaultStateAndInvalidAccess()
{
    static_assert(!std::is_convertible_v<math::Index, rigid::BodyId>);
    static_assert(noexcept(
        std::declval<const rigid::RigidBodyRegistry&>().contains(
            rigid::BodyId{}
        )
    ));
    static_assert(!noexcept(
        std::declval<const rigid::RigidBodyRegistry&>().isActive(
            rigid::BodyId{}
        )
    ));

    rigid::RigidBodyRegistry registry;
    const rigid::BodyId invalidId;
    const rigid::BodyId outOfRangeId{math::Index{1234}};

    require(!invalidId.isValid(), "A default BodyId must be invalid.");
    require(invalidId.value == math::InvalidIndex,
            "A default BodyId must use InvalidIndex.");
    require(registry.empty(), "A new registry must be empty.");
    require(registry.size() == std::size_t{0},
            "A new registry must have size zero.");
    require(registry.activeCount() == std::size_t{0},
            "A new registry must have no active bodies.");
    requireMissingIdOperationsThrow(
        registry, invalidId, "The default BodyId");
    requireMissingIdOperationsThrow(
        registry, outOfRangeId, "An out-of-range BodyId");
}

void testAddLookupAndStableIds()
{
    rigid::RigidBodyRegistry registry;
    const rigid::BodyId firstId = registry.add(makeBody(
        math::Real{2},
        math::Vector3(math::Real{1}, math::Real{2}, math::Real{3})
    ));
    const rigid::BodyId secondId = registry.add(makeBody(
        math::Real{4},
        math::Vector3(math::Real{4}, math::Real{5}, math::Real{6})
    ));

    require(firstId.isValid() && secondId.isValid(),
            "Added bodies must receive valid IDs.");
    require(firstId != secondId, "Distinct bodies must have distinct IDs.");
    require(firstId.value == math::Index{0},
            "The first body must occupy slot zero.");
    require(secondId.value == math::Index{1},
            "The second body must occupy slot one.");
    require(registry.size() == std::size_t{2},
            "Adding two bodies must produce size two.");
    require(registry.activeCount() == std::size_t{2},
            "New bodies must be active by default.");
    require(!registry.empty(), "A populated registry cannot be empty.");
    require(registry.contains(firstId) && registry.contains(secondId),
            "Added bodies must be contained.");
    require(registry.isActive(firstId) && registry.isActive(secondId),
            "Added bodies must be active.");
    require(registry.get(firstId).mass() == math::Real{2},
            "Mutable lookup returned the wrong body.");

    rigid::RigidBodyState updatedState = registry.get(firstId).state();
    updatedState.linearVelocity = math::Vector3(
        math::Real{7}, math::Real{8}, math::Real{9}
    );
    registry.get(firstId).setState(updatedState);
    const rigid::RigidBodyRegistry& constRegistry = registry;
    require(constRegistry.get(firstId).state().linearVelocity.isApprox(
                math::Vector3(math::Real{7}, math::Real{8}, math::Real{9})),
            "Const lookup did not observe a mutable lookup change.");

    constexpr std::size_t additionalBodyCount = 128;
    for (std::size_t index = 0; index < additionalBodyCount; ++index) {
        static_cast<void>(registry.add(makeBody(
            math::Real{1},
            math::Vector3(
                static_cast<math::Real>(index),
                math::Real{0},
                math::Real{0}
            )
        )));
    }

    require(registry.get(firstId).mass() == math::Real{2},
            "Registry growth changed an existing BodyId meaning.");
    require(registry.get(secondId).mass() == math::Real{4},
            "Registry growth changed the second body.");
    require(registry.size() == additionalBodyCount + std::size_t{2},
            "Registry growth produced the wrong size.");
}

void testActivationAndIteration()
{
    rigid::RigidBodyRegistry registry;
    std::vector<rigid::BodyId> ids;
    for (math::Index index = 0; index < math::Index{4}; ++index) {
        ids.push_back(registry.add(makeBody(
            math::Real{1},
            math::Vector3(
                static_cast<math::Real>(index),
                math::Real{0},
                math::Real{0}
            )
        )));
    }

    registry.setActive(ids[1], false);
    registry.setActive(ids[3], false);
    registry.setActive(ids[1], false);
    require(registry.size() == std::size_t{4},
            "Deactivation must not remove a body.");
    require(registry.activeCount() == std::size_t{2},
            "Deactivation produced the wrong active count.");
    require(registry.contains(ids[1]) && !registry.isActive(ids[1]),
            "An inactive body must remain registered.");

    std::vector<rigid::BodyId> visitedIds;
    registry.forEachActive(
        [&visitedIds](rigid::BodyId id, rigid::RigidBody& body) {
            visitedIds.push_back(id);
            body.addForce(math::Vector3::UnitY());
        }
    );
    require(visitedIds.size() == std::size_t{2},
            "Mutable iteration visited the wrong number of bodies.");
    require(visitedIds[0] == ids[0] && visitedIds[1] == ids[2],
            "Mutable iteration visited inactive bodies or changed order.");
    require(registry.get(ids[0]).accumulatedForce().isApprox(
                math::Vector3::UnitY()),
            "Mutable iteration did not expose the stored body.");
    require(registry.get(ids[1]).accumulatedForce().isZero(),
            "Mutable iteration modified an inactive body.");

    const rigid::RigidBodyRegistry& constRegistry = registry;
    std::size_t constVisitCount = 0;
    constRegistry.forEachActive(
        [&constVisitCount](rigid::BodyId, const rigid::RigidBody&) {
            ++constVisitCount;
        }
    );
    require(constVisitCount == std::size_t{2},
            "Const iteration visited the wrong number of bodies.");

    registry.setActive(ids[1], true);
    registry.setActive(ids[1], true);
    require(registry.activeCount() == std::size_t{3},
            "Reactivation or its idempotence changed the count incorrectly.");
    require(registry.isActive(ids[1]),
            "A reactivated body must report as active.");

    std::size_t reactivatedVisitCount = 0;
    registry.forEachActive(
        [&reactivatedVisitCount, reactivatedId = ids[1]](
            rigid::BodyId id,
            rigid::RigidBody&) {
            if (id == reactivatedId) {
                ++reactivatedVisitCount;
            }
        }
    );
    require(reactivatedVisitCount == std::size_t{1},
            "A reactivated body did not rejoin active iteration.");
}

void testRemovalAndNoIdReuse()
{
    rigid::RigidBodyRegistry registry;
    const rigid::BodyId firstId = registry.add(makeBody(
        math::Real{1}, math::Vector3::Zero()));
    const rigid::BodyId removedActiveId = registry.add(makeBody(
        math::Real{2}, math::Vector3::Zero()));
    const rigid::BodyId removedInactiveId = registry.add(makeBody(
        math::Real{3}, math::Vector3::Zero()));
    const rigid::BodyId lastId = registry.add(makeBody(
        math::Real{4}, math::Vector3::Zero()));

    registry.setActive(removedInactiveId, false);
    registry.remove(removedActiveId);
    registry.remove(removedInactiveId);

    require(registry.size() == std::size_t{2},
            "Removing two bodies must reduce the registry size.");
    require(registry.activeCount() == std::size_t{2},
            "Removing active and inactive bodies changed the active count incorrectly.");
    require(registry.contains(firstId) && registry.contains(lastId),
            "Removing bodies invalidated an unrelated BodyId.");
    require(!registry.contains(removedActiveId)
                && !registry.contains(removedInactiveId),
            "Removed BodyIds must no longer be contained.");
    requireMissingIdOperationsThrow(
        registry, removedActiveId, "A removed active BodyId");
    requireMissingIdOperationsThrow(
        registry, removedInactiveId, "A removed inactive BodyId");

    const rigid::BodyId newId = registry.add(makeBody(
        math::Real{5}, math::Vector3::Zero()));
    require(newId.value == math::Index{4},
            "A new body must append instead of reusing a removed BodyId.");
    require(!registry.contains(removedActiveId),
            "Appending a body made an old BodyId valid again.");
    require(registry.contains(newId), "The appended body is unavailable.");

    std::vector<rigid::BodyId> visitedIds;
    registry.forEachActive(
        [&visitedIds](rigid::BodyId id, rigid::RigidBody&) {
            visitedIds.push_back(id);
        }
    );
    require(visitedIds.size() == std::size_t{3},
            "Iteration did not skip removed body slots.");
    require(visitedIds[0] == firstId
                && visitedIds[1] == lastId
                && visitedIds[2] == newId,
            "Iteration visited a removed body or changed ID order.");

    registry.remove(firstId);
    registry.remove(lastId);
    registry.remove(newId);
    require(registry.empty(), "Removing every body must empty the registry.");
    require(registry.size() == std::size_t{0},
            "An empty registry must report size zero.");
    require(registry.activeCount() == std::size_t{0},
            "An empty registry must report no active bodies.");
}

} // namespace

int main()
{
    try {
        testDefaultStateAndInvalidAccess();
        testAddLookupAndStableIds();
        testActivationAndIteration();
        testRemovalAndNoIdReuse();
    } catch (const std::exception& error) {
        std::cerr << "RigidBodyRegistry test failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "GyPhysics RigidBodyRegistry tests passed with Real="
              << math::realTypeName() << '\n';
    return 0;
}
