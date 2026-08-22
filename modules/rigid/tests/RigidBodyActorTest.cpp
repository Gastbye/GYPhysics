#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "gy/physics/rigid/RigidBodyActor.h"

namespace {

namespace geometry = gy::physics::geometry;
namespace material = gy::physics::material;
namespace math = gy::physics::math;
namespace rigid = gy::physics::rigid;

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

[[nodiscard]] rigid::RigidBodyShapeAttachment makeAttachment(
    math::Index shapeIndex,
    math::Index materialIndex,
    const math::Vector3& localPosition,
    const math::Quaternion& localOrientation)
{
    rigid::RigidBodyShapeAttachment attachment;
    attachment.shapeId = geometry::ShapeId{shapeIndex};
    attachment.materialId = material::MaterialId{materialIndex};
    attachment.localPosition = localPosition;
    attachment.localOrientation = localOrientation;
    return attachment;
}

void requireSameAttachment(
    const rigid::RigidBodyShapeAttachment& actual,
    const rigid::RigidBodyShapeAttachment& expected,
    const std::string& message)
{
    require(actual.shapeId == expected.shapeId,
            message + " ShapeId changed.");
    require(actual.materialId == expected.materialId,
            message + " MaterialId changed.");
    require((actual.localPosition.array()
             == expected.localPosition.array()).all(),
            message + " Local position changed.");
    require((actual.localOrientation.coeffs().array()
             == expected.localOrientation.coeffs().array()).all(),
            message + " Local orientation changed.");
}

void testConstructionAndEmptyAttachments()
{
    static_assert(!std::is_convertible_v<rigid::BodyId,
                                         rigid::RigidBodyActor>);
    static_assert(std::is_same_v<
        decltype(std::declval<const rigid::RigidBodyActor&>().getBodyId()),
        rigid::BodyId
    >);
    static_assert(std::is_same_v<
        decltype(
            std::declval<const rigid::RigidBodyActor&>().getAttachments()
        ),
        const std::vector<rigid::RigidBodyShapeAttachment>&
    >);
    static_assert(noexcept(
        std::declval<const rigid::RigidBodyActor&>().getBodyId()
    ));
    static_assert(noexcept(
        std::declval<const rigid::RigidBodyActor&>().getAttachments()
    ));
    static_assert(noexcept(
        std::declval<const rigid::RigidBodyActor&>().attachmentCount()
    ));

    const rigid::BodyId bodyId{math::Index{7}};
    const rigid::RigidBodyActor actor(bodyId);

    require(actor.getBodyId() == bodyId,
            "Construction did not preserve BodyId.");
    require(actor.attachmentCount() == std::size_t{0},
            "A new actor must have no attachments.");
    require(actor.getAttachments().empty(),
            "A new actor's attachment container must be empty.");
}

void testAddAttachmentsAndPreserveOrder()
{
    rigid::RigidBodyActor actor(rigid::BodyId{math::Index{2}});
    rigid::RigidBodyShapeAttachment first = makeAttachment(
        math::Index{3},
        math::Index{5},
        math::Vector3(math::Real{1}, math::Real{2}, math::Real{3}),
        math::Quaternion(
            math::Real{0.5},
            math::Real{0.5},
            math::Real{0.5},
            math::Real{0.5}
        )
    );
    const rigid::RigidBodyShapeAttachment expectedFirst = first;
    const rigid::RigidBodyShapeAttachment second = makeAttachment(
        math::Index{8},
        math::Index{13},
        math::Vector3(math::Real{-1}, math::Real{4}, math::Real{6}),
        math::Quaternion::Identity()
    );

    actor.addShapeAttachment(first);
    first.shapeId = geometry::ShapeId{math::Index{99}};
    first.localPosition.setZero();
    actor.addShapeAttachment(second);

    require(actor.attachmentCount() == std::size_t{2},
            "Adding attachments produced the wrong count.");
    const auto& attachments = actor.getAttachments();
    require(attachments.size() == std::size_t{2},
            "getAttachments returned the wrong container size.");
    requireSameAttachment(
        attachments[0], expectedFirst,
        "The first stored attachment"
    );
    requireSameAttachment(
        attachments[1], second,
        "The second stored attachment"
    );
}

void testSetBodyIdAndClearAttachments()
{
    const rigid::BodyId originalId{math::Index{1}};
    const rigid::BodyId replacementId{math::Index{42}};
    rigid::RigidBodyActor actor(originalId);
    actor.addShapeAttachment(makeAttachment(
        math::Index{4},
        math::Index{6},
        math::Vector3::UnitX(),
        math::Quaternion::Identity()
    ));

    actor.setBodyId(replacementId);
    require(actor.getBodyId() == replacementId,
            "setBodyId did not replace the BodyId.");
    require(actor.attachmentCount() == std::size_t{1},
            "setBodyId unexpectedly changed attachments.");

    actor.clearAttachments();
    require(actor.getAttachments().empty(),
            "clearAttachments did not empty the attachment container.");
    require(actor.attachmentCount() == std::size_t{0},
            "clearAttachments did not update attachmentCount.");
    require(actor.getBodyId() == replacementId,
            "clearAttachments unexpectedly changed BodyId.");

    actor.clearAttachments();
    require(actor.getAttachments().empty(),
            "Clearing an empty actor must remain safe and idempotent.");
}

void testActorCopyIsIndependent()
{
    rigid::RigidBodyActor original(rigid::BodyId{math::Index{10}});
    original.addShapeAttachment(makeAttachment(
        math::Index{11},
        math::Index{12},
        math::Vector3::UnitY(),
        math::Quaternion::Identity()
    ));

    rigid::RigidBodyActor copy = original;
    copy.setBodyId(rigid::BodyId{math::Index{20}});
    copy.clearAttachments();

    require(original.getBodyId() == rigid::BodyId{math::Index{10}},
            "Changing a copied actor changed the original BodyId.");
    require(original.attachmentCount() == std::size_t{1},
            "Clearing a copied actor changed the original attachments.");
    require(copy.getBodyId() == rigid::BodyId{math::Index{20}},
            "The copied actor did not retain its independent BodyId.");
    require(copy.getAttachments().empty(),
            "The copied actor did not retain an independent container.");
}

} // namespace

int main()
{
    try {
        testConstructionAndEmptyAttachments();
        testAddAttachmentsAndPreserveOrder();
        testSetBodyIdAndClearAttachments();
        testActorCopyIsIndependent();
    } catch (const std::exception& error) {
        std::cerr << "RigidBodyActor test failed: " << error.what() << '\n';
        return 1;
    }

    std::cout << "GyPhysics RigidBodyActor tests passed with Real="
              << math::realTypeName() << '\n';
    return 0;
}
