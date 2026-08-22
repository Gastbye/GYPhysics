#include "gy/physics/rigid/RigidBodyActor.h"

#include <utility>

namespace gy::physics::rigid {

BodyId RigidBodyActor::getBodyId() const noexcept
{
    return bodyId_;
}

const std::vector<RigidBodyShapeAttachment>&
RigidBodyActor::getAttachments() const noexcept
{
    return attachments_;
}

void RigidBodyActor::addShapeAttachment(RigidBodyShapeAttachment attachment)
{
    attachments_.push_back(std::move(attachment));
}

void RigidBodyActor::setBodyId(const BodyId& bodyId)
{
    bodyId_ = bodyId;
}

void RigidBodyActor::clearAttachments()
{
    attachments_.clear();
}

std::size_t RigidBodyActor::attachmentCount() const noexcept
{
    return attachments_.size();
}

} // namespace gy::physics::rigid
