/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include <cstddef>
#include <vector>

#include "gy/physics/rigid/RigidBodyStruct.h"

namespace gy::physics::rigid {

class RigidBodyActor
{
public:
    explicit RigidBodyActor(BodyId bodyId)
        : bodyId_{bodyId}
    {
    }

    [[nodiscard]] BodyId getBodyId() const noexcept;
    [[nodiscard]] const std::vector<RigidBodyShapeAttachment>&
    getAttachments() const noexcept;

    void addShapeAttachment(RigidBodyShapeAttachment attachment);
    void setBodyId(const BodyId& bodyId);

    void clearAttachments();
    [[nodiscard]] std::size_t attachmentCount() const noexcept;

private:
    BodyId bodyId_;
    std::vector<RigidBodyShapeAttachment> attachments_;
};

} // namespace gy::physics::rigid
