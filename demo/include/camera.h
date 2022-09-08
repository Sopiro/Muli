#pragma once

#include "common.h"

namespace spe
{

struct Camera
{
    glm::vec2 position;
    float rotation;
    glm::vec2 scale;

    glm::mat4 GetCameraMatrix() const
    {
        return glm::translate(
            glm::rotate(glm::scale(glm::mat4{ 1.0f }, { 1.0f / scale.x, 1.0f / scale.y, 1.0f }), -rotation, { 0, 0, 1 }),
            glm::vec3{ -position, -1 });
    }
};

} // namespace spe