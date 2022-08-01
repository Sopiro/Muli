#pragma once

#include "common.h"

namespace spe
{
class Camera : public Entity
{
public:
    Camera();

    glm::mat4 CameraTransform() const;
};
}