#pragma once

#include "common.h"
#include "entity.h"

namespace spe
{
    class Camera : public Entity
    {
    public:
        Camera();

        glm::mat4 CameraTransform();

    private:
    };
}