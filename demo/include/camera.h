#pragma once

#include "common.h"

namespace muli
{

struct Camera
{
    Vec2 position;
    float rotation;
    Vec2 scale;

    Mat4 GetCameraMatrix() const
    {
        return Mat4{ 1.0f }
            .Scale(1.0f / scale.x, 1.0f / scale.y, 1.0f)
            .Rotate(0.0f, 0.0f, -rotation)
            .Translate(-position.x, -position.y, 1.0f);
    }
};

} // namespace muli