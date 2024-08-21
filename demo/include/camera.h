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
        // Inverse scale
        Mat4 m{ Vec4(1 / scale.x, 1 / scale.y, 1, 1) };

        // Inverse rotation
        m = MulT(Mat4(Quat::FromEuler({ 0, 0, rotation }), Vec3::zero), m);

        // Inverse translation
        m = m.Translate({ -position.x, -position.y, 0 });

        return m;
    }
};

} // namespace muli