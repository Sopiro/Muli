#include "muli/circle_shape.h"

namespace muli
{

bool CircleShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Vec2 position = transform * localPosition;

    Vec2 d = input.to - input.from;
    Vec2 f = input.from - position;
    float r2 = radius * radius;

    float a = Dot(d, d);
    float b = 2.0f * Dot(f, d);
    float c = Dot(f, f) - r2;

    // Quadratic equation discriminant
    float discriminant = b * b - 4 * a * c;

    if (discriminant < 0.0f)
    {
        return false;
    }

    discriminant = Sqrt(discriminant);

    float t = (-b - discriminant) / (2.0f * a);
    if (0.0f <= t && t <= input.maxFraction)
    {
        output->fraction = t;
        output->normal = (f + d * t).Normalized();

        return true;
    }

    return false;
}

} // namespace muli
