#include "muli/circle.h"

namespace muli
{

Circle::Circle(float _radius, const Vec2& _center)
    : Shape(circle, _radius)
{
    area = radius * radius * pi;
    center = _center;
}

Vec2 Circle::GetClosestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 position = Mul(transform, center);
    Vec2 dir = (q - position);

    float distance = dir.Normalize();
    if (distance <= radius)
    {
        return q;
    }
    else
    {
        return position + dir * radius;
    }
}

bool Circle::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Vec2 position = Mul(transform, center);

    Vec2 d = input.to - input.from;
    Vec2 f = input.from - position;
    float r2 = radius * radius;

    float a = Dot(d, d);
    float b = 2.0f * Dot(f, d);
    float c = Dot(f, f) - r2;

    // Quadratic equation discriminant
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f)
    {
        return false;
    }

    discriminant = Sqrt(discriminant);

    float t = (-b - discriminant) / (2.0f * a);
    if (0.0f <= t && t <= input.maxFraction)
    {
        output->fraction = t;
        output->normal = Normalize(f + d * t);

        return true;
    }

    return false;
}

} // namespace muli
