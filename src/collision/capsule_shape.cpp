#include "muli/capsule_shape.h"

namespace muli
{

bool CapsuleShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Vec2 p1 = MulT(transform, input.from);
    Vec2 p2 = MulT(transform, input.to);

    Vec2 v1 = va;
    Vec2 v2 = vb;

    if (SignedDistanceToLineSegment(p1, v1, v2, radius) <= 0.0f)
    {
        return false;
    }

    Vec2 normal = Cross(1.0f, (v2 - v1)).Normalized();

    // Signed distance along noraml
    float distance = Dot(p1, normal);
    if (Abs(distance) <= radius)
    {
        Vec2 v = p1.x < 0.0f ? va : vb;

        Vec2 d = p2 - p1;
        Vec2 f = p1 - v;

        float a = Dot(d, d);
        float b = 2.0f * Dot(f, d);
        float c = Dot(f, f) - radius * radius;

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
            output->normal = transform.rotation * (f + d * t).Normalized();
            return true;
        }
        else
        {
            return false;
        }
    }

    // Translate edge along the normal
    if (distance > 0.0f)
    {
        v1 += normal * radius;
        v2 += normal * radius;
    }
    else
    {
        v1 -= normal * radius;
        v2 -= normal * radius;
    }

    // Ray casting to a line segment
    Vec2 d = p2 - p1;
    Vec2 e = v2 - v1;

    float denominator = Dot(normal, d);

    // Parallel or collinear case
    if (denominator == 0.0f)
    {
        return false;
    }

    float numerator = Dot(normal, v1 - p1);

    float t = numerator / denominator;
    if (t < 0.0f || input.maxFraction < t)
    {
        return false;
    }

    // Point on the v1-v2 line
    Vec2 q = p1 + t * d;

    float l2 = Dot(e, e);
    muliAssert(l2 > 0.0f);

    float u = Dot(q - v1, e) / l2;
    if (u < 0.0f)
    {
        // Ray cast to va circle
        Vec2 f = p1 - va;

        float a = Dot(d, d);
        float b = 2.0f * Dot(f, d);
        float c = Dot(f, f) - radius * radius;

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
            output->normal = transform.rotation * (f + d * t).Normalized();
            return true;
        }
        else
        {
            return false;
        }
    }
    else if (u > 1.0f)
    {
        // Ray cast to vb circle
        Vec2 f = p1 - vb;

        float a = Dot(d, d);
        float b = 2.0f * Dot(f, d);
        float c = Dot(f, f) - radius * radius;

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
            output->normal = transform.rotation * (f + d * t).Normalized();
            return true;
        }
        else
        {
            return false;
        }
    }
    else // Inside the edge region
    {
        output->fraction = t;
        if (numerator > 0.0f)
        {
            output->normal = transform.rotation * -normal;
        }
        else
        {
            output->normal = transform.rotation * normal;
        }

        return true;
    }
}

} // namespace muli
