#include "muli/util.h"
#include "muli/aabb.h"
#include "muli/box.h"
#include "muli/circle.h"

namespace muli
{

float ComputePolygonInertia(const std::vector<Vec2>& vertices, float mass)
{
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (uint32 i = 0; i < vertices.size(); i++)
    {
        uint32 j = (i + 1) % vertices.size();

        const Vec2& vi = vertices[i];
        const Vec2& vj = vertices[j];

        float crs = Abs(Cross(vj, vi));

        numerator += crs * (Dot(vj, vj) + Dot(vj, vi) + Dot(vi, vi));
        denominator += crs;
    }

    return (numerator * mass) / (denominator * 6.0f);
}

float ComputeCapsuleInertia(float length, float radius, float mass)
{
    float width = length;
    float height = radius * 2.0f;

    float rectArea = width * height;
    float circleArea = MULI_PI * radius * radius;
    float totalArea = rectArea + circleArea;

    float rectInertia = (width * width + height * height) / 12.0f;
    float halfCircleInertia = ((MULI_PI / 4) - 8.0f / (9.0f * MULI_PI)) * radius * radius * radius * radius;

    float dist2 = length * 0.5f + (4.0f * radius) / (MULI_PI * 3.0f);
    dist2 *= dist2;

    // Parallel axis theorem applied
    return mass * (rectInertia * rectArea + (halfCircleInertia + (circleArea * 0.5f) * dist2) * 2.0f) / (rectArea + circleArea);
}

} // namespace muli