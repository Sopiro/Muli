#include "spe/util.h"
#include "spe/aabb.h"
#include "spe/box.h"
#include "spe/circle.h"

namespace spe
{

float ComputePolygonInertia(const std::vector<Vec2>& vertices, float mass)
{
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (uint32_t i = 0; i < vertices.size(); i++)
    {
        uint32_t j = (i + 1) % vertices.size();

        const Vec2& vi = vertices[i];
        const Vec2& vj = vertices[j];

        float crs = Abs(Cross(vj, vi));

        numerator += crs * (Dot(vj, vj) + Dot(vj, vi) + Dot(vi, vi));
        denominator += crs;
    }

    return (numerator * mass) / (denominator * 6.0f);
}

} // namespace spe