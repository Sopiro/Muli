#include "muli/polytope.h"

namespace muli
{

PolytopeEdge Polytope::GetClosestEdge() const
{
    int32 minIndex = 0;
    float minDistance = FLT_MAX;
    Vec2 minNormal{ 0.0f };

    for (int32 i = 0; i < vertices.Count(); ++i)
    {
        int32 j = (i + 1) % vertices.Count();

        const Vec2& vertexI = vertices[i];
        const Vec2& vertexJ = vertices[j];

        Vec2 edge = vertexJ - vertexI;

        Vec2 normal = Cross(edge, 1.0f).Normalized();
        float distance = Dot(normal, vertexI);

        if (distance < 0)
        {
            distance *= -1;
            normal *= -1;
        }

        if (distance < minDistance)
        {
            minDistance = distance;
            minNormal = normal;
            minIndex = i;
        }
    }

    return PolytopeEdge{ minIndex, minDistance, minNormal };
}

} // namespace muli