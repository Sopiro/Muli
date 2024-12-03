#include "muli/polytope.h"

namespace muli
{

// Returns the edge closest to the Origin (0, 0)
PolytopeEdge Polytope::GetClosestEdge() const
{
    int32 minIndex = -1;
    float minDistance = max_value;
    Vec2 minNormal{ 0.0f };

    for (int32 i0 = vertices.Count() - 1, i1 = 0; i1 < vertices.Count(); i0 = i1, ++i1)
    {
        Vec2& v0 = vertices[i0];
        Vec2& v1 = vertices[i1];
        Vec2 edge = v1 - v0;

        Vec2 normal = Normalize(Cross(edge, 1.0f));
        float distance = Dot(normal, v0);

        if (distance < 0)
        {
            normal = -normal;
            distance = -distance;
        }

        if (distance < minDistance)
        {
            minDistance = distance;
            minNormal = normal;
            minIndex = i0;
        }
    }

    return PolytopeEdge{ minIndex, minDistance, minNormal };
}

} // namespace muli