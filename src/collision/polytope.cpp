#include "spe/polytope.h"

namespace spe
{

ClosestEdgeInfo Polytope::GetClosestEdge() const
{
    uint32_t minIndex = 0;
    float minDistance = FLT_MAX;
    glm::vec2 minNormal{ 0.0f };

    for (uint32_t i = 0; i < vertices.Count(); i++)
    {
        uint32_t j = (i + 1) % vertices.Count();

        const glm::vec2& vertexI = vertices[i];
        const glm::vec2& vertexJ = vertices[j];

        const glm::vec2 edge = vertexJ - vertexI;

        glm::vec2 normal = glm::normalize(glm::vec2{ -edge.y, edge.x });
        float distance = glm::dot(normal, vertexI);

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

    return { minIndex, minDistance, minNormal };
}

} // namespace spe