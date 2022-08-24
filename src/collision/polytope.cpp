#include "spe/polytope.h"

namespace spe
{

Polytope::Polytope(const Simplex& simplex)
{
    vertices.reserve(5);
    vertices.push_back(simplex.vertices[0]);
    vertices.push_back(simplex.vertices[1]);
    vertices.push_back(simplex.vertices[2]);
}

ClosestEdgeInfo Polytope::GetClosestEdge() const
{
    size_t minIndex = 0;
    float minDistance = FLT_MAX;
    glm::vec2 minNormal{ 0.0f };

    for (size_t i = 0; i < vertices.size(); i++)
    {
        size_t j = (i + 1) % vertices.size();

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

}