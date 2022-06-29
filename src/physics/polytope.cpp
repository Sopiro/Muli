#include "polytope.h"

using namespace spe;

Polytope::Polytope(const Simplex& simplex) :
    vertices{}
{
    vertices.reserve(3);
    vertices.push_back(simplex.vertices[0]);
    vertices.push_back(simplex.vertices[1]);
    vertices.push_back(simplex.vertices[2]);
}

ClosestEdgeInfo Polytope::GetClosestEdge()
{
    size_t minIndex = 0;
    float minDistance = std::numeric_limits<float>::max();
    glm::vec2 minNormal{};

    for (size_t i = 0; i < vertices.size(); i++)
    {
        size_t j = (i + 1) % vertices.size();

        glm::vec2& vertexI = vertices[i];
        glm::vec2& vertexJ = vertices[j];

        glm::vec2 edge = vertexJ - vertexI;

        glm::vec2 normal{ -edge.y, edge.x };
        normal = glm::normalize(normal);

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
