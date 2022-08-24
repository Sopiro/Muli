#include "spe/util.h"
#include "spe/box.h"
#include "spe/circle.h"
#include "spe/aabb.h"

namespace spe
{

float spe::calculate_convex_polygon_inertia(const std::vector<glm::vec2>& vertices, float mass, float area)
{
    float inertia = 0;

    size_t count = vertices.size();

    if (area <= 0)
    {
        area = 0;

        for (int i = 0; i < count; i++)
        {
            auto v1 = vertices[i];
            auto v2 = vertices[(i + 1) % count];
            area += glm::abs(glm::cross(v1, v2));
        }

        area *= 0.5;
    }

    for (int i = 0; i < count; i++)
    {
        auto v1 = vertices[i];
        auto v2 = vertices[(i + 1) % count];
        float l1 = glm::length(v1);
        float l2 = glm::length(v2);
        auto beta = glm::acos(glm::dot(v1, v2) / (l1 * l2)) / 2;

        float partialMass = (glm::abs(glm::cross(v1, v2)) / 2.0f) / area * mass;

        inertia += 0.5f * partialMass * l1 * l2 * (1 - 2.0f / 3.0f * glm::sin(beta) * glm::sin(beta));
    }

    return inertia;
}

}