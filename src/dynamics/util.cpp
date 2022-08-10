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

spe::Polygon* spe::create_random_convex_body(float radius, uint32_t num_vertices, float density)
{
    if (num_vertices < 3)
        num_vertices = glm::linearRand<uint32_t>(3, 8);

    std::vector<float> angles{};
    angles.reserve(num_vertices);

    for (size_t i = 0; i < num_vertices; i++)
    {
        angles.push_back(glm::linearRand<float>(0.0f, 1.0f) * glm::pi<float>() * 2.0f);
    }

    std::sort(angles.begin(), angles.end());

    std::vector<glm::vec2> vertices{};
    vertices.reserve(num_vertices);

    for (size_t i = 0; i < num_vertices; i++)
    {
        vertices.emplace_back(glm::cos(angles[i]) * radius, glm::sin(angles[i]) * radius);
    }

    return new Polygon(vertices, Dynamic, true, density);
}

spe::Polygon* spe::create_regular_polygon(float radius, uint32_t num_vertices, float initial_angle, float density)
{
    if (num_vertices < 3) num_vertices = glm::linearRand<uint32_t>(3, 11);

    float angleStart = initial_angle;
    float angle = glm::pi<float>() * 2.0f / num_vertices;

    if (num_vertices % 2 == 0)
        angleStart += angle / 2.0f;

    std::vector<glm::vec2> vertices;
    vertices.reserve(num_vertices);

    for (size_t i = 0; i < num_vertices; i++)
    {
        float currentAngle = angleStart + angle * i;

        glm::vec2 corner = glm::vec2{ glm::cos(currentAngle), glm::sin(currentAngle) };
        corner *= radius * glm::sqrt(2);

        vertices.push_back(corner);
    }

    return new Polygon(vertices, Dynamic, true, density);
}


std::vector<std::pair<RigidBody*, RigidBody*>> spe::get_collision_pair_n2(const std::vector<RigidBody*>& bodies)
{
    std::vector<std::pair<RigidBody*, RigidBody*>> pairs{};

    size_t numBodies = bodies.size();

    pairs.reserve(numBodies / 2 + 1);

    for (size_t i = 0; i < numBodies; i++)
    {
        RigidBody* a = bodies[i];
        for (size_t j = 0; j < numBodies; j++)
        {
            RigidBody* b = bodies[j];

            if (detect_collision_AABB(create_AABB(a), create_AABB(b)))
            {
                pairs.emplace_back(a, b);
            }
        }
    }

    return pairs;
}

}