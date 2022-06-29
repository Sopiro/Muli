#include "util.h"
#include "box.h"
#include "circle.h"

using namespace spe;

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

spe::Polygon spe::create_random_convex_body(float radius, uint32_t num_vertices, float density)
{
    if (num_vertices < 3)
        num_vertices = glm::linearRand<uint32_t>(3, 8);

    std::vector<float> angles;
    angles.reserve(num_vertices);

    for (size_t i = 0; i < num_vertices; i++)
    {
        angles.emplace_back(glm::linearRand<float>(0, 1) * glm::pi<float>() * 2.0f);
    }

    std::sort(angles.begin(), angles.end());

    std::vector<glm::vec2> vertices;
    vertices.reserve(num_vertices);

    for (size_t i = 0; i < num_vertices; i++)
    {
        glm::vec2 corner = glm::vec2{ glm::cos(angles[i]), glm::sin(angles[i]) };
        corner *= radius;

        vertices.push_back(corner);
    }

    return Polygon(vertices, Dynamic, true, density);
}

spe::Polygon spe::create_regular_polygon(size_t radius, uint32_t num_vertices, float initial_angle, float density)
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

    return Polygon(vertices, Dynamic, true, density);
}

Mesh spe::generate_mesh_from_rigidbody(RigidBody& body, uint32_t circle_polygon_count)
{
    auto& bodyType = typeid(body);

    if (bodyType == typeid(Circle))
    {
        Circle& c = static_cast<Circle&>(body);

        float radius = c.GetRadius();

        float angle = glm::pi<float>() * 2.0f / circle_polygon_count;

        std::vector<glm::vec3> vertices;
        std::vector<glm::vec2> texCoords;
        vertices.reserve(circle_polygon_count);

        for (size_t i = 0; i < circle_polygon_count; i++)
        {
            float currentAngle = angle * i;

            glm::vec3 corner = glm::vec3{ glm::cos(currentAngle), glm::sin(currentAngle), 0.0f };
            corner *= radius * glm::sqrt(2);

            vertices.push_back(corner);
            texCoords.emplace_back(corner.x, corner.y);
        }

        std::vector<uint32_t> indices = triangulate(texCoords);

        return Mesh{ vertices, texCoords, indices };
    }
    else if (bodyType == typeid(Polygon) || bodyType == typeid(Box))
    {
        Polygon& p = static_cast<Polygon&>(body);

        auto& vertices2 = p.GetVertices();

        std::vector<glm::vec3> vertices3;
        vertices3.reserve(vertices2.size());

        for (auto& v = vertices2.begin(); v != vertices2.end(); v++)
        {
            vertices3.emplace_back(v->x, v->y, 0.0f);
        }

        std::vector<uint32_t> indices = triangulate(vertices2);

        return Mesh{ vertices3, vertices2, indices };
    }
    else
    {
        throw std::exception("Not a supported shape");
    }
}

inline static uint32_t get_next(std::set<uint32_t>& done, uint32_t i, uint32_t count, uint32_t step = 1)
{
    uint32_t i1 = (i + step) % count;

    while (done.find(i1) != done.end())
    {
        i1 = (i1 + 1) % count;
    }

    return i1;
}

std::vector<uint32_t> spe::triangulate(const std::vector<glm::vec2>& vertices)
{
    std::vector<uint32_t> indices;

    uint32_t count = static_cast<uint32_t>(vertices.size());

    indices.reserve((count - 2) * 3);

    std::set<uint32_t> done;

    uint32_t prev = count;
    uint32_t i0 = 0;

    while (done.size() < count - 2)
    {
        uint32_t i1 = get_next(done, i0, count);
        uint32_t i2 = get_next(done, i1, count);

        indices.push_back(i0);
        indices.push_back(i1);
        indices.push_back(i2);

        done.insert(i1);

        prev = i0;
        i0 = get_next(done, i2, count, (count + 1) / 2u);
    }

    return indices;
}