#include "util.h"
#include "common.h"
#include "spe/box.h"
#include "spe/circle.h"
#include "spe/polygon.h"

namespace spe
{

std::unique_ptr<Mesh> generate_mesh_from_rigidbody(RigidBody& body, uint32_t circle_polygon_count)
{
    RigidBody::Shape shape = body.GetShape();

    if (shape == RigidBody::Shape::ShapeCircle)
    {
        Circle& c = static_cast<Circle&>(body);

        float radius = c.GetRadius();

        float angle = glm::pi<float>() * 2.0f / circle_polygon_count;

        std::vector<glm::vec3> vertices;
        std::vector<glm::vec2> texCoords;
        vertices.reserve(circle_polygon_count);

        for (uint32_t i = 0; i < circle_polygon_count; i++)
        {
            float currentAngle = angle * i;

            glm::vec3 corner = glm::vec3{ glm::cos(currentAngle), glm::sin(currentAngle), 0.0f };
            corner *= radius;

            vertices.push_back(corner);
            texCoords.emplace_back(corner.x, corner.y);
        }

        std::vector<uint32_t> indices = triangulate(texCoords);

        vertices.push_back(glm::vec3(vertices[0]));
        vertices.push_back(glm::vec3{ 0.0f });

        return std::make_unique<Mesh>(vertices, texCoords, indices);
    }
    else if (shape == RigidBody::Shape::ShapePolygon)
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

        return std::make_unique<Mesh>(vertices3, vertices2, indices);
    }
    else
    {
        throw std::exception("Not a supported shape");
    }
}

inline static uint32_t get_next(std::unordered_set<uint32_t>& done, uint32_t i, uint32_t count, uint32_t step = 1)
{
    uint32_t i1 = (i + step) % count;

    while (done.find(i1) != done.end())
    {
        i1 = (i1 + 1) % count;
    }

    return i1;
}

std::vector<uint32_t> triangulate(const std::vector<glm::vec2>& vertices)
{
    std::vector<uint32_t> indices;

    uint32_t count = static_cast<uint32_t>(vertices.size());

    indices.reserve((count - 2) * 3);

    std::unordered_set<uint32_t> done;

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

} // namespace spe