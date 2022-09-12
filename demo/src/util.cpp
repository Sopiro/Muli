#include "util.h"
#include "common.h"
#include "spe/box.h"
#include "spe/circle.h"
#include "spe/polygon.h"

namespace spe
{

std::unique_ptr<Mesh> GenerateMesh(RigidBody& body, uint32_t circlePolygonCount)
{
    RigidBody::Shape shape = body.GetShape();

    if (shape == RigidBody::Shape::ShapeCircle)
    {
        Circle& c = static_cast<Circle&>(body);

        float radius = c.GetRadius();

        float angle = SPE_PI * 2.0f / circlePolygonCount;

        std::vector<Vec3> vertices;
        std::vector<Vec2> texCoords;
        vertices.reserve(circlePolygonCount);

        for (uint32_t i = 0; i < circlePolygonCount; i++)
        {
            float currentAngle = angle * i;

            Vec3 corner = Vec3{ Cos(currentAngle), Sin(currentAngle), 0.0f };
            corner *= radius;

            vertices.push_back(corner);
            texCoords.emplace_back(corner.x, corner.y);
        }

        std::vector<uint32_t> indices = Triangulate(texCoords);

        vertices.push_back(Vec3(vertices[0]));
        vertices.push_back(Vec3{ 0.0f });

        return std::make_unique<Mesh>(vertices, texCoords, indices);
    }
    else if (shape == RigidBody::Shape::ShapePolygon)
    {
        Polygon& p = static_cast<Polygon&>(body);

        auto& vertices2 = p.GetVertices();

        std::vector<Vec3> vertices3;
        vertices3.reserve(vertices2.size());

        for (auto& v = vertices2.begin(); v != vertices2.end(); v++)
        {
            vertices3.emplace_back(v->x, v->y, 0.0f);
        }

        std::vector<uint32_t> indices = Triangulate(vertices2);

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

std::vector<uint32_t> Triangulate(const std::vector<Vec2>& vertices)
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

Vec3 rgb2hsl(float r, float g, float b)
{
    r /= 255.0f;
    g /= 255.0f;
    b /= 255.0f;

    float max = Max(Max(r, g), b);
    float min = Min(Min(r, g), b);

    Vec3 res{ (max + min) / 2.0f };

    if (max == min)
    {
        // achromatic
        res.x = 0.0f;
        res.y = 0.0f;
    }
    else
    {
        float d = max - min;
        res.x = (res.z > 0.5f) ? d / (2.0f - max - min) : d / (max + min);

        if (max == r)
            res.x = (g - b) / d + (g < b ? 6 : 0);
        else if (max == g)
            res.x = (b - r) / d + 2;
        else if (max == b)
            res.x = (r - g) / d + 4;

        res.x /= 6;
    }

    return res;
}

float hue2rgb(float p, float q, float t)
{
    if (t < 0.0f) t += 1.0f;
    if (t > 1.0f) t -= 1.0f;
    if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
    if (t < 1.0f / 2.0f) return q;
    if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;

    return p;
}

Vec3 hsl2rgb(float h, float s, float l)
{
    Vec3 res;

    if (s == 0.0f)
    {
        res.x = res.y = res.z = l; // achromatic
    }
    else
    {
        float q = l < 0.5f ? l * (1.0f + s) : l + s - l * s;
        float p = 2.0f * l - q;
        res.x = hue2rgb(p, q, h + 1.0f / 3.0f);
        res.y = hue2rgb(p, q, h);
        res.z = hue2rgb(p, q, h - 1.0f / 3.0f);
    }

    return res;
}

} // namespace spe