#include "util.h"
#include "common.h"
#include "muli/capsule.h"
#include "muli/circle.h"
#include "muli/polygon.h"
#include "options.h"

namespace muli
{

std::unique_ptr<Mesh> GenerateMesh(RigidBody* body, uint32 circlePolygonCount)
{
    RigidBody::Shape shape = body->GetShape();

    switch (shape)
    {
    case RigidBody::Shape::circle:
    {
        Circle* c = static_cast<Circle*>(body);
        float radius = c->GetRadius();

        float angle = MULI_PI * 2.0f / circlePolygonCount;

        std::vector<Vec3> vertices;
        std::vector<Vec2> texCoords;
        vertices.reserve(circlePolygonCount);

        for (uint32 i = 0; i < circlePolygonCount; ++i)
        {
            float currentAngle = angle * i;

            Vec3 corner = Vec3{ Cos(currentAngle), Sin(currentAngle), 0.0f };
            corner *= radius;

            vertices.push_back(corner);
            texCoords.emplace_back(corner.x, corner.y);
        }

        std::vector<uint32> indices = Triangulate(texCoords);

        vertices.push_back(Vec3(vertices[0]));
        vertices.push_back(Vec3{ 0.0f });

        return std::make_unique<Mesh>(vertices, texCoords, indices);
    }
    case RigidBody::Shape::polygon:
    {
        Polygon* p = static_cast<Polygon*>(body);
        float radius = p->GetRadius();

        const Vec2* vertices = p->GetVertices();
        const Vec2* normals = p->GetNormals();
        int32 vertexCount = p->GetVertexCount();

        std::vector<Vec2> vertices2;
        std::vector<Vec3> vertices3;
        vertices2.reserve(vertexCount * 2 + circlePolygonCount);
        vertices3.reserve(vertexCount * 2 + circlePolygonCount);

        for (int32 i0 = 0; i0 < vertexCount; ++i0)
        {
            int32 i1 = (i0 + 1) % vertexCount;
            int32 i2 = (i0 + 2) % vertexCount;

            const Vec2& v0 = vertices[i0];

            if (!(p->userFlag & UserFlag::RENDER_POLYGON_RADIUS))
            {
                vertices2.push_back(v0);
                vertices3.push_back(v0);
                continue;
            }

            const Vec2& v1 = vertices[i1];
            const Vec2& v2 = vertices[i2];

            Vec2 n0 = normals[i0];
            Vec2 n1 = normals[i1];

            Vec2 vn0 = v0 + n0 * radius;

            vertices2.push_back(vn0);
            vertices3.push_back(vn0);

            float theta = AngleBetween(n0, n1);
            float cCount = circlePolygonCount * theta / (2.0f * MULI_PI);

            for (int32 j = 0; j < cCount; ++j)
            {
                float per = j / cCount;
                Vec2 cv = v1 + Slerp(n0, n1, per) * radius;

                vertices2.push_back(cv);
                vertices3.push_back(cv);
            }
        }

        std::vector<uint32> indices = Triangulate(vertices2);

        return std::make_unique<Mesh>(vertices3, vertices2, indices);
    }
    case RigidBody::Shape::capsule:
    {
        Capsule* c = static_cast<Capsule*>(body);

        float l = c->GetLength();
        float r = c->GetRadius();

        std::vector<Vec3> vertices;
        std::vector<Vec2> texCoords;
        vertices.reserve(4 + circlePolygonCount);

        // Start from bottom right
        float angle = MULI_PI * 2.0f / circlePolygonCount;
        for (uint32_t i = 0; i < circlePolygonCount / 2.0f; ++i)
        {
            float currentAngle = -MULI_PI / 2.0f + angle * i;

            Vec3 corner = Vec3{ Cos(currentAngle), Sin(currentAngle), 0.0f };
            corner *= r;
            corner.x += l / 2.0f;

            vertices.push_back(corner);
            texCoords.emplace_back(corner.x, corner.y);
        }

        vertices.emplace_back(l / 2.0f, r, 0.0f);
        texCoords.emplace_back(l / 2.0f, r);

        vertices.emplace_back(-l / 2.0f, r, 0.0f);
        texCoords.emplace_back(-l / 2.0f, r);

        // Left top to left bottom
        for (uint32_t i = 0; i < circlePolygonCount / 2.0f; ++i)
        {
            float currentAngle = MULI_PI / 2.0f + angle * i;

            Vec3 corner = Vec3{ Cos(currentAngle), Sin(currentAngle), 0.0f };
            corner *= r;
            corner.x -= l / 2.0f;

            vertices.push_back(corner);
            texCoords.emplace_back(corner.x, corner.y);
        }

        vertices.emplace_back(-l / 2.0f, -r, 0.0f);
        texCoords.emplace_back(-l / 2.0f, -r);

        std::vector<uint32_t> indices = Triangulate(texCoords);

        return std::make_unique<Mesh>(vertices, texCoords, indices);
    }

    default:
        throw std::runtime_error("Not a supported shape");
    }
}

inline static uint32 get_next(std::unordered_set<uint32>& done, uint32 i, uint32 count, uint32 step = 1)
{
    uint32 i1 = (i + step) % count;

    while (done.find(i1) != done.end())
    {
        i1 = (i1 + 1) % count;
    }

    return i1;
}

std::vector<uint32> Triangulate(const std::vector<Vec2>& vertices)
{
    std::vector<uint32> indices;

    uint32 count = static_cast<uint32>(vertices.size());

    indices.reserve((count - 2) * 3);

    std::unordered_set<uint32> done;

    uint32 prev = count;
    uint32 i0 = 0;

    while (done.size() < count - 2)
    {
        uint32 i1 = get_next(done, i0, count);
        uint32 i2 = get_next(done, i1, count);

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

} // namespace muli