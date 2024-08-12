#include "util.h"
#include "common.h"
#include "options.h"

namespace muli
{

static inline int32 GetNext(std::unordered_set<int32>& done, int32 i, int32 count, int32 step = 1)
{
    int32 i1 = (i + step) % count;

    while (done.find(i1) != done.end())
    {
        i1 = (i1 + 1) % count;
    }

    return i1;
}

// Modified Ear-clipping algorithm.
// Only works for convex shape
static std::vector<int32> Triangulate(std::span<Vec2> vertices)
{
    std::vector<int32> indices;
    int32 count = int32(vertices.size());
    indices.reserve((count - 2) * 3);

    std::unordered_set<int32> done;

    int32 prev = count;
    int32 i0 = 0;

    while (done.size() < count - 2)
    {
        int32 i1 = GetNext(done, i0, count);
        int32 i2 = GetNext(done, i1, count);

        indices.push_back(i0);
        indices.push_back(i1);
        indices.push_back(i2);

        done.insert(i1);

        prev = i0;
        i0 = GetNext(done, i2, count, (count + 1) / 2u);
    }

    return indices;
}

std::unique_ptr<Mesh> GenerateMesh(const Collider* collider, int32 circlePolygonCount)
{
    const RigidBody* body = collider->GetBody();
    const Shape* shape = collider->GetShape();
    Shape::Type type = collider->GetType();

    switch (type)
    {
    case Shape::Type::circle:
    {
        const Circle* c = (const Circle*)shape;
        Vec2 localCenter = c->GetCenter();
        float radius = c->GetRadius();

        float angle = pi * 2.0f / circlePolygonCount;

        std::vector<Vec3> vertices;
        std::vector<Vec2> texCoords;
        vertices.reserve(circlePolygonCount);
        texCoords.reserve(circlePolygonCount);

        for (int32 i = 0; i < circlePolygonCount; ++i)
        {
            float currentAngle = angle * i;

            Vec3 vertex{ Cos(currentAngle), Sin(currentAngle), 0.0f };
            vertex *= radius;
            vertex.x += localCenter.x;
            vertex.y += localCenter.y;

            vertices.push_back(vertex);
            texCoords.emplace_back(vertex.x, vertex.y);
        }

        std::vector<int32> indices = Triangulate(texCoords);

        vertices.emplace_back(Vec3{ vertices[0] });
        vertices.emplace_back(Vec3{ localCenter });

        return std::make_unique<Mesh>(vertices, texCoords, indices);
    }
    case Shape::Type::polygon:
    {
        const Polygon* p = (const Polygon*)shape;
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

            const Vec2& v0 = vertices[i0];

            if (!UserFlag::IsEnabled(body, UserFlag::render_polygon_radius))
            {
                vertices2.push_back(v0);
                vertices3.push_back(v0);
                continue;
            }

            const Vec2& v1 = vertices[i1];

            Vec2 n0 = normals[i0];
            Vec2 n1 = normals[i1];

            Vec2 vn0 = v0 + n0 * radius;

            vertices2.push_back(vn0);
            vertices3.push_back(vn0);

            float theta = AngleBetween(n0, n1);
            float cCount = circlePolygonCount * theta / (2.0f * pi);

            for (int32 j = 0; j < cCount; ++j)
            {
                float per = j / cCount;
                Vec2 cv = v1 + Slerp(n0, n1, per) * radius;

                vertices2.push_back(cv);
                vertices3.push_back(cv);
            }
        }

        std::vector<int32> indices = Triangulate(vertices2);

        return std::make_unique<Mesh>(vertices3, vertices2, indices);
    }
    case Shape::Type::capsule:
    {
        const Capsule* c = (const Capsule*)shape;

        Vec2 v1 = c->GetVertexA();
        Vec2 v2 = c->GetVertexB();
        Vec2 normal = Normalize(Cross(1.0f, v2 - v1));
        float r = c->GetRadius();

        float angleOffset = AngleBetween(Vec2{ 0.0f, 1.0f }, normal);

        std::vector<Vec3> vertices;
        std::vector<Vec2> texCoords;

        vertices.emplace_back(v1 - normal * r);
        texCoords.emplace_back(v1 - normal * r);

        // Start from bottom right
        float angle = pi * 2.0f / circlePolygonCount;
        for (int32 i = 0; i < circlePolygonCount / 2.0f; ++i)
        {
            float currentAngle = angleOffset - pi / 2.0f + angle * i;

            Vec3 vertex{ Cos(currentAngle), Sin(currentAngle), 0.0f };
            vertex *= r;
            vertex += v2;

            vertices.push_back(vertex);
            texCoords.emplace_back(vertex.x, vertex.y);
        }

        vertices.emplace_back(v2 + normal * r);
        texCoords.emplace_back(v2 + normal * r);

        // Left top to left bottom
        for (int32 i = 0; i < circlePolygonCount / 2.0f; ++i)
        {
            float currentAngle = angleOffset + pi / 2.0f + angle * i;

            Vec3 vertex{ Cos(currentAngle), Sin(currentAngle), 0.0f };
            vertex *= r;
            vertex += v1;

            vertices.push_back(vertex);
            texCoords.emplace_back(vertex.x, vertex.y);
        }

        std::vector<int32> indices = Triangulate(texCoords);

        return std::make_unique<Mesh>(vertices, texCoords, indices);
    }

    default:
        throw std::runtime_error("Not a supported shape");
    }
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
    if (t < 0.0f)
        t += 1.0f;
    else if (t > 1.0f)
        t -= 1.0f;

    if (t < 1.0f / 6.0f)
        return p + (q - p) * 6.0f * t;
    else if (t < 1.0f / 2.0f)
        return q;
    else if (t < 2.0f / 3.0f)
        return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;

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

void PrintSizes()
{
    std::cout << "Circle: " << sizeof(Circle) << '\n';
    std::cout << "Capsule: " << sizeof(Capsule) << '\n';
    std::cout << "Polygon: " << sizeof(Polygon) << '\n';
    std::cout << "Collider: " << sizeof(Collider) << '\n';
    std::cout << "RigidBody: " << sizeof(RigidBody) << '\n';
    std::cout << "Contact: " << sizeof(Contact) << '\n';
    std::cout << "Manifold: " << sizeof(ContactManifold) << '\n';
    std::cout << '\n';

    std::cout << "Angle joint: " << sizeof(AngleJoint) << '\n';
    std::cout << "Distance joint: " << sizeof(DistanceJoint) << '\n';
    std::cout << "Grab joint: " << sizeof(GrabJoint) << '\n';
    std::cout << "Line joint: " << sizeof(LineJoint) << '\n';
    std::cout << "Motor joint: " << sizeof(MotorJoint) << '\n';
    std::cout << "Prismatic joint: " << sizeof(PrismaticJoint) << '\n';
    std::cout << "Pulley joint: " << sizeof(PulleyJoint) << '\n';
    std::cout << "Revolute joint: " << sizeof(RevoluteJoint) << '\n';
    std::cout << "Weld joint: " << sizeof(WeldJoint) << '\n' << std::endl;
}

} // namespace muli