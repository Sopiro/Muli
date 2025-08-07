#include "builder.h"

namespace muli
{

RigidBody* CreateRandomConvexPolygon(
    World* world, float length, int32 vertexCount, const Transform& tf, RigidBody::Type type, float radius, float density
)
{
    if (vertexCount < 3)
    {
        vertexCount = (int32)Rand(6, 12);
    }

    std::vector<float> angles;
    angles.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; ++i)
    {
        angles.push_back(Rand(0.0f, 1.0f) * (pi * 2.0f - epsilon));
    }

    std::sort(angles.begin(), angles.end());

    std::vector<Vec2> vertices;
    vertices.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; ++i)
    {
        vertices.emplace_back(Cos(angles[i]) * length, Sin(angles[i]) * length);
    }

    RigidBody* b = world->CreateEmptyBody(tf, type);

    Polygon polygon{ vertices.data(), vertexCount, true, radius };
    b->CreateCollider(&polygon, identity, density);

    return b;
}

RigidBody* CreateRegularPolygon(
    World* world,
    float length,
    int32 vertexCount,
    float initialAngle,
    const Transform& tf,
    RigidBody::Type type,
    float radius,
    float density
)
{
    if (vertexCount < 3)
    {
        vertexCount = (int32)Rand(3, 12);
    }

    float angleStart = initialAngle - pi / 2.0f;
    float angle = pi * 2.0f / vertexCount;

    std::vector<Vec2> vertices;
    vertices.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; ++i)
    {
        float currentAngle = angleStart + angle * i;

        Vec2 vertex{ Cos(currentAngle), Sin(currentAngle) };
        vertex *= length * Sqrt(2.0f);

        vertices.push_back(vertex);
    }

    RigidBody* b = world->CreateEmptyBody(tf, type);

    Polygon polygon{ vertices.data(), vertexCount, true, radius };
    b->CreateCollider(&polygon, identity, density);

    return b;
}

} // namespace muli
