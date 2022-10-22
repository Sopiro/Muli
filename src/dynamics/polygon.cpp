#include "muli/polygon.h"

namespace muli
{

Polygon::Polygon(const Vec2* _vertices, int32 _vertexCount, Type _type, bool _resetCenter, float _radius, float _density)
    : RigidBody(_type, RigidBody::Shape::ShapePolygon)
{
    radius = _radius;
    vertexCount = _vertexCount;

    if (vertexCount > MAX_LOCAL_POLYGON_VERTICES)
    {
        vertices = (Vec2*)malloc(vertexCount * sizeof(Vec2));
    }
    else
    {
        vertices = localVertices;
    }

    ComputeConvexHull(_vertices, vertexCount, vertices);

    Vec2 centerOfMass{ 0.0f };
    for (int32 i0 = 0; i0 < vertexCount; i0++)
    {
        int32 i1 = (i0 + 1) % vertexCount;

        centerOfMass += vertices[i0];
    }
    centerOfMass *= 1.0f / vertexCount;
    vertices[0] -= centerOfMass;

    area = 0;
    for (int32 i = 1; i < vertexCount; i++)
    {
        vertices[i] -= centerOfMass;
        area += Cross(vertices[i - 1], vertices[i]);
    }
    area += Cross(vertices[vertexCount - 1], vertices[0]);
    area = Abs(area) / 2.0f;

    if (type == Dynamic)
    {
        muliAssert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = ComputePolygonInertia(vertices, vertexCount, mass);
        invInertia = 1.0f / inertia;
    }

    if (!_resetCenter)
    {
        Translate(centerOfMass);
    }
}

Polygon::~Polygon()
{
    if (vertexCount > MAX_LOCAL_POLYGON_VERTICES)
    {
        free(vertices);
    }
}

void Polygon::SetMass(float _mass)
{
    muliAssert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = ComputePolygonInertia(vertices, vertexCount, mass);
    invInertia = 1.0f / inertia;
}

void Polygon::SetDensity(float _density)
{
    muliAssert(_density > 0);

    density = _density;
    mass = _density * area;
    invMass = 1.0f / mass;
    inertia = ComputePolygonInertia(vertices, vertexCount, mass);
    invInertia = 1.0f / inertia;
}

AABB Polygon::GetAABB() const
{
    Vec2 min = transform * vertices[0];
    Vec2 max = min;

    for (int32 i = 1; i < vertexCount; i++)
    {
        Vec2 v = transform * vertices[i];

        min = Min(min, v);
        max = Max(max, v);
    }

    min -= radius;
    max += radius;

    return AABB{ min, max };
}

ContactPoint Polygon::Support(const Vec2& localDir) const
{
    int32 index = 0;
    float maxValue = Dot(localDir, vertices[index]);

    for (int32 i = 1; i < vertexCount; i++)
    {
        float value = Dot(localDir, vertices[i]);
        if (value > maxValue)
        {
            index = i;
            maxValue = value;
        }
    }

    return ContactPoint{ vertices[index], index };
}

Edge Polygon::GetFeaturedEdge(const Vec2& dir) const
{
    Vec2 localDir = MulT(transform.rotation, dir);
    ContactPoint farthest = Support(localDir);

    Vec2 curr = farthest.position;
    int32 index = farthest.id;
    int32 prevIndex = (index - 1 + vertexCount) % vertexCount;
    int32 nextIndex = (index + 1) % vertexCount;

    Vec2 prev = vertices[prevIndex];
    Vec2 next = vertices[nextIndex];

    Vec2 e1 = (curr - prev).Normalized();
    Vec2 e2 = (curr - next).Normalized();

    bool w = Dot(e1, localDir) <= Dot(e2, localDir);
    if (w)
    {
        return Edge{ transform * prev, transform * curr, prevIndex, index };
    }
    else
    {
        return Edge{ transform * curr, transform * next, index, nextIndex };
    }
}

} // namespace muli