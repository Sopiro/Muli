#include "muli/polygon.h"

namespace muli
{

Polygon::Polygon(const std::vector<Vec2>& _vertices, Type _type, bool _resetCenter, float _radius, float _density)
    : RigidBody(_type, RigidBody::Shape::ShapePolygon)
    , vertices{ ComputeConvexHull(_vertices) }
{
    radius = _radius;

    Vec2 centerOfMass{ 0.0f };
    size_t vertexCount = vertices.size();
    normals.reserve(vertexCount);

    for (size_t i0 = 0; i0 < vertexCount; i0++)
    {
        size_t i1 = (i0 + 1) % vertexCount;

        centerOfMass += vertices[i0];
        normals.push_back(Cross(vertices[i1] - vertices[i0], 1.0f).Normalized());
    }
    centerOfMass *= 1.0f / vertexCount;
    vertices[0] -= centerOfMass;

    area = 0;
    for (size_t i = 1; i < vertexCount; i++)
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
        inertia = ComputePolygonInertia(vertices, mass);
        invInertia = 1.0f / inertia;
    }

    if (!_resetCenter)
    {
        Translate(centerOfMass);
    }
}

void Polygon::SetMass(float _mass)
{
    muliAssert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = ComputePolygonInertia(vertices, mass);
    invInertia = 1.0f / inertia;
}

void Polygon::SetDensity(float _density)
{
    muliAssert(_density > 0);

    density = _density;
    mass = _density * area;
    invMass = 1.0f / mass;
    inertia = ComputePolygonInertia(vertices, mass);
    invInertia = 1.0f / inertia;
}

AABB Polygon::GetAABB() const
{
    const Transform& t = GetTransform();

    Vec2 min = t * vertices[0];
    Vec2 max = min;

    for (size_t i = 1; i < vertices.size(); i++)
    {
        Vec2 v = t * vertices[i];

        min = Min(min, v);
        max = Max(max, v);
    }

    min -= radius;
    max += radius;

    return AABB{ min, max };
}

ContactPoint Polygon::Support(const Vec2& localDir) const
{
    int32 idx = 0;
    float maxValue = Dot(localDir, vertices[idx]);

    for (int32 i = 1; i < vertices.size(); i++)
    {
        float value = Dot(localDir, vertices[i]);
        if (value > maxValue)
        {
            idx = i;
            maxValue = value;
        }
    }

    return ContactPoint{ vertices[idx], idx };
}

Edge Polygon::GetFeaturedEdge(const Vec2& dir) const
{
    const Vec2 localDir = MulT(transform.rotation, dir);
    const ContactPoint farthest = Support(localDir);

    Vec2 curr = farthest.position;
    int32 idx = farthest.id;

    int32 vertexCount = static_cast<int32>(vertices.size());
    int32 prevIdx = (idx - 1 + vertexCount) % vertexCount;
    int32 nextIdx = (idx + 1) % vertexCount;
    const Vec2& prev = vertices[prevIdx];
    const Vec2& next = vertices[nextIdx];

    Vec2 e1 = (curr - prev).Normalized();
    Vec2 e2 = (curr - next).Normalized();

    bool w = Dot(e1, localDir) <= Dot(e2, localDir);

    if (w)
    {
        return Edge{ transform * prev, transform * curr, prevIdx, idx };
    }
    else
    {
        return Edge{ transform * curr, transform * next, idx, nextIdx };
    }
}

} // namespace muli