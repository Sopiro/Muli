#include "spe/polygon.h"

namespace spe
{

Polygon::Polygon(const std::vector<Vec2>& _vertices, Type _type, bool _resetCenter, float _radius, float _density)
    : RigidBody(_type, RigidBody::Shape::ShapePolygon)
    , vertices{ _vertices }
{
    radius = _radius;

    Vec2 centerOfMass{ 0.0f };
    size_t count = vertices.size();

    for (size_t i = 0; i < count; i++)
    {
        centerOfMass += vertices[i];
    }

    centerOfMass *= 1.0f / count;

    float _area = 0;

    vertices[0] -= centerOfMass;

    for (uint32 i = 1; i < count; i++)
    {
        vertices[i] -= centerOfMass;
        _area += Cross(vertices[i - 1], vertices[i]);
    }
    _area += Cross(vertices[count - 1], vertices[0]);

    area = Abs(_area) / 2.0f;

    if (type == Dynamic)
    {
        speAssert(_density > 0);

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
    speAssert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = ComputePolygonInertia(vertices, mass);
    invInertia = 1.0f / inertia;
}

void Polygon::SetDensity(float _density)
{
    speAssert(_density > 0);

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

} // namespace spe