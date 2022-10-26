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

    // Traslate center of mass to the origin
    Vec2 centerOfMass{ 0.0f };
    for (int32 i = 0; i < vertexCount; i++)
    {
        centerOfMass += vertices[i];
    }
    centerOfMass *= 1.0f / vertexCount;

    // Compute area
    area = 0;
    vertices[0] -= centerOfMass;
    for (int32 i = 1; i < vertexCount; i++)
    {
        vertices[i] -= centerOfMass;
        area += Cross(vertices[i - 1], vertices[i]) * 0.5f;        // inside triangle
        area += radius * (vertices[i - 1] - vertices[i]).Length(); // edge rect
    }
    area += Cross(vertices[vertexCount - 1], vertices[0]) * 0.5f;
    area += radius * (vertices[vertexCount - 1] - vertices[0]).Length();

    area += MULI_PI * radius * radius; // corner arc

    if (type == Dynamic)
    {
        muliAssert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = ComputePolygonInertia(this);
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
    inertia = ComputePolygonInertia(this);
    invInertia = 1.0f / inertia;
}

void Polygon::SetDensity(float _density)
{
    muliAssert(_density > 0);

    density = _density;
    mass = _density * area;
    invMass = 1.0f / mass;
    inertia = ComputePolygonInertia(this);
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

inline bool Polygon::TestPoint(const Vec2& p) const
{
    Vec2 localP = MulT(transform, p);

    float dir = Cross(vertices[0] - localP, vertices[1] - localP);
    for (int32 i0 = 1; i0 < vertexCount; i0++)
    {
        int32 i1 = (i0 + 1) % vertexCount;

        float nDir = Cross(vertices[i0] - localP, vertices[i1] - localP);

        if (dir * nDir < 0)
        {
            return false;
        }
    }

    return true;
}

bool Polygon::RayCast(const RayCastInput& input, RayCastOutput* output) const
{
    output->fraction = 0.0f;
    output->normal.SetZero();

    return true;
}

} // namespace muli