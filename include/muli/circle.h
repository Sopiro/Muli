#pragma once

#include "shape.h"
#include "util.h"

namespace muli
{

class Circle : public Shape
{
public:
    Circle(float radius, const Vec2& center = Vec2{ 0.0f });
    ~Circle() = default;

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual Vec2 GetVertex(int32 id) const override;
    virtual ContactPoint Support(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const override;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const override;
    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual Vec2 GetClosestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

protected:
    virtual Shape* Clone(Allocator* allocator) const override;
};

inline Shape* Circle::Clone(Allocator* allocator) const
{
    void* mem = allocator->Allocate(sizeof(Circle));
    Circle* shape = new (mem) Circle(*this);
    return shape;
}

inline Edge Circle::GetFeaturedEdge(const Transform& transform, const Vec2& dir) const
{
    return Edge{ transform * center, transform * center };
}

inline Vec2 Circle::GetVertex(int32 id) const
{
    muliAssert(id == 0);
    return center;
}

inline ContactPoint Circle::Support(const Vec2& localDir) const
{
    return ContactPoint{ center, 0 };
}

inline void Circle::ComputeMass(float density, MassData* outMassData) const
{
    outMassData->mass = density * area;
    float inertia = 0.5f * radius * radius;
    outMassData->inertia = outMassData->mass * (inertia + Length2(center));
    outMassData->centerOfMass = center;
}

inline void Circle::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    Vec2 p = transform * center;

    outAABB->min = Vec2{ p.x - radius, p.y - radius };
    outAABB->max = Vec2{ p.x + radius, p.y + radius };
}

inline bool Circle::TestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 localQ = MulT(transform, q);
    Vec2 d = center - localQ;

    return Dot(d, d) <= radius * radius;
}

} // namespace muli
