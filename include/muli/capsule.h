#pragma once

#include "shape.h"
#include "util.h"
namespace muli
{

class Capsule : public Shape
{
public:
    Capsule(float length, float radius, bool horizontal = false, const Vec2& center = zero_vec2);
    Capsule(const Vec2& p1, const Vec2& p2, float radius, bool resetPosition = false);
    ~Capsule() = default;

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual Vec2 GetVertex(int32 id) const override;
    virtual int32 GetVertexCount() const override;
    virtual ContactPoint Support(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const override;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const override;
    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual Vec2 GetClosestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

    const Vec2& GetVertexA() const;
    const Vec2& GetVertexB() const;

protected:
    virtual Shape* Clone(Allocator* allocator) const override;

private:
    float length;

    Vec2 va;
    Vec2 vb;
};

inline Shape* Capsule::Clone(Allocator* allocator) const
{
    void* mem = allocator->Allocate(sizeof(Capsule));
    Capsule* shape = new (mem) Capsule(*this);
    return shape;
}

inline Edge Capsule::GetFeaturedEdge(const Transform& transform, const Vec2& dir) const
{
    return Edge{ transform * va, transform * vb, 0, 1 };
}

inline Vec2 Capsule::GetVertex(int32 id) const
{
    muliAssert(id == 0 || id == 1);
    return id == 0 ? va : vb;
}

inline int32 Capsule::GetVertexCount() const
{
    return 2;
}

inline ContactPoint Capsule::Support(const Vec2& localDir) const
{
    Vec2 dir = vb - va;
    return Dot(dir, localDir) > 0.0f ? ContactPoint{ vb, 1 } : ContactPoint{ va, 0 };
}

inline void Capsule::ComputeMass(float density, MassData* outMassData) const
{
    outMassData->mass = density * area;

    float height = radius * 2.0f;
    float invArea = 1.0f / area;

    float inertia;

    float rectArea = length * height;
    float rectInertia = (length * length + height * height) / 12.0f;

    inertia = rectInertia * rectArea * invArea;

    float circleArea = pi * radius * radius;
    float halfCircleInertia = ((pi / 4) - 8.0f / (9.0f * pi)) * radius * radius * radius * radius;
    float dist2 = length * 0.5f + (4.0f * radius) / (pi * 3.0f);
    dist2 *= dist2;

    inertia += (halfCircleInertia + (circleArea * 0.5f) * dist2) * 2.0f * invArea;

    outMassData->inertia = outMassData->mass * (inertia + Length2(center));
    outMassData->centerOfMass = center;
}

inline void Capsule::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    Vec2 v1 = transform * va;
    Vec2 v2 = transform * vb;

    outAABB->min = Min(v1, v2) - Vec2{ radius, radius };
    outAABB->max = Max(v1, v2) + Vec2{ radius, radius };
}

inline bool Capsule::TestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 localQ = MulT(transform, q);

    return SignedDistanceToLineSegment(localQ, va, vb, radius) < 0.0f;
}

inline const Vec2& Capsule::GetVertexA() const
{
    return va;
}

inline const Vec2& Capsule::GetVertexB() const
{
    return vb;
}

} // namespace muli
