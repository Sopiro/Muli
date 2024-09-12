#pragma once

#include "shape.h"

namespace muli
{

class Capsule : public Shape
{
public:
    Capsule(float length, float radius, bool horizontal = false, const Vec2& position = Vec2::zero);
    Capsule(const Vec2& p1, const Vec2& p2, float radius, bool resetPosition = false);
    ~Capsule() = default;

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const override;

    virtual int32 GetVertexCount() const override;
    virtual Vec2 GetVertex(int32 id) const override;
    virtual int32 GetSupport(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const override;

    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual Vec2 GetClosestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

    float GetLength() const;
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
    MuliNotUsed(dir);
    return Edge{ Mul(transform, va), Mul(transform, vb), 0, 1 };
}

inline Vec2 Capsule::GetVertex(int32 id) const
{
    MuliAssert(id == 0 || id == 1);
    return id == 0 ? va : vb;
}

inline int32 Capsule::GetVertexCount() const
{
    return 2;
}

inline int32 Capsule::GetSupport(const Vec2& localDir) const
{
    Vec2 e = vb - va;
    return Dot(e, localDir) > 0.0f ? 1 : 0;
}

inline float Capsule::GetLength() const
{
    return length;
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
