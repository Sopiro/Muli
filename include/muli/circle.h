#pragma once

#include "shape.h"

namespace muli
{

class Circle : public Shape
{
public:
    Circle(float radius, const Vec2& center = Vec2::zero);
    ~Circle() = default;

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const override;

    virtual int32 GetVertexCount() const override;
    virtual Vec2 GetVertex(int32 id) const override;
    virtual int32 GetSupport(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const override;

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
    MuliNotUsed(dir);
    return Edge{ Mul(transform, center), Mul(transform, center) };
}

inline Vec2 Circle::GetVertex(int32 id) const
{
    MuliAssert(id == 0);
    MuliNotUsed(id);
    return center;
}

inline int32 Circle::GetVertexCount() const
{
    return 1;
}

inline int32 Circle::GetSupport(const Vec2& localDir) const
{
    MuliNotUsed(localDir);
    return 0;
}

} // namespace muli
