#pragma once

#include "settings.h"
#include "shape.h"

namespace muli
{

class Box : public Shape
{
public:
    Box(float width, float height, float radius = default_radius, const Transform& tf = identity);
    Box(float size, float radius = default_radius, const Transform& tf = identity);

    Box(const Box& other, const Transform& tf);

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const override;

    virtual int32 GetVertexCount() const override;
    virtual Vec2 GetVertex(int32 id) const override;
    virtual int32 GetSupport(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const override;

    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual Vec2 GetClosestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

    const Vec2& GetHalfExtents() const;
    const Rotation& GetRotation() const;
    Vec2 GetNormal(int32 id) const;

protected:
    virtual Shape* Clone(Allocator* allocator, const Transform& tf) const override;

private:
    Vec2 halfExtents;
    Rotation rotation;
};

inline Box::Box(float size, float radius, const Transform& tf)
    : Box(size, size, radius, tf)
{
}

inline Shape* Box::Clone(Allocator* allocator, const Transform& tf) const
{
    void* mem = allocator->Allocate(sizeof(Box));
    Box* shape = new (mem) Box(*this, tf);
    return shape;
}

inline int32 Box::GetVertexCount() const
{
    return 4;
}

inline Vec2 Box::GetVertex(int32 id) const
{
    MuliAssert(0 <= id && id < 4);

    Vec2 corner;
    switch (id)
    {
    case 0:
        corner = { -halfExtents.x, -halfExtents.y };
        break;
    case 1:
        corner = { halfExtents.x, -halfExtents.y };
        break;
    case 2:
        corner = { halfExtents.x, halfExtents.y };
        break;
    default:
        corner = { -halfExtents.x, halfExtents.y };
        break;
    }

    return center + Mul(rotation, corner);
}

inline int32 Box::GetSupport(const Vec2& localDir) const
{
    Vec2 dir = MulT(rotation, localDir);

    if (dir.x >= 0.0f)
    {
        return dir.y >= 0.0f ? 2 : 1;
    }
    else
    {
        return dir.y >= 0.0f ? 3 : 0;
    }
}

inline const Vec2& Box::GetHalfExtents() const
{
    return halfExtents;
}

inline const Rotation& Box::GetRotation() const
{
    return rotation;
}

inline Vec2 Box::GetNormal(int32 id) const
{
    MuliAssert(0 <= id && id < 4);

    switch (id)
    {
    case 0:
        return Mul(rotation, Vec2{ 0.0f, -1.0f });
    case 1:
        return Mul(rotation, Vec2{ 1.0f, 0.0f });
    case 2:
        return Mul(rotation, Vec2{ 0.0f, 1.0f });
    default:
        return Mul(rotation, Vec2{ -1.0f, 0.0f });
    }
}

} // namespace muli
