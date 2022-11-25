#pragma once

#include "aabb.h"
#include "collision.h"
#include "common.h"
#include "predefined_block_allocator.h"

namespace muli
{

struct MassData
{
    float mass;
    float inertia;
    Vec2 centerOfMass;
};

class Shape
{
public:
    enum Type
    {
        circle = 0,
        capsule,
        polygon,
        shape_count
    };

    Shape(Type type, float radius);
    virtual ~Shape();

    virtual Shape* Clone(PredefinedBlockAllocator* allocator) const = 0;

    Type GetType() const;
    float GetRadius() const;
    float GetArea() const;

    virtual void ComputeMass(float density, MassData* outMassData) const = 0;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const = 0;
    virtual bool TestPoint(const Transform& transform, const Vec2& q) const = 0;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const = 0;

protected:
    Vec2 localPosition;

    Type type;
    float radius;
    float area;
};

inline Shape::Shape(Type _type, float _radius)
    : type{ _type }
    , radius{ _radius }
{
}

inline Shape::Type Shape::GetType() const
{
    return type;
}

inline float Shape::GetRadius() const
{
    return radius;
}

inline float Shape::GetArea() const
{
    return area;
}

} // namespace muli
