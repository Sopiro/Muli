#pragma once

#include "aabb.h"
#include "allocator.h"
#include "collision.h"
#include "common.h"
#include "primitives.h"
#include "raycast.h"

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
        // Order matters!
        circle = 0,
        capsule,
        polygon,
        shape_count
    };

    Shape(Type type, float radius);
    virtual ~Shape() = default;

    Type GetType() const;
    float GetRadius() const;
    float GetArea() const;
    const Vec2& GetCenter() const;

    virtual void ComputeMass(float density, MassData* outMassData) const = 0;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const = 0;

    virtual int32 GetVertexCount() const = 0;
    virtual Vec2 GetVertex(int32 id) const = 0;
    virtual int32 GetSupport(const Vec2& localDir) const = 0;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const = 0;

    virtual bool TestPoint(const Transform& transform, const Vec2& q) const = 0;
    virtual Vec2 GetClosestPoint(const Transform& transform, const Vec2& q) const = 0;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const = 0;

protected:
    friend class ContactManager;
    friend class Collider;
    friend class RigidBody;

    virtual Shape* Clone(Allocator* allocator) const = 0;

    Type type;

    Vec2 center;
    float radius;
    float area;
};

inline Shape::Shape(Type type, float radius)
    : type{ type }
    , center{ 0.0f }
    , radius{ radius }
{
#if 0
    // Radius must be greater than or equal to linear_slop * 2.0 for stable CCD
    MuliAssert(radius >= linear_slop * 2.0f);
#endif
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

inline const Vec2& Shape::GetCenter() const
{
    return center;
}

} // namespace muli
