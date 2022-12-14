#pragma once

#include "aabb.h"
#include "collision.h"
#include "common.h"
#include "edge.h"
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
    virtual ContactPoint Support(const Vec2& localDir) const = 0;
    virtual Vec2 GetVertex(int32 id) const = 0;
    virtual int32 GetVertexCount() const = 0;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const = 0;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const = 0;
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

inline Shape::Shape(Type _type, float _radius)
    : type{ _type }
    , radius{ _radius }
    , center{ 0.0f }
{
    // Radius must be greater than linear_slop * 2.0
    // Smaller radius than linear_slop * 2.0 will lead unstable CCD
    muliAssert(radius >= LINEAR_SLOP * 2.0f);
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
