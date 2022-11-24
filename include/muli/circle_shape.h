#pragma once

#include "shape.h"
#include "util.h"

namespace muli
{

class CircleShape : public Shape
{
public:
    CircleShape(float radius);
    ~CircleShape();

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const override;
    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;
};

inline CircleShape::CircleShape(float radius)
    : Shape(Shape::Type::circle, radius)
{
    area = radius * radius * MULI_PI;
    localPosition.SetZero();
}

inline void CircleShape::ComputeMass(float density, MassData* outMassData) const
{
    outMassData->mass = density * area;
    outMassData->inertia = outMassData->mass * (0.5f * radius * radius + Length2(localPosition));
    outMassData->centerOfMass = localPosition;
}

inline void CircleShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    Vec2 p = transform * localPosition;

    outAABB->min = Vec2{ p.x - radius, p.y - radius };
    outAABB->max = Vec2{ p.x + radius, p.y + radius };
}

inline bool CircleShape::TestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 w = transform * localPosition;
    Vec2 d = d - q;

    return Dot(d, d) <= radius * radius;
}

} // namespace muli
