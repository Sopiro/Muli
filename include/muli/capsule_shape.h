#pragma once

#include "shape.h"
#include "util.h"

namespace muli
{

class CapsuleShape : public Shape
{
public:
    CapsuleShape(float _length, float radius);
    ~CapsuleShape();

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual void ComputeAABB(const Transform& t, AABB* outAABB) const override;
    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

private:
    float length;

    Vec2 va;
    Vec2 vb;
};

inline CapsuleShape::CapsuleShape(float _length, float radius)
    : Shape(Shape::Type::capsule, radius)
    , length{ _length }
    , va{ -_length / 2.0f, 0.0f }
    , vb{ _length / 2.0f, 0.0f }
{
    area = length * radius * 2.0f + MULI_PI * radius * radius;
}

inline void CapsuleShape::ComputeMass(float density, MassData* outMassData) const
{
    outMassData->mass = density * area;

    float height = radius * 2.0f;
    float invArea = 1.0f / area;

    float inertia;

    float rectArea = length * height;
    float rectInertia = (length * length + height * height) / 12.0f;

    inertia = rectInertia * rectArea * invArea;

    float circleArea = MULI_PI * radius * radius;
    float halfCircleInertia = ((MULI_PI / 4) - 8.0f / (9.0f * MULI_PI)) * radius * radius * radius * radius;
    float dist2 = length * 0.5f + (4.0f * radius) / (MULI_PI * 3.0f);
    dist2 *= dist2;

    inertia += (halfCircleInertia + (circleArea * 0.5f) * dist2) * 2.0f * invArea;

    outMassData->inertia = outMassData->mass * (inertia + Length2(localPosition));
    outMassData->centerOfMass = localPosition;
}

inline void CapsuleShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    Vec2 v1 = transform * va;
    Vec2 v2 = transform * vb;

    outAABB->min = Min(v1, v2) - Vec2{ radius, radius };
    outAABB->max = Max(v1, v2) + Vec2{ radius, radius };
}

inline bool CapsuleShape::TestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 localQ = MulT(transform, q);

    return SignedDistanceToLineSegment(localQ, va, vb, radius) < 0.0f;
}

} // namespace muli
