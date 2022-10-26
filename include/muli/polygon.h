#pragma once

#include "rigidbody.h"
#include "util.h"

#define MAX_LOCAL_POLYGON_VERTICES 8

namespace muli
{

class Polygon : public RigidBody
{
public:
    Polygon(const Vec2* _vertices,
            int32 _vertexCount,
            RigidBody::Type _type = Dynamic,
            bool _resetCenter = true,
            float _radius = DEFAULT_RADIUS,
            float _density = DEFAULT_DENSITY);
    virtual ~Polygon() noexcept;

    virtual void SetDensity(float d) override;
    virtual void SetMass(float m) override;

    virtual float GetArea() const override final;
    virtual AABB GetAABB() const override;
    virtual ContactPoint Support(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Vec2& dir) const override;
    virtual bool RayCast(const RayCastInput& input, RayCastOutput* output) const override;

    const Vec2* GetVertices() const;
    int32 GetVertexCount() const;

protected:
    Vec2* vertices;
    int32 vertexCount;
    float area;

private:
    Vec2 localVertices[MAX_LOCAL_POLYGON_VERTICES];
};

inline float Polygon::GetArea() const
{
    return area;
}

inline const Vec2* Polygon::GetVertices() const
{
    return vertices;
}

inline int32 Polygon::GetVertexCount() const
{
    return vertexCount;
}

} // namespace muli