#pragma once

#include "rigidbody.h"
#include "util.h"

namespace spe
{

// Children: Box
class Polygon : public RigidBody
{
public:
    Polygon(const std::vector<Vec2>& _vertices,
            RigidBody::Type _type = Dynamic,
            bool _resetCenter = true,
            float _radius = DEFAULT_RADIUS,
            float _density = DEFAULT_DENSITY);

    virtual void SetDensity(float d) override;
    virtual void SetMass(float m) override;

    virtual float GetArea() const override final;
    virtual AABB GetAABB() const override;

    const std::vector<Vec2>& GetVertices() const;
    size_t VertexCount() const;

protected:
    std::vector<Vec2> vertices;
    float area;
};

inline float Polygon::GetArea() const
{
    return area;
}

inline const std::vector<Vec2>& Polygon::GetVertices() const
{
    return vertices;
}

inline size_t Polygon::VertexCount() const
{
    return vertices.size();
}

inline AABB Polygon::GetAABB() const
{
    const Transform& localToGlobal = GetTransform();

    Vec2 min = localToGlobal * vertices[0];
    Vec2 max = min;

    for (size_t i = 1; i < vertices.size(); i++)
    {
        Vec2 v = localToGlobal * vertices[i];

        min = Min(min, v);
        max = Max(max, v);
    }

    min -= radius;
    max += radius;

    return AABB{ min, max };
}

} // namespace spe