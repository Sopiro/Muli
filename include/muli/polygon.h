#pragma once

#include "rigidbody.h"
#include "util.h"

namespace muli
{

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
    virtual ContactPoint Support(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Vec2& dir) const override;

    const std::vector<Vec2>& GetVertices() const;
    const std::vector<Vec2>& GetNormals() const;
    size_t GetVertexCount() const;

protected:
    std::vector<Vec2> vertices;
    std::vector<Vec2> normals;
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

inline const std::vector<Vec2>& Polygon::GetNormals() const
{
    return normals;
}

inline size_t Polygon::GetVertexCount() const
{
    return vertices.size();
}

} // namespace muli