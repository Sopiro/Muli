#pragma once

#include "shape.h"

namespace muli
{

class Polygon : public Shape
{
public:
    Polygon(const Vec2* vertices, int32 vertexCount, bool resetPosition = false, float radius = default_radius);
    Polygon(std::initializer_list<Vec2> vertices, bool resetPosition = false, float radius = default_radius);
    Polygon(float width, float height, float radius = default_radius, const Vec2& position = zero_vec2, float angle = 0.0f);
    Polygon(float size, float radius = default_radius, const Vec2& position = zero_vec2, float angle = 0.0f);
    ~Polygon();

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual Vec2 GetVertex(int32 id) const override;
    virtual int32 GetVertexCount() const override;
    virtual ContactPoint Support(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const override;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const override;
    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual Vec2 GetClosestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

    const Vec2* GetVertices() const;
    const Vec2* GetNormals() const;
    float GetArea() const;

protected:
    virtual Shape* Clone(Allocator* allocator) const override;

    Vec2* vertices;
    Vec2* normals;
    int32 vertexCount;

private:
    Vec2 localVertices[max_local_polygon_vertices];
    Vec2 localNormals[max_local_polygon_vertices];
};

inline Vec2 Polygon::GetVertex(int32 id) const
{
    muliAssert(0 <= id && id < vertexCount);
    return vertices[id];
}

inline int32 Polygon::GetVertexCount() const
{
    return vertexCount;
}

inline const Vec2* Polygon::GetVertices() const
{
    return vertices;
}

inline const Vec2* Polygon::GetNormals() const
{
    return normals;
}

inline float Polygon::GetArea() const
{
    return area;
}

} // namespace muli
