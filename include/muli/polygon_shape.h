#pragma once

#include "shape.h"

#define MAX_LOCAL_POLYGON_VERTICES 8

namespace muli
{

class PolygonShape : public Shape
{
public:
    PolygonShape(const Vec2* vertices, int32 vertexCount, bool resetPosition = true, float radius = DEFAULT_RADIUS);
    PolygonShape(
        float width, float height, float radius = DEFAULT_RADIUS, const Vec2& position = Vec2{ 0.0f }, float angle = 0.0f);
    PolygonShape(float size, float radius = DEFAULT_RADIUS, const Vec2& position = Vec2{ 0.0f }, float angle = 0.0f);
    ~PolygonShape();

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual ContactPoint Support(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Transform& transform, const Vec2& dir) const override;
    virtual void ComputeAABB(const Transform& transform, AABB* outAABB) const override;
    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual Vec2 GetClosestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

    const Vec2* GetVertices() const;
    const Vec2* GetNormals() const;
    int32 GetVertexCount() const;
    float GetArea() const;

protected:
    virtual Shape* Clone(PredefinedBlockAllocator* allocator) const override;

    Vec2* vertices;
    Vec2* normals;
    int32 vertexCount;

private:
    Vec2 localVertices[MAX_LOCAL_POLYGON_VERTICES];
    Vec2 localNormals[MAX_LOCAL_POLYGON_VERTICES];
};

inline const Vec2* PolygonShape::GetVertices() const
{
    return vertices;
}

inline const Vec2* PolygonShape::GetNormals() const
{
    return normals;
}

inline int32 PolygonShape::GetVertexCount() const
{
    return vertexCount;
}

inline float PolygonShape::GetArea() const
{
    return area;
}

} // namespace muli