#pragma once

#include "shape.h"

#define MAX_LOCAL_POLYGON_VERTICES 8

namespace muli
{

class PolygonShape : public Shape
{
public:
    PolygonShape(const Vec2* _vertices, int32 _vertexCount, float radius);
    ~PolygonShape();

    virtual void ComputeMass(float density, MassData* outMassData) const override;
    virtual void ComputeAABB(const Transform& t, AABB* outAABB) const override;
    virtual bool TestPoint(const Transform& transform, const Vec2& q) const override;
    virtual bool RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const override;

protected:
    Vec2* vertices;
    Vec2* normals;
    int32 vertexCount;
    float area;

private:
    Vec2 localVertices[MAX_LOCAL_POLYGON_VERTICES];
    Vec2 localNormals[MAX_LOCAL_POLYGON_VERTICES];
};

} // namespace muli
