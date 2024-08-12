#pragma once

#include "circle.h"
#include "common.h"
#include "polygon.h"

namespace muli
{

// Compute CCW Convex hull
void ComputeConvexHull(const Vec2* vertices, int32 vertexCount, Vec2* outVertices, int32* outVertexCount);
std::vector<Vec2> ComputeConvexHull(std::span<const Vec2> vertices);

// Compute minimum circle containing all points
Circle ComputeCircle(std::span<Vec2> points);

// Compute delaunay triangles
std::vector<Polygon> ComputeTriangles(std::span<Vec2> points, bool removeOutliers = false);

} // namespace muli
