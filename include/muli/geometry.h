#pragma once

#include "circle.h"
#include "common.h"

namespace muli
{

// Compute CCW Convex hull
void ComputeConvexHull(const Vec2* vertices, int32 vertexCount, Vec2* outVertices, int32* outVertexCount);
std::vector<Vec2> ComputeConvexHull(std::span<Vec2> vertices);

// Compute minimum circle containing all points
Circle ComputeCircle(std::span<Vec2> points);

} // namespace muli
