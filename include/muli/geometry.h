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
Circle ComputeCircle(std::span<Vec2> vertices);

// Compute Delaunay triangles
// Behavior is undefined for self-intersecting outline and holes
std::vector<Polygon> ComputeTriangles(
    std::span<Vec2> vertices, std::span<Vec2> outline = {}, std::span<std::vector<Vec2>> hole = {}
);

// Compute convex polygon set from concave outline
// There must be no duplicate vertices!
// Behavior is undefined for self-intersecting outline
std::vector<Polygon> ComputeDecomposition(std::span<Vec2> outline);

} // namespace muli
