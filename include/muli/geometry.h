#pragma once

#include "circle.h"
#include "common.h"
#include "polygon.h"

namespace muli
{

// Compute CCW Convex hull
std::vector<Vec2> ComputeConvexHull(std::span<const Vec2> vertices);

// Compute minimum circle containing all points
Circle ComputeCircle(std::span<Vec2> vertices);

// Compute Delaunay triangles
// Behavior is undefined for self-intersecting constraints
std::vector<Polygon> ComputeTriangles(std::span<Vec2> vertices, std::span<std::vector<Vec2>> constraints = {});

// Compute convex polygon set from concave outlines
// Behavior is undefined for self-intersecting constraints
std::vector<Polygon> ComputeDecomposition(std::span<std::vector<Vec2>> constraints);

} // namespace muli
