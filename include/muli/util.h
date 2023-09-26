#pragma once

#include "common.h"
#include "rigidbody.h"
#include "settings.h"

namespace muli
{

// default alloc/dealloc funcitons

inline void* Alloc(int32 size)
{
    return std::malloc(size);
}

inline void Free(void* mem)
{
    std::free(mem);
}

// Compute CCW Convex hull
void ComputeConvexHull(const Vec2* vertices, int32 vertexCount, Vec2* outVertices, int32* outVertexCount);
std::vector<Vec2> ComputeConvexHull(const std::vector<Vec2>& vertices);

// Raycast functions
bool RayCastCircle(const Vec2& center, float radius, const RayCastInput& input, RayCastOutput* output);
bool RayCastLineSegment(const Vec2& vertex1, const Vec2& vertex2, const RayCastInput& input, RayCastOutput* output);
bool RayCastCapsule(const Vec2& vertex1, const Vec2& vertex2, float radius, const RayCastInput& input, RayCastOutput* output);

} // namespace muli
