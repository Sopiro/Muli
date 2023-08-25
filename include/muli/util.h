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

// Randoms

// https://www.pcg-random.org/
inline uint32 PCGHash(uint32 rngState)
{
    uint32 state = rngState * 747796405u + 2891336453u;
    uint32 word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

inline uint32 Rand(uint32& seed)
{
    seed = PCGHash(seed);
    return seed;
}

inline thread_local uint32 seed = 1234;

inline float Rand()
{
    return Rand(seed) / float(UINT32_MAX);
}

inline void Srand(uint32 newSeed)
{
    seed = newSeed;
}

inline int32 RandRange(int32 min, int32 max)
{
    return min + (max - min) * Rand();
}

inline float RandRange(float min, float max)
{
    return min + (max - min) * Rand();
}

inline Vec2 RandVec2(Vec2 min, Vec2 max)
{
    return Vec2{ RandRange(min.x, max.x), RandRange(min.y, max.y) };
}

inline Vec3 RandVec3(Vec3 min, Vec3 max)
{
    return Vec3{ RandRange(min.x, max.x), RandRange(min.y, max.y), RandRange(min.z, max.z) };
}

inline Vec4 RandVec4(Vec4 min, Vec4 max)
{
    return Vec4{ RandRange(min.x, max.x), RandRange(min.y, max.y), RandRange(min.z, max.z), RandRange(min.w, max.w) };
}

// Compute CCW Convex hull
void ComputeConvexHull(const Vec2* vertices, int32 vertexCount, Vec2* outVertices, int32* outVertexCount);
std::vector<Vec2> ComputeConvexHull(const std::vector<Vec2>& vertices);

// Raycast functions
bool RayCastCircle(const Vec2& center, float radius, const RayCastInput& input, RayCastOutput* output);
bool RayCastLineSegment(const Vec2& vertex1, const Vec2& vertex2, const RayCastInput& input, RayCastOutput* output);
bool RayCastCapsule(const Vec2& vertex1, const Vec2& vertex2, float radius, const RayCastInput& input, RayCastOutput* output);

} // namespace muli
