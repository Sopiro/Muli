#pragma once

#include "common.h"
#include "rigidbody.h"
#include "settings.h"

#include <random>

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

static std::random_device rd;
static std::mt19937 g(rd());

inline int32 LinearRand(int32 _min, int32 _max)
{
    std::uniform_int_distribution<int32> ud(_min, _max);

    return ud(g);
}

inline float LinearRand(float _min, float _max)
{
    std::uniform_real_distribution<float> ud(_min, _max);

    return ud(g);
}

inline Vec2 LinearRand(Vec2 _min, Vec2 _max)
{
    return Vec2{ LinearRand(_min.x, _max.x), LinearRand(_min.y, _max.y) };
}

// Compute CCW Convex hull
void ComputeConvexHull(const Vec2* vertices, int32 vertexCount, Vec2* outVertices, int32* outVertexCount);
std::vector<Vec2> ComputeConvexHull(const std::vector<Vec2>& vertices);

struct UV
{
    float u;
    float v;
};

// Project point P to line segment AB to compute barycentric weights
inline UV ComputeWeights(const Vec2& a, const Vec2& b, const Vec2& p)
{
    Vec2 e = b - a;
    float len = e.Normalize();
    float region = Dot(e, p - a) / len;

    return UV{ 1.0f - region, region };
}

// Linearly combine(interpolate) the vector using weights u, v
inline Vec2 LerpVector(const Vec2& a, const Vec2& b, const UV& uv)
{
    return Vec2{ a.x * uv.u + b.x * uv.v, a.y * uv.u + b.y * uv.v };
}

float RayCastCircle(const Vec2& position, float radius, const RayCastInput& input, RayCastOutput* output);
bool RayCastLineSegment(const Vec2& v1, const Vec2& v2, const RayCastInput& input, RayCastOutput* output);

// https://iquilezles.org/articles/distfunctions/
inline float SignedDistanceToLineSegment(const Vec2& p, const Vec2& a, const Vec2& b, float r)
{
    Vec2 pa = p - a;
    Vec2 ba = b - a;
    float h = Clamp(Dot(pa, ba) / Dot(ba, ba), 0.0f, 1.0f);

    return Length(pa - ba * h) - r;
}

inline std::ostream& operator<<(std::ostream& out, const Vec2& v)
{
    return out << v.x << ' ' << v.y << '\n';
}

inline std::ostream& operator<<(std::ostream& out, const Vec3& v)
{
    return out << v.x << ' ' << v.y << ' ' << v.z << '\n';
}

inline std::ostream& operator<<(std::ostream& out, const Vec4& v)
{
    return out << v.x << ' ' << v.y << ' ' << v.z << ' ' << v.w << '\n';
}

} // namespace muli
