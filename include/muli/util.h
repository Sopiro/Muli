#pragma once

#include "common.h"
#include "rigidbody.h"
#include "settings.h"
#include <random>

namespace muli
{

struct BodyPair
{
    uint32 first;
    uint32 second;
};

union PairID
{
    BodyPair pair;
    uint64 key;
};

// Compute CCW Convex hull
void ComputeConvexHull(const Vec2* vertices, int32 vertexCount, Vec2* outVertices, int32* outVertexCount);
std::vector<Vec2> ComputeConvexHull(const std::vector<Vec2>& vertices);

// Cantor pairing function, ((N, N) -> N) mapping function
// https://en.wikipedia.org/wiki/Pairing_function#Cantor_pairing_function
inline uint32 MakePairNatural(uint32 a, uint32 b)
{
    return (a + b) * (a + b + 1) / 2 + b;
}

inline PairID CombineID(uint32 a, uint32 b)
{
    muliAssert(a != b);

    PairID pid;

    if (a < b)
    {
        pid.pair.first = a;
        pid.pair.second = b;
    }
    else
    {
        pid.pair.first = b;
        pid.pair.first = a;
    }

    return pid;
}

inline bool operator==(PairID lhs, PairID rhs)
{
    return lhs.key == rhs.key;
}

// Reverse version of pairing function
// this guarantees initial pairing order
inline std::pair<uint32, uint32> SeparatePair(uint32 p)
{
    float w = Floor((Sqrt(8.0f * p + 1.0f) - 1) / 2.0f);
    float t = (w * w + w) / 2.0f;

    float y = p - t;
    float x = w - y;

    return { static_cast<uint32>(x), static_cast<uint32>(y) };
}

static std::random_device rd;
static std::mt19937 g(rd());

inline int LinearRand(int _min, int _max)
{
    std::uniform_int_distribution<int> ud(_min, _max);

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

inline float Lerp(float left, float right, float per)
{
    return left + (right - left) * per;
}

inline float Map(float v, float left, float right, float min, float max)
{
    float per = (v - left) / (right - left);

    return Lerp(min, max, per);
}

struct UV
{
    float u;
    float v;
};

// Project point P to line segment AB to compute barycentric weights
inline UV ComputeWeights(const Vec2& a, const Vec2& b, const Vec2& p)
{
    Vec2 dir = b - a;
    float len = dir.Normalize();
    float region = Dot(dir, p - a) / len;

    return UV{ 1.0f - region, region };
}

// Linearly combine(interpolate) the vector using weights u, v
inline Vec2 LerpVector(const Vec2& a, const Vec2& b, const UV& uv)
{
    return Vec2{ a.x * uv.u + b.x * uv.v, a.y * uv.u + b.y * uv.v };
}

inline void print()
{
    std::cout << '\n';
}

template <typename T>
inline void print(T msg, bool lineFeed = true)
{
    if (lineFeed)
        std::cout << msg << '\n';
    else
        std::cout << msg;
}

inline void print(const Vec2& v, bool lineFeed = true)
{
    if (lineFeed)
        printf("%.6f, %.6f\n", v.x, v.y);
    else
        printf("%.6f, %.6f\n", v.x, v.y);
}

inline std::ostream& operator<<(std::ostream& out, const Vec2& v)
{
    return out << v.x << ' ' << v.y << '\n';
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

} // namespace muli
