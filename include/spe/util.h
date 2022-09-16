#pragma once

#include "common.h"
#include "rigidbody.h"
#include "settings.h"
#include <random>

namespace spe
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

// https://en.wikipedia.org/wiki/List_of_moments_of_inertia
float ComputePolygonInertia(const std::vector<Vec2>& vertices, float mass);

float ComputeCapsuleInertia(float length, float radius, float mass);

inline float ComputeBoxInertia(float width, float height, float mass)
{
    return (width * width + height * height) * mass / 12.0f;
}

inline float ComputeCircleInertia(float radius, float mass)
{
    return mass * radius * radius / 2.0f;
}

// Cantor pairing function, ((N, N) -> N) mapping function
// https://en.wikipedia.org/wiki/Pairing_function#Cantor_pairing_function
inline uint32 MakePairNatural(uint32 a, uint32 b)
{
    return (a + b) * (a + b + 1) / 2 + b;
}

inline PairID CombineID(uint32 a, uint32 b)
{
    speAssert(a != b);
    return a < b ? PairID{ a, b } : PairID{ b, a };
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

inline int LinearRand(int _min, int _max)
{
    std::random_device rd;
    std::mt19937 g(rd());
    std::uniform_int_distribution<int> ud(_min, _max);

    return ud(g);
}

inline float LinearRand(float _min, float _max)
{
    std::random_device rd;
    std::mt19937 g(rd());
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

    return UV{ 1 - region, region };
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

} // namespace spe
