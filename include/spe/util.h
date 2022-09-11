#pragma once

#include "common.h"
#include "rigidbody.h"
#include "settings.h"
#include <random>

namespace spe
{

struct BodyPair
{
    uint32_t first;
    uint32_t second;
};

union PairID
{
    BodyPair pair;
    uint64_t key;
};

// https://en.wikipedia.org/wiki/List_of_moments_of_inertia
float compute_convex_polygon_inertia(const std::vector<Vec2>& vertices, float mass);

inline float compute_box_inertia(float width, float height, float mass)
{
    return (width * width + height * height) * mass / 12.0f;
}

inline float compute_circle_inertia(float radius, float mass)
{
    return mass * radius * radius / 2.0f;
}

// Cantor pairing function, ((N, N) -> N) mapping function
// https://en.wikipedia.org/wiki/Pairing_function#Cantor_pairing_function
inline uint32_t make_pair_natural(uint32_t a, uint32_t b)
{
    return (a + b) * (a + b + 1) / 2 + b;
}

inline PairID combine_id(uint32_t a, uint32_t b)
{
    assert(a != b);
    return a < b ? PairID{ a, b } : PairID{ b, a };
}

inline bool operator==(PairID lhs, PairID rhs)
{
    return lhs.key == rhs.key;
}

// Reverse version of pairing function
// this guarantees initial pairing order
inline std::pair<uint32_t, uint32_t> separate_pair(uint32_t p)
{
    float w = spe::floor((spe::sqrt(8.0f * p + 1.0f) - 1) / 2.0f);
    float t = (w * w + w) / 2.0f;

    float y = p - t;
    float x = w - y;

    return { static_cast<uint32_t>(x), static_cast<uint32_t>(y) };
}

inline int linear_rand(int _min, int _max)
{
    std::random_device rd;
    std::mt19937 g(rd());
    std::uniform_int_distribution<int> ud(_min, _max);

    return ud(g);
}

inline float linear_rand(float _min, float _max)
{
    std::random_device rd;
    std::mt19937 g(rd());
    std::uniform_real_distribution<float> ud(_min, _max);

    return ud(g);
}

inline Vec2 linear_rand(Vec2 _min, Vec2 _max)
{
    return Vec2{ linear_rand(_min.x, _max.x), linear_rand(_min.y, _max.y) };
}

inline float lerp(float left, float right, float per)
{
    return left + (right - left) * per;
}

inline float map(float v, float left, float right, float min, float max)
{
    float per = (v - left) / (right - left);

    return lerp(min, max, per);
}

struct UV
{
    float u;
    float v;
};

// Project point P to line segment AB, calculate barycentric weights
inline UV compute_uv(const Vec2& a, const Vec2& b, const Vec2& p)
{
    Vec2 dir = b - a;
    float len = dir.Length();
    dir.Normalize();

    float region = dot(dir, p - a) / len;

    return UV{ 1 - region, region };
}

// Linearly combine(interpolate) the vector using weights u, v
inline Vec2 lerp_vector(const Vec2& a, const Vec2& b, const UV& uv)
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
        printf("%.4f, %.4f\n", v.x, v.y);
    else
        printf("%.4f, %.4f\n", v.x, v.y);
}

} // namespace spe
