#ifndef __UTIL_H__
#define __UTIL_H__

#include "../common.h"

typedef glm::vec2 uv;

float CalculateConvexPolygonInertia(const std::vector<glm::vec2>& vertices, float mass, float area);

inline float CalculateBoxInertia(float width, float height, float mass)
{
    return (width * width + height * height) * mass / 12.0f;
}

inline float CalculateCircleInertia(float radius, float mass)
{
    return mass * radius * radius / 2.0f;
}

// Project point P to line segment AB, calculate barycentric weights
inline uv GetUV(glm::vec2 a, glm::vec2 b, glm::vec2 p)
{
    glm::vec2 dir = b - a;
    float len = glm::length(dir);
    dir = glm::normalize(dir);

    float region = glm::dot(dir, p - a) / len;

    return { 1 - region,  region };
}

// Linearly combine(interpolate) the vector using weights u, v
inline glm::vec2 LerpVector(glm::vec2 a, glm::vec2 b, uv uv)
{
    return { a.x * uv.x + b.x * uv.y, a.y * uv.x + b.y * uv.y };
}

// Cantor pairing function, ((N, N) -> N) mapping function
// https://en.wikipedia.org/wiki/Pairing_function#Cantor_pairing_function
inline uint32_t make_pair_natural(uint32_t a, uint32_t b)
{
    return (a + b) * (a + b + 1) / 2 + b;
}

// Reverse version of pairing function
// this guarantees initial pairing order
inline std::pair<uint32_t, uint32_t> separate_pair(uint32_t p)
{
    auto w = glm::floor((glm::sqrt(8 * p + 1) - 1) / 2.0);
    auto t = (w * w + w) / 2.0;

    auto y = p - t;
    auto x = w - y;

    return { (uint32_t)x, (uint32_t)y };
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

inline glm::vec2 mid(glm::vec2 a, glm::vec2 b)
{
    return (a + b) / 2.0f;
}

#endif
