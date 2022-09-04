#pragma once

#include "common.h"
#include "rigidbody.h"
#include "settings.h"

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

struct UV
{
    float u;
    float v;
};

// https://en.wikipedia.org/wiki/List_of_moments_of_inertia
float compute_convex_polygon_inertia(const std::vector<glm::vec2>& vertices, float mass);

inline float compute_box_inertia(float width, float height, float mass)
{
    return (width * width + height * height) * mass / 12.0f;
}

inline float compute_circle_inertia(float radius, float mass)
{
    return mass * radius * radius / 2.0f;
}

// Project point P to line segment AB, calculate barycentric weights
inline UV compute_uv(glm::vec2 a, glm::vec2 b, glm::vec2 p)
{
    glm::vec2 dir = b - a;
    float len = glm::length(dir);
    dir = glm::normalize(dir);

    float region = glm::dot(dir, p - a) / len;

    return { 1 - region, region };
}

// Linearly combine(interpolate) the vector using weights u, v
inline glm::vec2 lerp_vector(glm::vec2 a, glm::vec2 b, UV uv)
{
    return { a.x * uv.u + b.x * uv.v, a.y * uv.u + b.y * uv.v };
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
    double w = glm::floor((glm::sqrt(8 * p + 1) - 1) / 2.0);
    double t = (w * w + w) / 2.0;

    double y = p - t;
    double x = w - y;

    return { static_cast<uint32_t>(x), static_cast<uint32_t>(y) };
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

// https://gist.github.com/ciembor/1494530
/*
 * Converts an RGB color value to HSL. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes r, g, and b are contained in the set [0, 255] and
 * returns HSL in the set [0, 1].
 */
glm::vec3 rgb2hsl(float r, float g, float b);

/*
 * Converts an HUE to r, g or b.
 * returns float in the set [0, 1].
 */
float hue2rgb(float p, float q, float t);

/*
 * Converts an HSL color value to RGB. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes h, s, and l are contained in the set [0, 1] and
 * returns RGB in the set [0, 1].
 */
glm::vec3 hsl2rgb(float h, float s, float l);

template <typename T>
inline void print(T msg)
{
    std::cout << msg << '\n';
}

inline void print(glm::vec2 msg)
{
    std::cout << *msg << '\n';
}

} // namespace spe
