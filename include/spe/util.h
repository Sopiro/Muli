#pragma once

#include "common.h"
#include "rigidbody.h"
#include "settings.h"

struct uv
{
    float u;
    float v;
};

namespace spe
{
class Polygon;

float calculate_convex_polygon_inertia(const std::vector<glm::vec2>& vertices, float mass, float area);

inline float calculate_box_inertia(float width, float height, float mass)
{
    return (width * width + height * height) * mass / 12.0f;
}

inline float calculate_circle_inertia(float radius, float mass)
{
    return mass * radius * radius / 2.0f;
}

// Project point P to line segment AB, calculate barycentric weights
inline uv get_uv(glm::vec2 a, glm::vec2 b, glm::vec2 p)
{
    glm::vec2 dir = b - a;
    float len = glm::length(dir);
    dir = glm::normalize(dir);

    float region = glm::dot(dir, p - a) / len;

    return { 1 - region,  region };
}

// Linearly combine(interpolate) the vector using weights u, v
inline glm::vec2 lerp_vector(glm::vec2 a, glm::vec2 b, uv uv)
{
    return { a.x * uv.u + b.x * uv.v, a.y * uv.u + b.y * uv.v };
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

Polygon* create_random_convex_body(float radius, uint32_t num_vertices = 0, float density = DEFAULT_DENSITY);

Polygon* create_regular_polygon(float radius, uint32_t num_vertices = 0, float initial_angle = 0, float density = DEFAULT_DENSITY);

std::vector<std::pair<RigidBody*, RigidBody*>> get_collision_pair_n2(const std::vector<RigidBody*>& bodies);

// https://gist.github.com/ciembor/1494530

/*
 * Converts an RGB color value to HSL. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes r, g, and b are contained in the set [0, 255] and
 * returns HSL in the set [0, 1].
 */
inline glm::vec3 rgb2hsl(float r, float g, float b)
{
    r /= 255.0f;
    g /= 255.0f;
    b /= 255.0f;

    float max = glm::max(glm::max(r, g), b);
    float min = glm::min(glm::min(r, g), b);

    glm::vec3 res{ (max + min) / 2.0f };

    if (max == min)
    {
        // achromatic
        res.x = 0.0f;
        res.y = 0.0f;
    }
    else
    {
        float d = max - min;
        res.s = (res.z > 0.5f) ? d / (2.0f - max - min) : d / (max + min);

        if (max == r) res.x = (g - b) / d + (g < b ? 6 : 0);
        else if (max == g) res.x = (b - r) / d + 2;
        else if (max == b) res.x = (r - g) / d + 4;

        res.x /= 6;
    }

    return res;
}

/*
 * Converts an HUE to r, g or b.
 * returns float in the set [0, 1].
 */
inline float hue2rgb(float p, float q, float t)
{
    if (t < 0.0f) t += 1.0f;
    if (t > 1.0f) t -= 1.0f;
    if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
    if (t < 1.0f / 2.0f) return q;
    if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;

    return p;
}

/*
 * Converts an HSL color value to RGB. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes h, s, and l are contained in the set [0, 1] and
 * returns RGB in the set [0, 1].
 */
inline glm::vec3 hsl2rgb(float h, float s, float l)
{
    glm::vec3 res;

    if (s == 0.0f)
    {
        res.r = res.g = res.b = l; // achromatic
    }
    else
    {
        float q = l < 0.5f ? l * (1.0f + s) : l + s - l * s;
        float p = 2.0f * l - q;
        res.r = hue2rgb(p, q, h + 1.0f / 3.0f);
        res.g = hue2rgb(p, q, h);
        res.b = hue2rgb(p, q, h - 1.0f / 3.0f);
    }

    return res;
}
}
