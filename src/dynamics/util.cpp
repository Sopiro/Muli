#include "spe/util.h"
#include "spe/aabb.h"
#include "spe/box.h"
#include "spe/circle.h"

namespace spe
{

float spe::compute_convex_polygon_inertia(const std::vector<Vec2>& vertices, float mass)
{
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (uint32_t i = 0; i < vertices.size(); i++)
    {
        uint32_t j = (i + 1) % vertices.size();

        const Vec2& vi = vertices[i];
        const Vec2& vj = vertices[j];

        float crs = abs(cross(vj, vi));

        numerator += crs * (dot(vj, vj) + dot(vj, vi) + dot(vi, vi));
        denominator += crs;
    }

    return (numerator * mass) / (denominator * 6.0f);
}

Vec3 rgb2hsl(float r, float g, float b)
{
    r /= 255.0f;
    g /= 255.0f;
    b /= 255.0f;

    float max = spe::max(spe::max(r, g), b);
    float min = spe::min(spe::min(r, g), b);

    Vec3 res{ (max + min) / 2.0f };

    if (max == min)
    {
        // achromatic
        res.x = 0.0f;
        res.y = 0.0f;
    }
    else
    {
        float d = max - min;
        res.x = (res.z > 0.5f) ? d / (2.0f - max - min) : d / (max + min);

        if (max == r)
            res.x = (g - b) / d + (g < b ? 6 : 0);
        else if (max == g)
            res.x = (b - r) / d + 2;
        else if (max == b)
            res.x = (r - g) / d + 4;

        res.x /= 6;
    }

    return res;
}

float hue2rgb(float p, float q, float t)
{
    if (t < 0.0f) t += 1.0f;
    if (t > 1.0f) t -= 1.0f;
    if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
    if (t < 1.0f / 2.0f) return q;
    if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;

    return p;
}

Vec3 hsl2rgb(float h, float s, float l)
{
    Vec3 res;

    if (s == 0.0f)
    {
        res.x = res.y = res.z = l; // achromatic
    }
    else
    {
        float q = l < 0.5f ? l * (1.0f + s) : l + s - l * s;
        float p = 2.0f * l - q;
        res.x = hue2rgb(p, q, h + 1.0f / 3.0f);
        res.y = hue2rgb(p, q, h);
        res.z = hue2rgb(p, q, h - 1.0f / 3.0f);
    }

    return res;
}

} // namespace spe