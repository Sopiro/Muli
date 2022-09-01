#include "spe/util.h"
#include "spe/aabb.h"
#include "spe/box.h"
#include "spe/circle.h"

namespace spe
{

float spe::compute_convex_polygon_inertia(const std::vector<glm::vec2>& vertices, float mass)
{
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (uint32_t i = 0; i < vertices.size(); i++)
    {
        uint32_t j = (i + 1) % vertices.size();

        const glm::vec2& vi = vertices[i];
        const glm::vec2& vj = vertices[j];

        float cross = glm::abs(glm::cross(vj, vi));

        numerator += cross * (glm::dot(vj, vj) + glm::dot(vj, vi) + glm::dot(vi, vi));
        denominator += cross;
    }

    return (numerator * mass) / (denominator * 6.0f);
}

glm::vec3 rgb2hsl(float r, float g, float b)
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

glm::vec3 hsl2rgb(float h, float s, float l)
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

} // namespace spe