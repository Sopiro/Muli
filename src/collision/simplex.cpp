#include "spe/simplex.h"
#include "spe/util.h"

namespace spe
{

// Returns the closest point to the input q
ClosestResult Simplex::GetClosest(const Vec2& q) const
{
    ClosestResult res;

    switch (count)
    {
    case 1: // 0-Simplex: Point
    {
        res.point = vertices[0];
        res.count = 1;
        res.contributors[0] = 0;

        return res;
    }
    case 2: // 1-Simplex: Line segment
    {
        const Vec2 a = vertices[0];
        const Vec2 b = vertices[1];
        const UV w = compute_uv(a, b, q);

        if (w.v <= 0)
        {
            res.point = a;
            res.count = 1;
            res.contributors[0] = 0;
            return res;
        }
        else if (w.v >= 1)
        {
            res.point = b;
            res.count = 1;
            res.contributors[0] = 1;
            return res;
        }
        else
        {
            res.point = lerp_vector(a, b, w);
            res.count = 2;
            res.contributors[0] = 0;
            res.contributors[1] = 1;
            return res;
        }
    }
    case 3: // 2-Simplex: Triangle
    {
        const Vec2 a = vertices[0];
        const Vec2 b = vertices[1];
        const Vec2 c = vertices[2];

        const UV wab = compute_uv(a, b, q);
        const UV wbc = compute_uv(b, c, q);
        const UV wca = compute_uv(c, a, q);

        if (wca.u <= 0 && wab.v <= 0) // A area
        {
            res.point = a;
            res.count = 1;
            res.contributors[0] = 0;
            return res;
        }
        else if (wab.u <= 0 && wbc.v <= 0) // B area
        {
            res.point = b;
            res.count = 1;
            res.contributors[0] = 1;
            return res;
        }
        else if (wbc.u <= 0 && wca.v <= 0) // C area
        {
            res.point = c;
            res.count = 1;
            res.contributors[0] = 2;
            return res;
        }

        const float area = cross(b - a, c - a);

        // If area == 0, 3 vertices are in collinear position, which means all aligned in a line

        const float u = cross(b - q, c - q);
        const float v = cross(c - q, a - q);
        const float w = cross(a - q, b - q);

        if (wab.u > 0 && wab.v > 0 && w * area <= 0) // On the AB edge
        {
            if (area != 0.0f)
            {
                res.count = 2;
                res.contributors[0] = 0;
                res.contributors[1] = 1;
            }
            else
            {
                res.count = 3;
                res.contributors[0] = 0;
                res.contributors[1] = 1;
                res.contributors[2] = 2;
            }

            res.point = lerp_vector(a, b, wab);
            return res;
        }
        else if (wbc.u > 0 && wbc.v > 0 && u * area <= 0) // On the BC edge
        {
            if (area != 0.0f)
            {
                res.count = 2;
                res.contributors[0] = 1;
                res.contributors[1] = 2;
            }
            else
            {
                res.count = 3;
                res.contributors[0] = 0;
                res.contributors[1] = 1;
                res.contributors[2] = 2;
            }

            res.point = lerp_vector(b, c, wbc);
            return res;
        }
        else if (wca.u > 0 && wca.v > 0 && v * area <= 0) // On the CA edge
        {
            if (area != 0.0f)
            {
                res.count = 2;
                res.contributors[0] = 2;
                res.contributors[1] = 0;
            }
            else
            {
                res.count = 3;
                res.contributors[0] = 0;
                res.contributors[1] = 1;
                res.contributors[2] = 2;
            }

            res.point = lerp_vector(c, a, wca);
            return res;
        }
        else // Inside the triangle
        {
            res.point = q;
            res.count = 0;
            return res;
        }
    }
    default:
    {
        throw std::exception("Simplex constains vertices more than 3");
    }
    };
}

} // namespace spe