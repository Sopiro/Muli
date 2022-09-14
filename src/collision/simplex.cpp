#include "spe/simplex.h"
#include "spe/util.h"

namespace spe
{

ClosestPoint Simplex::GetClosestPoint(const Vec2& q) const
{
    ClosestPoint res;

    switch (count)
    {
    case 1: // 0-Simplex: Point
    {
        res.position = vertices[0];
        res.count = 1;
        res.contributors[0] = 0;

        break;
    }
    case 2: // 1-Simplex: Line segment
    {
        const Vec2 a = vertices[0];
        const Vec2 b = vertices[1];
        const UV w = ComputeWeights(a, b, q);

        if (w.v <= 0)
        {
            res.position = a;
            res.count = 1;
            res.contributors[0] = 0;
        }
        else if (w.v >= 1)
        {
            res.position = b;
            res.count = 1;
            res.contributors[0] = 1;
        }
        else
        {
            res.position = LerpVector(a, b, w);
            res.count = 2;
            res.contributors[0] = 0;
            res.contributors[1] = 1;
        }

        break;
    }
    case 3: // 2-Simplex: Triangle
    {
        const Vec2 a = vertices[0];
        const Vec2 b = vertices[1];
        const Vec2 c = vertices[2];

        const UV wab = ComputeWeights(a, b, q);
        const UV wbc = ComputeWeights(b, c, q);
        const UV wca = ComputeWeights(c, a, q);

        if (wca.u <= 0 && wab.v <= 0) // A area
        {
            res.position = a;
            res.count = 1;
            res.contributors[0] = 0;
            break;
        }
        else if (wab.u <= 0 && wbc.v <= 0) // B area
        {
            res.position = b;
            res.count = 1;
            res.contributors[0] = 1;
            break;
        }
        else if (wbc.u <= 0 && wca.v <= 0) // C area
        {
            res.position = c;
            res.count = 1;
            res.contributors[0] = 2;
            break;
        }

        const float area = Cross(b - a, c - a);

        // If area == 0, 3 vertices are in the collinear position

        const float u = Cross(b - q, c - q);
        const float v = Cross(c - q, a - q);
        const float w = Cross(a - q, b - q);

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

            res.position = LerpVector(a, b, wab);
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

            res.position = LerpVector(b, c, wbc);
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

            res.position = LerpVector(c, a, wca);
        }
        else // Inside the triangle
        {
            res.position = q;
            res.count = 0;
        }

        break;
    }
    };

    return res;
}

} // namespace spe