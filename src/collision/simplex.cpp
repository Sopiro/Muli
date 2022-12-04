#include "muli/simplex.h"
#include "muli/util.h"

namespace muli
{

ClosestResult Simplex::GetClosestPoint(const Vec2& q) const
{
    ClosestResult res;

    switch (count)
    {
    case 1: // 0-Simplex: Point
    {
        res.point = vertices[0].point;
        res.count = 1;
        res.contributors[0] = 0;

        break;
    }
    case 2: // 1-Simplex: Line segment
    {
        Vec2 a = vertices[0].point;
        Vec2 b = vertices[1].point;
        UV w = ComputeWeights(a, b, q);

        if (w.v <= 0)
        {
            res.point = a;
            res.count = 1;
            res.contributors[0] = 0;
        }
        else if (w.v >= 1)
        {
            res.point = b;
            res.count = 1;
            res.contributors[0] = 1;
        }
        else
        {
            res.point = LerpVector(a, b, w);
            res.count = 2;
            res.contributors[0] = 0;
            res.contributors[1] = 1;
        }

        break;
    }
    case 3: // 2-Simplex: Triangle
    {
        Vec2 a = vertices[0].point;
        Vec2 b = vertices[1].point;
        Vec2 c = vertices[2].point;

        UV wab = ComputeWeights(a, b, q);
        UV wbc = ComputeWeights(b, c, q);
        UV wca = ComputeWeights(c, a, q);

        if (wca.u <= 0 && wab.v <= 0) // A area
        {
            res.point = a;
            res.count = 1;
            res.contributors[0] = 0;
            break;
        }
        else if (wab.u <= 0 && wbc.v <= 0) // B area
        {
            res.point = b;
            res.count = 1;
            res.contributors[0] = 1;
            break;
        }
        else if (wbc.u <= 0 && wca.v <= 0) // C area
        {
            res.point = c;
            res.count = 1;
            res.contributors[0] = 2;
            break;
        }

        float area = Cross(b - a, c - a);

        // If area == 0, 3 vertices are in the collinear position

        float u = Cross(b - q, c - q);
        float v = Cross(c - q, a - q);
        float w = Cross(a - q, b - q);

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

            res.point = LerpVector(a, b, wab);
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
                res.contributors[0] = 1;
                res.contributors[1] = 2;
                res.contributors[2] = 0;
            }

            res.point = LerpVector(b, c, wbc);
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
                res.contributors[0] = 2;
                res.contributors[1] = 0;
                res.contributors[2] = 1;
            }

            res.point = LerpVector(c, a, wca);
        }
        else // Inside the triangle
        {
            res.point = q;
            res.count = 0;
        }

        break;
    }
    };

    return res;
}

} // namespace muli