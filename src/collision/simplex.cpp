#include "muli/simplex.h"
#include "muli/util.h"

namespace muli
{

void Simplex::Advance(const Vec2& q)
{
    switch (count)
    {
    case 1: // 0-Simplex: Point
    {
        vertices[0].weight = 1.0f;
        return;
    }
    case 2: // 1-Simplex: Line segment
    {
        Vec2 a = vertices[0].point;
        Vec2 b = vertices[1].point;
        UV w{ Dot(q - b, a - b), Dot(q - a, b - a) };

        // Region A
        if (w.v <= 0.0f)
        {
            count = 1;
            vertices[0].weight = 1.0f;
            divisor = 1.0f;
            return;
        }

        // Region B
        if (w.u <= 0.0f)
        {
            count = 1;
            vertices[0] = vertices[1];
            vertices[0].weight = 1.0f;
            divisor = 1.0f;
            return;
        }

        // Region AB

        count = 2;
        vertices[0].weight = w.u;
        vertices[1].weight = w.v;
        Vec2 e = b - a;
        divisor = Dot(e, e);
        return;
    }
    case 3: // 2-Simplex: Triangle
    {
        Vec2 a = vertices[0].point;
        Vec2 b = vertices[1].point;
        Vec2 c = vertices[2].point;

        // UV wab = ComputeWeights(a, b, q);
        UV wab{ Dot(q - b, a - b), Dot(q - a, b - a) };
        UV wbc{ Dot(q - c, b - c), Dot(q - b, c - b) };
        UV wca{ Dot(q - a, c - a), Dot(q - c, a - c) };

        // Region A
        if (wca.u <= 0.0f && wab.v <= 0.0f)
        {
            count = 1;
            vertices[0].weight = 1.0f;
            divisor = 1.0f;
            return;
        }

        // Region B
        if (wab.u <= 0.0f && wbc.v <= 0.0f)
        {
            count = 1;
            vertices[0] = vertices[1];
            vertices[0].weight = 1.0f;
            divisor = 1.0f;
            return;
        }

        // Region C
        if (wbc.u <= 0.0f && wca.v <= 0.0f)
        {
            count = 1;
            vertices[0] = vertices[2];
            vertices[0].weight = 1.0f;
            divisor = 1.0f;
            return;
        }

        // 3 vertices are in the collinear position if area == 0
        float area = Cross(b - a, c - a);

        float u = Cross(b - q, c - q);
        float v = Cross(c - q, a - q);
        float w = Cross(a - q, b - q);

        // Region AB
        if (wab.u > 0.0f && wab.v > 0.0f && w * area <= 0.0f)
        {
            count = 2;
            vertices[0].weight = wab.u;
            vertices[1].weight = wab.v;
            Vec2 e = b - a;
            divisor = Dot(e, e);
            return;
        }

        // Region BC
        if (wbc.u > 0.0f && wbc.v > 0.0f && u * area <= 0.0f)
        {
            count = 2;
            vertices[0] = vertices[1];
            vertices[1] = vertices[2];
            vertices[0].weight = wbc.u;
            vertices[1].weight = wbc.v;
            Vec2 e = c - b;
            divisor = Dot(e, e);
            return;
        }

        // Region CA
        if (wca.u > 0.0f && wca.v > 0.0f && v * area <= 0.0f)
        {
            count = 2;
            vertices[0] = vertices[2];
            vertices[1] = vertices[0];
            vertices[0].weight = wca.u;
            vertices[1].weight = wca.v;
            Vec2 e = a - c;
            divisor = Dot(e, e);
            return;
        }

        // Region ABC
        count = 3;
        vertices[0].weight = u;
        vertices[1].weight = v;
        vertices[2].weight = w;
        divisor = 1.0f;
    }
    };
}

} // namespace muli