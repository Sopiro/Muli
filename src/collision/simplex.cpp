#include "muli/simplex.h"
#include "muli/util.h"

namespace muli
{

Vec2 Simplex::GetSearchDirection() const
{
    switch (count)
    {
    case 1:
        return -vertices[0].point;

    case 2:
    {
        // Triple product: a⨯b⨯c = b(a⋅c) - c(a⋅b)
        // In this case, ab⨯ao⨯ab = ao(ab⋅ab) - ab(ab⋅ao)
        Vec2 ab = vertices[1].point - vertices[0].point;
        Vec2 ao = -vertices[0].point;

        float d1 = Dot(ab, ab);
        float d2 = Dot(ab, ao);

        return ao * d1 - ab * d2;
    }
    default:
        muliAssert(false);
        return Vec2::zero;
    }
}

Vec2 Simplex::GetClosestPoint() const
{
    switch (count)
    {
    case 1:
        return vertices[0].point;

    case 2:
    {
        float d = 1.0f / divisor;
        return (d * vertices[0].weight) * vertices[0].point + (d * vertices[1].weight) * vertices[1].point;
    }
    case 3:
        return Vec2::zero;

    default:
        muliAssert(false);
        return Vec2::zero;
    }
}

void Simplex::GetWitnessPoint(Vec2* pointA, Vec2* pointB)
{
    switch (count)
    {
    case 1:
    {
        *pointA = vertices[0].pointA.p;
        *pointB = vertices[0].pointB.p;
        return;
    }

    case 2:
    {
        float d = 1.0f / divisor;
        *pointA = (d * vertices[0].weight) * vertices[0].pointA.p + (d * vertices[1].weight) * vertices[1].pointA.p;
        *pointB = (d * vertices[0].weight) * vertices[0].pointB.p + (d * vertices[1].weight) * vertices[1].pointB.p;
        return;
    }

    case 3:
    {
        // clang-format off
        float d = 1.0f / divisor;
        *pointA = (d * vertices[0].weight) * vertices[0].pointA.p +
                  (d * vertices[1].weight) * vertices[1].pointA.p +
                  (d * vertices[2].weight) * vertices[2].pointA.p;
        *pointB = *pointA;
        return;
        // clang-format on
    }

    default:
        muliAssert(false);
    }
}

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
        Vec2 w{ Dot(q - b, a - b), Dot(q - a, b - a) };

        // Region A
        if (w.y <= 0.0f)
        {
            count = 1;
            vertices[0].weight = 1.0f;
            divisor = 1.0f;
            return;
        }

        // Region B
        if (w.x <= 0.0f)
        {
            count = 1;
            vertices[0] = vertices[1];
            vertices[0].weight = 1.0f;
            divisor = 1.0f;
            return;
        }

        // Region AB

        count = 2;
        vertices[0].weight = w.x;
        vertices[1].weight = w.y;
        Vec2 e = b - a;
        divisor = Dot(e, e);
        return;
    }
    case 3: // 2-Simplex: Triangle
    {
        Vec2 a = vertices[0].point;
        Vec2 b = vertices[1].point;
        Vec2 c = vertices[2].point;

        Vec2 wab{ Dot(q - b, a - b), Dot(q - a, b - a) };
        Vec2 wbc{ Dot(q - c, b - c), Dot(q - b, c - b) };
        Vec2 wca{ Dot(q - a, c - a), Dot(q - c, a - c) };

        // Region A
        if (wca.x <= 0.0f && wab.y <= 0.0f)
        {
            count = 1;
            vertices[0].weight = 1.0f;
            divisor = 1.0f;
            return;
        }

        // Region B
        if (wab.x <= 0.0f && wbc.y <= 0.0f)
        {
            count = 1;
            vertices[0] = vertices[1];
            vertices[0].weight = 1.0f;
            divisor = 1.0f;
            return;
        }

        // Region C
        if (wbc.x <= 0.0f && wca.y <= 0.0f)
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
        if (wab.x > 0.0f && wab.y > 0.0f && w * area <= 0.0f)
        {
            count = 2;
            vertices[0].weight = wab.x;
            vertices[1].weight = wab.y;
            Vec2 e = b - a;
            divisor = Dot(e, e);
            return;
        }

        // Region BC
        if (wbc.x > 0.0f && wbc.y > 0.0f && u * area <= 0.0f)
        {
            count = 2;
            vertices[0] = vertices[1];
            vertices[1] = vertices[2];
            vertices[0].weight = wbc.x;
            vertices[1].weight = wbc.y;
            Vec2 e = c - b;
            divisor = Dot(e, e);
            return;
        }

        // Region CA
        if (wca.x > 0.0f && wca.y > 0.0f && v * area <= 0.0f)
        {
            count = 2;
            vertices[1] = vertices[0];
            vertices[0] = vertices[2];
            vertices[0].weight = wca.x;
            vertices[1].weight = wca.y;
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