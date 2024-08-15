#include "muli/raycast.h"
#include "muli/collision.h"
#include "muli/shape.h"

namespace muli
{

// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
bool RayCastCircle(const Vec2& p, float r, const RayCastInput& input, RayCastOutput* output)
{
    Vec2 d = input.to - input.from;
    Vec2 f = input.from - p;
    float radii = r + input.radius;

    float a = Dot(d, d);
    float b = 2.0f * Dot(f, d);
    float c = Dot(f, f) - radii * radii;

    // Quadratic equation discriminant
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f)
    {
        return false;
    }

    discriminant = Sqrt(discriminant);

    float t = (-b - discriminant) / (2.0f * a);
    if (0.0f <= t && t <= input.maxFraction)
    {
        output->fraction = t;
        output->normal = Normalize(f + d * t);

        return true;
    }

    return false;
}

bool RayCastLineSegment(const Vec2& v1, const Vec2& v2, const RayCastInput& input, RayCastOutput* output)
{
    Vec2 d = input.to - input.from;
    Vec2 e = v2 - v1;
    Vec2 normal = Cross(e, 1.0f);

    float length = normal.NormalizeSafe();
    if (length == 0.0f)
    {
        return false;
    }

    float denominator = Dot(normal, d);

    // Parallel or collinear case
    if (denominator == 0.0f)
    {
        return false;
    }

    float numerator = Dot(normal, v1 - input.from);
    if (denominator < 0)
    {
        numerator += input.radius;
    }
    else
    {
        numerator -= input.radius;
    }

    float t = numerator / denominator;
    if (t < 0.0f || t > input.maxFraction)
    {
        return false;
    }

    // Point on the v1-v2 line
    Vec2 q = input.from + t * d;

    float u = Dot(q - v1, e);
    if (u < 0.0f || u > Dot(e, e))
    {
        return false;
    }

    output->fraction = t;
    if (denominator > 0.0f)
    {
        output->normal = -normal;
    }
    else
    {
        output->normal = normal;
    }

    return true;
}

bool RayCastCapsule(const Vec2& va, const Vec2& vb, float radius, const RayCastInput& input, RayCastOutput* output)
{
    Vec2 p1 = input.from;
    Vec2 p2 = input.to;

    Vec2 v1 = va;
    Vec2 v2 = vb;

    Vec2 d = p2 - p1;
    Vec2 e = v2 - v1;
    Vec2 n = Cross(1.0f, e);
    n.Normalize();

    Vec2 pv = p1 - v1;

    float radii = radius + input.radius;

    // Signed distance along normal
    float distance = Dot(pv, n);

    // Does the ray start within the capsule band?
    if (Abs(distance) <= radii)
    {
        float r = Dot(e, pv);

        // Raycast to va circle
        if (r < 0.0)
        {
            // Ray cast to va circle
            Vec2 f = p1 - va;

            float a = Dot(d, d);
            float b = 2.0f * Dot(f, d);
            float c = Dot(f, f) - radii * radii;

            // Quadratic equation discriminant
            float discriminant = b * b - 4.0f * a * c;

            if (discriminant < 0.0f)
            {
                return false;
            }

            discriminant = Sqrt(discriminant);

            float t = (-b - discriminant) / (2.0f * a);
            if (0.0f <= t && t <= input.maxFraction)
            {
                output->fraction = t;
                output->normal = Normalize(f + d * t);
                return true;
            }
            else
            {
                return false;
            }
        }

        // Raycast to vb circle
        if (r > Dot(e, e))
        {
            // Raycast to vb circle
            Vec2 f = p1 - vb;

            float a = Dot(d, d);
            float b = 2.0f * Dot(f, d);
            float c = Dot(f, f) - radii * radii;

            // Quadratic equation discriminant
            float discriminant = b * b - 4.0f * a * c;

            if (discriminant < 0.0f)
            {
                return false;
            }

            discriminant = Sqrt(discriminant);

            float t = (-b - discriminant) / (2.0f * a);
            if (0.0f <= t && t <= input.maxFraction)
            {
                output->fraction = t;
                output->normal = Normalize(f + d * t);
                return true;
            }
            else
            {
                return false;
            }
        }

        // Totally inside the capsule
        return false;
    }

    Vec2 rn = n * radii;

    // Translate edge along normal
    if (distance > 0.0f)
    {
        v1 += rn;
        v2 += rn;
    }
    else
    {
        v1 -= rn;
        v2 -= rn;
    }

    // Raycast to a line segment

    float denominator = Dot(n, d);

    // Parallel or collinear case
    if (denominator == 0.0f)
    {
        return false;
    }

    float numerator = Dot(n, v1 - p1);

    float t = numerator / denominator;
    if (t < 0.0f || input.maxFraction < t)
    {
        return false;
    }

    // Point on the v1-v2 line
    Vec2 q = p1 + t * d;

    float u = Dot(q - v1, e);
    if (u < 0.0f)
    {
        // Ray cast to va circle
        Vec2 f = p1 - va;

        float a = Dot(d, d);
        float b = 2.0f * Dot(f, d);
        float c = Dot(f, f) - radii * radii;

        // Quadratic equation discriminant
        float discriminant = b * b - 4.0f * a * c;

        if (discriminant < 0.0f)
        {
            return false;
        }

        discriminant = Sqrt(discriminant);

        t = (-b - discriminant) / (2.0f * a);
        if (0.0f <= t && t <= input.maxFraction)
        {
            output->fraction = t;
            output->normal = Normalize(f + d * t);
            return true;
        }
        else
        {
            return false;
        }
    }

    if (Dot(e, e) < u)
    {
        // Raycast to vb circle
        Vec2 f = p1 - vb;

        float a = Dot(d, d);
        float b = 2.0f * Dot(f, d);
        float c = Dot(f, f) - radii * radii;

        // Quadratic equation discriminant
        float discriminant = b * b - 4.0f * a * c;

        if (discriminant < 0.0f)
        {
            return false;
        }

        discriminant = Sqrt(discriminant);

        t = (-b - discriminant) / (2.0f * a);
        if (0.0f <= t && t <= input.maxFraction)
        {
            output->fraction = t;
            output->normal = Normalize(f + d * t);
            return true;
        }
        else
        {
            return false;
        }
    }

    // Inside the edge region
    output->fraction = t;
    if (numerator > 0.0f)
    {
        output->normal = -n;
    }
    else
    {
        output->normal = n;
    }

    return true;
}

extern SupportPoint CSOSupport(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, const Vec2& dir);

bool ShapeCast(
    const Shape* a,
    const Transform& tfA,
    const Shape* b,
    const Transform& tfB,
    const Vec2& translationA,
    const Vec2& translationB,
    ShapeCastOutput* output
)
{
    output->point.SetZero();
    output->normal.SetZero();
    output->t = 1.0f;

    float t = 0.0f;
    Vec2 n = Vec2::zero;

    const float radii = a->GetRadius() + b->GetRadius();
    const Vec2 r = translationB - translationA; // Ray vector

    Simplex simplex;

    // Get CSO support point in inverse ray direction
    int32 idA = a->GetSupport(MulT(tfA.rotation, -r));
    Vec2 pointA = Mul(tfA, a->GetVertex(idA));
    int32 idB = b->GetSupport(MulT(tfB.rotation, r));
    Vec2 pointB = Mul(tfB, b->GetVertex(idB));
    Vec2 v = pointA - pointB;

    const float target = Max(default_radius, radii - toi_position_solver_threshold);
    const float tolerance = linear_slop * 0.1f;

    const int32 maxIterations = 20;
    int32 iteration = 0;

    while (iteration < maxIterations && v.Length() - target > tolerance)
    {
        MuliAssert(simplex.count < 3);

        // Get CSO support point in search direction(-v)
        idA = a->GetSupport(MulT(tfA.rotation, -v));
        pointA = Mul(tfA, a->GetVertex(idA));
        idB = b->GetSupport(MulT(tfB.rotation, v));
        pointB = Mul(tfB, b->GetVertex(idB));
        Vec2 p = pointA - pointB; // Outer vertex of CSO

        // -v is the plane normal at p
        v.Normalize();

        // Find intersection with support plane
        float vp = Dot(v, p);
        float vr = Dot(v, r);

        // March ray by (vp - target) / vr if the new t is greater
        if (vp - target > t * vr)
        {
            if (vr <= 0.0f)
            {
                return false;
            }

            t = (vp - target) / vr;
            if (t > 1.0f)
            {
                return false;
            }

            n = -v;
            simplex.count = 0;
        }

        SupportPoint* vertex = simplex.vertices + simplex.count;
        vertex->pointA.id = idA;
        vertex->pointA.p = pointA;
        vertex->pointB.id = idB;
        vertex->pointB.p = pointB + t * r; // This effectively shifts the ray origin to the new clip plane
        vertex->point = vertex->pointA.p - vertex->pointB.p;
        vertex->weight = 1.0f;
        simplex.count += 1;

        simplex.Advance(Vec2::zero);

        if (simplex.count == 3)
        {
            // Initial overlap
            return false;
        }

        // Update search direciton
        v = simplex.GetClosestPoint();

        ++iteration;
    }

    if (iteration == 0 || t == 0.0f)
    {
        // Initial overlap
        return false;
    }

    simplex.GetWitnessPoint(&pointA, &pointB);

    if (v.Length2() > 0.0f)
    {
        n = -v;
        n.Normalize();
    }

    output->point = pointA + a->GetRadius() * n + translationA * t;
    output->normal = n;
    output->t = t;
    return true;
}

} // namespace muli
