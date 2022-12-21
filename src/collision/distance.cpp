#include "muli/distance.h"
#include "muli/collision.h"
#include "muli/util.h"

namespace muli
{

float GetClosestFeatures(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ClosestFeatures* features)
{
    GJKResult gjkResult;

    bool collide = GJK(a, tfA, b, tfB, &gjkResult);
    if (collide == true)
    {
        return 0.0f;
    }

    Simplex& simplex = gjkResult.simplex;
    muliAssert(simplex.count < MAX_SIMPLEX_VERTEX_COUNT);

    features->count = simplex.count;
    for (int32 i = 0; i < features->count; ++i)
    {
        features->featuresA[i] = simplex.vertices[i].pointA;
        features->featuresB[i] = simplex.vertices[i].pointB;
    }

    return gjkResult.distance;
}

float ComputeDistance(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, Vec2* pointA, Vec2* pointB)
{
    GJKResult gjkResult;
    bool collide = GJK(a, tfA, b, tfB, &gjkResult);

    if (collide == true)
    {
        return 0.0f;
    }

    float r2 = a->GetRadius() + b->GetRadius();
    if (gjkResult.distance < r2)
    {
        return 0.0f;
    }

    Simplex& simplex = gjkResult.simplex;

    muliAssert(simplex.count < MAX_SIMPLEX_VERTEX_COUNT);

    simplex.GetWitnessPoint(pointA, pointB);

    // displace simplex vertices along normal
    *pointA += gjkResult.direction * a->GetRadius();
    *pointB -= gjkResult.direction * b->GetRadius();

    return gjkResult.distance - r2;
}

bool ShapeCast(const Shape* a,
               const Transform& tfA,
               const Shape* b,
               const Transform& tfB,
               const Vec2& translationA,
               const Vec2& translationB,
               ShapeCastOutput* output)
{
    output->point.SetZero();
    output->normal.SetZero();
    output->t = 1.0f;

    float t = 0.0f;
    Vec2 n{ 0.0f };
    float r2 = a->GetRadius() + b->GetRadius();
    Vec2 r = translationB - translationA;

    Simplex simplex;

    // Get CSO support point in -r direction
    SupportPoint support = CSOSupport(a, tfA, b, tfB, -r);
    Vec2 v = support.point;

    float target = Max(toi_position_solver_threshold, r2 - toi_position_solver_threshold);
    float tolerance = linear_slop * 0.2f;

    const int32 maxIterations = 20;
    int32 iteration = 0;

    while (iteration < maxIterations && v.Length() - target > tolerance)
    {
        muliAssert(simplex.count < 3);

        // Get CSO support point in -v direction
        support = CSOSupport(a, tfA, b, tfB, -v);

        // -v is a normal at hit point
        v.Normalize();

        // Find intersection with support plane
        float vp = Dot(v, support.point);
        float vr = Dot(v, r);
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
        vertex->pointA = support.pointA;
        vertex->pointB = support.pointB;
        vertex->pointB.position += t * r; // This effectively shifts the ray origin to the new clip point
        vertex->point = vertex->pointA.position - vertex->pointB.position;
        vertex->weight = 1.0f;
        simplex.count += 1;

        simplex.Advance(zero_vec2);

        if (simplex.count == 3)
        {
            // Overlap
            return false;
        }

        v = simplex.GetClosestPoint();

        ++iteration;
    }

    if (iteration == 0)
    {
        // Initial overlap
        return false;
    }

    Vec2 pointA, pointB;
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
