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

} // namespace muli
