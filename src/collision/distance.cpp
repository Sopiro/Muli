#include "muli/distance.h"
#include "muli/util.h"

namespace muli
{

struct GJKResult
{
    Simplex simplex;
    Vec2 direction;
    float distance;
};

extern bool GJK(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, GJKResult* result);

float ComputeDistance(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ClosestFeatures* features)
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

    features->count = simplex.count;
    for (int32 i = 0; i < features->count; ++i)
    {
        features->fA[i] = simplex.vertices[i].pointA;
        features->fB[i] = simplex.vertices[i].pointB;
        features->fA[i].position += gjkResult.direction * a->GetRadius();
        features->fB[i].position -= gjkResult.direction * b->GetRadius();
    }

    return gjkResult.distance - r2;
}

} // namespace muli
