#include "muli/distance.h"
#include "muli/collision.h"
#include "muli/util.h"

namespace muli
{

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

    // displace simplex vertices along normal
    Vec2 displacementA = gjkResult.direction * a->GetRadius();
    Vec2 displacementB = gjkResult.direction * -b->GetRadius();

    features->count = simplex.count;
    for (int32 i = 0; i < features->count; ++i)
    {
        features->featuresA[i] = simplex.vertices[i].pointA;
        features->featuresB[i] = simplex.vertices[i].pointB;
        features->featuresA[i].position += displacementA;
        features->featuresB[i].position += displacementB;
    }

    simplex.GetWitnessPoint(&features->pointA, &features->pointB);
    features->pointA += displacementA;
    features->pointB += displacementB;

    return gjkResult.distance - r2;
}

} // namespace muli
