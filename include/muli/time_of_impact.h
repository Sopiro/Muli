#pragma once

#include "distance.h"

namespace muli
{
struct TOIInput
{
    Shape* shapeA;
    Shape* shapeB;
    Sweep sweepA;
    Sweep sweepB;
    float tMax; // defines sweep interval [0, tMax]
};

struct TOIOutput
{
    enum State
    {
        unknown,
        failed,
        overlapped,
        touching,
        separated,
    };

    State state;
    float t;
};

// clang-format off
// Bilateral advancement method by Erin Catto, the author of Box2d(https://box2d.org/)
// https://www.youtube.com/watch?v=7_nKOET6zwI
void ComputeTimeOfImpact(const Shape* shapeA, Sweep sweepA,
                         const Shape* shapeB, Sweep sweepB, 
                         float tMax,
                         TOIOutput* output);
// clang-format on

inline void ComputeTimeOfImpact(const TOIInput& input, TOIOutput* output)
{
    ComputeTimeOfImpact(input.shapeA, input.sweepA, input.shapeB, input.sweepB, input.tMax, output);
}

} // namespace muli
