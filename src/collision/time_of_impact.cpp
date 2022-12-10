#include "muli/time_of_impact.h"

namespace muli
{

struct SeparationFunction
{
    enum Type
    {
        points,
        edgeA,
        edgeB,
    };

    void Initialize(const ClosestFeatures& closestFeatures,
                    const Shape* _shapeA,
                    const Shape* _shapeB,
                    const Sweep& _sweepA,
                    const Sweep& _sweepB,
                    float t1)
    {
        muliAssert(0 < closestFeatures.count && closestFeatures.count < MAX_SIMPLEX_VERTEX_COUNT);

        shapeA = _shapeA;
        shapeB = _shapeB;

        sweepA = _sweepA;
        sweepB = _sweepB;

        Transform tfA, tfB;
        sweepA.GetTransform(t1, &tfA);
        sweepB.GetTransform(t1, &tfB);

        int32 count = closestFeatures.count;
        const ContactPoint* featuresA = closestFeatures.featuresA;
        const ContactPoint* featuresB = closestFeatures.featuresB;

        if (count == 1)
        {
            // Point A vs. Point B
            type = points;

            // separating axis in world space
            axis = closestFeatures.pointB - closestFeatures.pointA;
            axis.Normalize();
        }
        else if (featuresA[0].id == featuresB[1].id)
        {
            // Point A vs. Edge B
            type = edgeB;

            Vec2 localPointB0 = shapeB->GetVertex(featuresB[0].id);
            Vec2 localPointB1 = shapeB->GetVertex(featuresB[1].id);

            // separating axis in frame of body B
            axis = Cross(localPointB1 - localPointB0, 1.0f);
            axis.Normalize();

            // world space face normal
            Vec2 normal = tfB.rotation * axis;

            Vec2 pointB = (featuresB[0].position + featuresB[1].position) * 0.5f;
            Vec2 pointA = featuresA[0].position;

            float separation = Dot(normal, pointA - pointB);
            if (separation < 0.0f)
            {
                axis = -axis;
            }
        }
        else
        {
            // Edge A vs. Point B
            type = edgeA;

            Vec2 localPointA0 = shapeA->GetVertex(featuresA[0].id);
            Vec2 localPointA1 = shapeA->GetVertex(featuresA[1].id);

            // separating axis in frame of body A
            axis = Cross(localPointA1 - localPointA0, 1.0f);
            axis.Normalize();

            // world space face normal
            Vec2 normal = tfA.rotation * axis;

            Vec2 pointA = (featuresA[0].position + featuresA[1].position) * 0.5f;
            Vec2 pointB = featuresB[0].position;

            float separation = Dot(normal, pointB - pointA);
            if (separation < 0.0f)
            {
                axis = -axis;
            }
        }
    }

    float FindMinSeparation(float t, int32* idA, int32* idB) const
    {
        Transform tfA, tfB;
        sweepA.GetTransform(t, &tfA);
        sweepB.GetTransform(t, &tfB);

        switch (type)
        {
        case points:
        {
            // Separation axis is in world space and pointing from a to b
            Vec2 axisA = MulT(tfA.rotation, axis);
            Vec2 axisB = MulT(tfB.rotation, -axis);

            *idA = shapeA->Support(axisA).id;
            *idB = shapeB->Support(axisB).id;

            Vec2 localPointA = shapeA->GetVertex(*idA);
            Vec2 localPointB = shapeB->GetVertex(*idB);

            Vec2 pointA = tfA * localPointA;
            Vec2 pointB = tfB * localPointB;

            float separation = Dot(axis, pointB - pointA);
            return separation;
        }
        case edgeA:
        {
            // world space face normal and refernce point
            Vec2 normal = tfA.rotation * axis;
            Vec2 pointA = tfA * localPoint;

            Vec2 localAxisB = MulT(tfB.rotation, -normal);

            *idA = -1;
            *idB = shapeB->Support(localAxisB).id;

            Vec2 localPointB = shapeB->GetVertex(*idB);
            Vec2 pointB = tfB * localPointB;

            float separation = Dot(normal, pointB - pointA);
            return separation;
        }
        case edgeB:
        {
            // world space face normal and refernce point
            Vec2 normal = tfB.rotation * axis;
            Vec2 pointB = tfB * localPoint;

            Vec2 localAxisA = MulT(tfA.rotation, -normal);

            *idA = shapeA->Support(localAxisA).id;
            *idB = -1;

            Vec2 localPointA = shapeA->GetVertex(*idA);
            Vec2 pointA = tfA * localPointA;

            float separation = Dot(normal, pointA - pointB);
            return separation;
        }
        }
    }

    float ComputeSeparation(int32 idA, int32 idB, float t) const
    {
        Transform tfA, tfB;
        sweepA.GetTransform(t, &tfA);
        sweepB.GetTransform(t, &tfB);

        switch (type)
        {
        case points:
        {
            Vec2 localPointA = shapeA->GetVertex(idA);
            Vec2 localPointB = shapeA->GetVertex(idB);

            Vec2 pointA = tfA * localPointA;
            Vec2 pointB = tfB * localPointB;

            float separation = Dot(axis, pointB - pointA);
            return separation;
        }
        case edgeA:
        {
            Vec2 normal = tfA.rotation * axis;

            Vec2 pointA = tfA * localPoint;

            Vec2 localPointB = shapeB->GetVertex(idB);
            Vec2 pointB = tfB * localPointB;

            float separation = Dot(normal, pointB - pointA);
            return separation;
        }
        case edgeB:
        {
            Vec2 normal = tfB.rotation * axis;

            Vec2 pointB = tfB * localPoint;

            Vec2 localPointA = shapeA->GetVertex(idA);
            Vec2 pointA = tfA * localPointA;

            float separation = Dot(normal, pointA - pointB);
            return separation;
        }
        }
    }

    const Shape* shapeA;
    const Shape* shapeB;
    Sweep sweepA;
    Sweep sweepB;
    Type type;
    Vec2 localPoint;
    Vec2 axis;
};

} // namespace muli
