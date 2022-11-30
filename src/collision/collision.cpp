#include "muli/collision.h"
#include "muli/capsule.h"
#include "muli/circle.h"
#include "muli/contact_point.h"
#include "muli/edge.h"
#include "muli/polygon.h"
#include "muli/polytope.h"
#include "muli/rigidbody.h"
#include "muli/shape.h"
#include "muli/simplex.h"

namespace muli
{

static constexpr Vec2 origin{ 0.0f };

static bool dectectionFunctionInitialized = false;
DetectionFunction* DetectionFunctionMap[Shape::Type::shape_count][Shape::Type::shape_count];
void InitializeDetectionFunctionMap();

static bool distanceFunctionInitialized = false;
DistanceFunction* DistanceFunctionMap[Shape::Type::shape_count][Shape::Type::shape_count];
void InitializeDistanceFunctionMap();

/*
 * Returns support point in 'Minkowski Difference' set
 * Minkowski Sum: A ⊕ B = {Pa + Pb| Pa ∈ A, Pb ∈ B}
 * Minkowski Difference : A ⊖ B = {Pa - Pb| Pa ∈ A, Pb ∈ B}
 * CSO stands for Configuration Space Object
 *
 * 'dir' should be normalized
 */
static Vec2 CSOSupport(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB, const Vec2& dir)
{
    Vec2 localDirA = MulT(tfA.rotation, dir);
    Vec2 localDirB = MulT(tfB.rotation, -dir);

    Vec2 supportA = tfA * a->Support(localDirA).position;
    Vec2 supportB = tfB * b->Support(localDirB).position;

    return supportA - supportB;
}

struct GJKResult
{
    Simplex simplex;
    float distance;
    bool collide;
};

static GJKResult GJK(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB, bool earlyReturn)
{
    Vec2 dir{ 1.0f, 0.0f }; // Random initial direction

    Simplex simplex;
    float distance = 0.0f;
    bool collide = false;

    Vec2 supportPoint = CSOSupport(a, tfA, b, tfB, dir);
    simplex.AddVertex(supportPoint);

    for (uint32 k = 0; k < GJK_MAX_ITERATION; ++k)
    {
        ClosestPoint closestPoint = simplex.GetClosestPoint(origin);

        if (Dist2(closestPoint.position, origin) < GJK_TOLERANCE)
        {
            collide = true;
            break;
        }
        else if (simplex.VertexCount() != 1)
        {
            // Rebuild the simplex with vertices that are used(involved) to calculate closest distance
            simplex.Shrink(closestPoint.contributors, closestPoint.count);
        }

        if (simplex.VertexCount() == 2)
        {
            // Avoid floating point error
            dir = Cross(1.0f, simplex.vertices[1] - simplex.vertices[0]).Normalized();
            distance = Dot(dir, origin - simplex.vertices[0]);
            if (distance < 0)
            {
                distance *= -1;
                dir *= -1;
            }
        }
        else
        {
            dir = origin - closestPoint.position;
            distance = dir.Normalize();
        }

        supportPoint = CSOSupport(a, tfA, b, tfB, dir);

        // If the new support point is not further along the search direction than the closest point,
        // two objects are not colliding so you can early return here.
        if (earlyReturn && distance > Dot(dir, supportPoint - closestPoint.position))
        {
            collide = false;
            break;
        }

        if (simplex.ContainsVertex(supportPoint))
        {
            collide = false;
            break;
        }
        else
        {
            simplex.AddVertex(supportPoint);
        }
    }

    return GJKResult{ simplex, distance, collide };
}

struct EPAResult
{
    Vec2 contactNormal;
    float penetrationDepth;
};

static EPAResult EPA(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB, const Simplex& simplex)
{
    Polytope polytope{ simplex };

    ClosestEdgeInfo closestEdge{ 0, FLT_MAX, Vec2{ 0.0f } };

    for (uint32 i = 0; i < EPA_MAX_ITERATION; ++i)
    {
        closestEdge = polytope.GetClosestEdge();
        const Vec2 supportPoint = CSOSupport(a, tfA, b, tfB, closestEdge.normal);
        const float newDistance = Dot(closestEdge.normal, supportPoint);

        if (Abs(closestEdge.distance - newDistance) > EPA_TOLERANCE)
        {
            // Insert the support vertex so that it expands our polytope
            polytope.vertices.Insert(closestEdge.index + 1, supportPoint);
        }
        else
        {
            // If you didn't expand edge, it means you reached the closest outer edge
            break;
        }
    }

    return EPAResult{ closestEdge.normal, closestEdge.distance };
}

static void ClipEdge(Edge* e, const Vec2& p, const Vec2& dir, bool removeClippedPoint = false)
{
    float d1 = Dot(e->p1.position - p, dir);
    float d2 = Dot(e->p2.position - p, dir);

    if (d1 >= 0 && d2 >= 0)
    {
        return;
    }

    float s = Abs(d1) + Abs(d2);

    if (d1 < 0)
    {
        if (removeClippedPoint)
        {
            e->p1 = e->p2;
        }
        else
        {
            e->p1.position = e->p1.position + (e->p2.position - e->p1.position) * (-d1 / s);
        }
    }
    else if (d2 < 0)
    {
        if (removeClippedPoint)
        {
            e->p2 = e->p1;
        }
        else
        {
            e->p2.position = e->p2.position + (e->p1.position - e->p2.position) * (-d2 / s);
        }
    }
}

static void FindContactPoints(
    const Vec2& n, Shape* a, const Transform& tfA, Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    Edge edgeA = a->GetFeaturedEdge(tfA, n);
    Edge edgeB = b->GetFeaturedEdge(tfB, -n);

    edgeA.Translate(n * a->GetRadius());
    edgeB.Translate(-n * b->GetRadius());

    Edge* ref = &edgeA; // Reference edge
    Edge* inc = &edgeB; // Incident edge
    manifold->contactNormal = n;
    manifold->featureFlipped = false;

    float aPerpendicularness = Abs(Dot(edgeA.dir, n));
    float bPerpendicularness = Abs(Dot(edgeB.dir, n));

    if (bPerpendicularness < aPerpendicularness)
    {
        ref = &edgeB;
        inc = &edgeA;
        manifold->contactNormal = -n;
        manifold->featureFlipped = true;
    }

    ClipEdge(inc, ref->p1.position, ref->dir);
    ClipEdge(inc, ref->p2.position, -ref->dir);
    ClipEdge(inc, ref->p1.position, -manifold->contactNormal, true);

    // If two points are closer than threshold, merge them into one point
    if (inc->GetLength() <= CONTACT_MERGE_THRESHOLD)
    {
        manifold->contactPoints[0] = ContactPoint{ inc->p1.position, inc->p1.id };
        manifold->numContacts = 1;
    }
    else
    {
        manifold->contactPoints[0] = ContactPoint{ inc->p1.position, inc->p1.id };
        manifold->contactPoints[1] = ContactPoint{ inc->p2.position, inc->p2.id };
        manifold->numContacts = 2;
    }

    manifold->referencePoint = ref->p1;
}

static bool CircleVsCircle(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    Vec2 pa = tfA * a->GetCenter();
    Vec2 pb = tfB * b->GetCenter();

    float d = Dist2(pa, pb);
    float r2 = a->GetRadius() + b->GetRadius();

    if (d > r2 * r2)
    {
        return false;
    }
    else
    {
        if (manifold == nullptr)
        {
            return true;
        }

        d = Sqrt(d);

        manifold->contactNormal = (pb - pa).Normalized();
        manifold->contactTangent = Vec2{ -manifold->contactNormal.y, manifold->contactNormal.x };
        manifold->contactPoints[0] = ContactPoint{ pb + (-manifold->contactNormal * b->GetRadius()), -1 };
        manifold->referencePoint = ContactPoint{ pa + (manifold->contactNormal * a->GetRadius()), -1 };
        manifold->numContacts = 1;
        manifold->penetrationDepth = r2 - d;
        manifold->featureFlipped = false;

        return true;
    }
}

static bool CapsuleVsCircle(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    Capsule* c = static_cast<Capsule*>(a);

    Vec2 pb = tfB * b->GetCenter();
    Vec2 localP = MulT(tfA, pb);

    Vec2 va = c->GetVertexA();
    Vec2 vb = c->GetVertexB();

    UV w = ComputeWeights(va, vb, localP);

    // Find closest point depending on the Voronoi region
    Vec2 normal;
    float distance;
    int32 index;

    if (w.v <= 0) // Region A: vertex collision
    {
        normal = localP - va;
        distance = normal.Normalize();
        index = 0;
    }
    else if (w.v >= 1) // Region B: vertex collision
    {
        normal = localP - vb;
        distance = normal.Normalize();
        index = 1;
    }
    else // Region AB: Edge vs. vertex collision
    {
        normal = Cross(1.0f, vb - va).Normalized();
        distance = Dot(localP - va, normal);
        if (distance < 0.0f)
        {
            normal *= -1;
            distance *= -1;
        }
        index = 0;
    }

    float r2 = a->GetRadius() + b->GetRadius();
    if (distance > r2)
    {
        return false;
    }

    if (manifold == nullptr)
    {
        return true;
    }

    normal = tfA.rotation * normal;
    Vec2 v = tfA * (index ? vb : va);

    manifold->contactNormal = normal;
    manifold->contactTangent = Vec2{ -manifold->contactNormal.y, manifold->contactNormal.x };
    manifold->penetrationDepth = r2 - distance;
    manifold->contactPoints[0] = ContactPoint{ pb + normal * -b->GetRadius(), -1 };
    manifold->referencePoint = ContactPoint{ v + normal * a->GetRadius(), index };
    manifold->numContacts = 1;
    manifold->featureFlipped = false;

    return true;
}

static bool PolygonVsCircle(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    Polygon* p = static_cast<Polygon*>(a);
    Vec2 pb = tfB * b->GetCenter();

    const Vec2* vertices = p->GetVertices();
    const Vec2* normals = p->GetNormals();
    int32 vertexCount = p->GetVertexCount();

    Vec2 localP = MulT(tfA, pb);

    float minSeparation = -FLT_MAX;
    float r2 = a->GetRadius() + b->GetRadius();

    int32 index;

    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        Vec2 n0 = normals[i0];

        float separation = Dot(n0, localP - vertices[i0]);
        if (separation > r2)
        {
            return false;
        }

        if (separation > minSeparation)
        {
            minSeparation = separation;
            index = i0;
        }

        i0 = i1;
    }

    // Circle center is inside the polygon
    if (minSeparation < 0)
    {
        if (manifold == nullptr)
        {
            return true;
        }

        Vec2 normal = tfA.rotation * normals[index];
        Vec2 v = tfA * vertices[index];

        manifold->contactNormal = normal;
        manifold->contactTangent = Vec2{ -normal.y, normal.x };
        manifold->penetrationDepth = r2 - minSeparation;
        manifold->contactPoints[0] = ContactPoint{ pb + normal * -b->GetRadius(), -1 };
        manifold->referencePoint = ContactPoint{ v + normal * a->GetRadius(), index };
        manifold->numContacts = 1;
        manifold->featureFlipped = false;

        return true;
    }

    Vec2 v0 = vertices[index];
    Vec2 v1 = vertices[(index + 1) % vertexCount];
    Vec2 normal;

    UV w = ComputeWeights(v0, v1, localP);
    float distance;

    if (w.v <= 0.0f) // Region v0
    {
        normal = localP - v0;
        distance = normal.Normalize();
    }
    else if (w.v >= 1.0f) // Region v1
    {
        normal = localP - v1;
        distance = normal.Normalize();
        index = (index + 1) % vertexCount;
    }
    else // Inside the region
    {
        normal = normals[index];
        distance = Dot(normal, localP - v0);
    }

    if (distance > r2)
    {
        return false;
    }

    if (manifold == nullptr)
    {
        return true;
    }

    normal = tfA.rotation * normal;
    Vec2 v = tfA * vertices[index];

    manifold->contactNormal = normal;
    manifold->contactTangent = Cross(1.0f, normal);
    manifold->penetrationDepth = r2 - distance;
    manifold->contactPoints[0] = ContactPoint{ pb + normal * -b->GetRadius(), -1 };
    manifold->referencePoint = ContactPoint{ v + normal * a->GetRadius(), index };
    manifold->numContacts = 1;
    manifold->featureFlipped = false;

    return true;
}

static bool ConvexVsConvex(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    GJKResult gjkResult = GJK(a, tfA, b, tfB, false);
    Simplex& simplex = gjkResult.simplex;

    float r2 = a->GetRadius() + b->GetRadius();

    if (gjkResult.collide == false)
    {
        switch (simplex.VertexCount())
        {
        case 1: // vertex vs. vertex collision
            if (gjkResult.distance < r2)
            {
                if (manifold == nullptr)
                {
                    return true;
                }

                Vec2 normal = (origin - simplex.vertices[0]).Normalized();

                Vec2 localDirA = MulT(tfA.rotation, normal);
                Vec2 localDirB = MulT(tfB.rotation, -normal);

                ContactPoint supportA = a->Support(localDirA);
                ContactPoint supportB = b->Support(localDirB);
                supportA.position = tfA * (supportA.position + localDirA * a->GetRadius());
                supportB.position = tfB * (supportB.position + localDirB * b->GetRadius());

                manifold->contactNormal = normal;
                manifold->contactTangent = Cross(1.0f, normal);
                manifold->contactPoints[0] = supportB;
                manifold->numContacts = 1;
                manifold->referencePoint = supportA;
                manifold->penetrationDepth = r2 - gjkResult.distance;
                manifold->featureFlipped = false;

                return true;
            }
            else
            {
                return false;
            }
        case 2: // vertex vs. edge collision

            [[fallthrough]];

        case 3: // Simplex vertices are in the collinear position
            if (gjkResult.distance < r2)
            {
                if (manifold == nullptr)
                {
                    return true;
                }

                Vec2 normal = Cross(1.0f, simplex.vertices[1] - simplex.vertices[0]).Normalized();
                Vec2 k = origin - simplex.vertices[0];
                if (Dot(normal, k) < 0)
                {
                    normal *= -1;
                }

                manifold->contactNormal = normal;
                manifold->penetrationDepth = r2 - gjkResult.distance;
            }
            else
            {
                return false;
            }
        }
    }
    else
    {
        if (manifold == nullptr)
        {
            return true;
        }

        // If the gjk termination simplex has vertices less than 3, expand to full simplex
        // Because EPA needs a full n-simplex to get started (actually it's pretty rare case)
        switch (simplex.VertexCount())
        {
        case 1:
        {
            Vec2 randomSupport = CSOSupport(a, tfA, b, tfB, Vec2{ 1.0f, 0.0f });
            if (randomSupport == simplex.vertices[0])
            {
                randomSupport = CSOSupport(a, tfA, b, tfB, Vec2{ -1.0f, 0.0f });
            }

            simplex.AddVertex(randomSupport);
        }

            [[fallthrough]];

        case 2:
        {
            Vec2 normal = Cross(1.0f, simplex.vertices[1] - simplex.vertices[0]).Normalized();
            Vec2 normalSupport = CSOSupport(a, tfA, b, tfB, normal);

            if (simplex.ContainsVertex(normalSupport))
            {
                simplex.AddVertex(CSOSupport(a, tfA, b, tfB, -normal));
            }
            else
            {
                simplex.AddVertex(normalSupport);
            }
        }
        }

        EPAResult epaResult = EPA(a, tfA, b, tfB, simplex);

        manifold->contactNormal = epaResult.contactNormal;
        manifold->penetrationDepth = epaResult.penetrationDepth;
    }

    FindContactPoints(manifold->contactNormal, a, tfA, b, tfB, manifold);
    manifold->contactTangent = Vec2{ -manifold->contactNormal.y, manifold->contactNormal.x };

    return true;
}

bool DetectCollision(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB, ContactManifold* manifold)
{
    if (dectectionFunctionInitialized == false)
    {
        InitializeDetectionFunctionMap();
    }

    Shape::Type shapeA = a->GetType();
    Shape::Type shapeB = b->GetType();

    if (shapeB > shapeA)
    {
        muliAssert(DetectionFunctionMap[shapeB][shapeA] != nullptr);

        bool collide = DetectionFunctionMap[shapeB][shapeA](b, tfB, a, tfA, manifold);
        if (manifold)
        {
            manifold->featureFlipped = true;
        }
        return collide;
    }
    else
    {
        muliAssert(DetectionFunctionMap[shapeA][shapeB] != nullptr);

        return DetectionFunctionMap[shapeA][shapeB](a, tfA, b, tfB, manifold);
    }
}

static float ComputeDistanceCircleVsCircle(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB)
{
    Vec2 pa = tfA * a->GetCenter();
    Vec2 pb = tfB * b->GetCenter();

    float d2 = Dist2(pa, pb);
    float r2 = a->GetRadius() + b->GetRadius();

    if (d2 > r2 * r2)
    {
        return Sqrt(d2) - r2;
    }
    else
    {
        return 0.0f;
    }
}

static float ComputeDistanceCapsuleVsCircle(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB)
{
    Vec2 pb = tfB * b->GetCenter();
    Vec2 localC = MulT(tfA, pb);

    Capsule* c = static_cast<Capsule*>(a);
    Vec2 va = c->GetVertexA();
    Vec2 vb = c->GetVertexB();

    float r2 = a->GetRadius() + b->GetRadius();

    UV w = ComputeWeights(va, vb, localC);

    float d2;
    if (w.v <= 0.0f)
    {
        d2 = Dist2(va, localC);
    }
    else if (w.v >= 1.0f)
    {
        d2 = Dist2(vb, localC);
    }
    else
    {
        Vec2 normal{ 0.0f, 1.0f };
        float d = Abs(Dot(normal, localC - va));

        if (d > r2)
        {
            return d - r2;
        }
        else
        {
            return 0.0f;
        }
    }

    if (d2 > r2 * r2)
    {
        return Sqrt(d2) - r2;
    }
    else
    {
        return 0.0f;
    }
}

static float ComputeDistancePolygonVsCircle(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB)
{
    Polygon* p = static_cast<Polygon*>(a);
    const Vec2* vertices = p->GetVertices();
    const Vec2* normals = p->GetNormals();
    int32 vertexCount = p->GetVertexCount();

    Vec2 pb = tfB * b->GetCenter();
    Vec2 localC = MulT(tfA, pb);

    float r2 = a->GetRadius() + b->GetRadius();

    UV w;
    int32 dir = 0;
    int32 i0 = 0;
    int32 insideCheck = 0;
    while (insideCheck < vertexCount)
    {
        int32 i1 = (i0 + 1) % vertexCount;

        const Vec2& v0 = vertices[i0];
        const Vec2& v1 = vertices[i1];

        w = ComputeWeights(v0, v1, localC);
        if (w.v <= 0) // Region v0
        {
            if (dir > 0)
            {
                float d2 = Dist2(v0, localC);
                if (d2 > r2 * r2)
                {
                    return Sqrt(d2) - r2;
                }
                else
                {
                    return 0.0f;
                }
            }

            dir = -1;
            i0 = (i0 - 1 + vertexCount) % vertexCount;
        }
        else if (w.v >= 1) // Region v1
        {
            if (dir < 0)
            {
                float d2 = Dist2(v1, localC);
                if (d2 > r2 * r2)
                {
                    return Sqrt(d2) - r2;
                }
                else
                {
                    return 0.0f;
                }
            }

            dir = 1;
            i0 = (i0 + 1) % vertexCount;
        }
        else // Inside the region
        {
            Vec2 normal = normals[i0];
            float d = Dot(localC - v0, normal);
            if (d >= 0)
            {
                if (d > r2)
                {
                    return d - r2;
                }
                else
                {
                    return 0.0f;
                }
            }

            if (dir != 0)
            {
                return 0.0f;
            }

            ++insideCheck;
            dir = 0;
            i0 = (i0 + 1) % vertexCount;
        }
    }

    return 0.0f;
}

static float ComputeDistanceConvexVsConvex(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB)
{
    GJKResult gr = GJK(a, tfA, b, tfB, false);

    if (gr.collide == true)
    {
        return 0.0f;
    }
    else
    {
        ClosestPoint cp = gr.simplex.GetClosestPoint(origin);

        float d2 = cp.position.Length2();
        float r2 = a->GetRadius() + b->GetRadius();

        if (d2 > r2 * r2)
        {
            return Sqrt(d2) - r2;
        }
        else
        {
            return 0.0f;
        }
    }
}

float ComputeDistance(Shape* a, const Transform& tfA, Shape* b, const Transform& tfB)
{
    if (distanceFunctionInitialized == false)
    {
        InitializeDistanceFunctionMap();
    }

    Shape::Type shapeA = a->GetType();
    Shape::Type shapeB = b->GetType();

    if (shapeB > shapeA)
    {
        muliAssert(DistanceFunctionMap[shapeB][shapeA] != nullptr);

        return DistanceFunctionMap[shapeB][shapeA](b, tfB, a, tfA);
    }
    else
    {
        muliAssert(DistanceFunctionMap[shapeA][shapeB] != nullptr);

        return DistanceFunctionMap[shapeA][shapeB](a, tfA, b, tfB);
    }
}

void InitializeDetectionFunctionMap()
{
    if (dectectionFunctionInitialized)
    {
        return;
    }

    DetectionFunctionMap[Shape::Type::circle][Shape::Type::circle] = &CircleVsCircle;

    DetectionFunctionMap[Shape::Type::capsule][Shape::Type::circle] = &CapsuleVsCircle;
    DetectionFunctionMap[Shape::Type::capsule][Shape::Type::capsule] = &ConvexVsConvex;

    DetectionFunctionMap[Shape::Type::polygon][Shape::Type::circle] = &PolygonVsCircle;
    DetectionFunctionMap[Shape::Type::polygon][Shape::Type::capsule] = &ConvexVsConvex;
    DetectionFunctionMap[Shape::Type::polygon][Shape::Type::polygon] = &ConvexVsConvex;

    dectectionFunctionInitialized = true;
}

void InitializeDistanceFunctionMap()
{
    if (distanceFunctionInitialized)
    {
        return;
    }

    DistanceFunctionMap[Shape::Type::circle][Shape::Type::circle] = &ComputeDistanceCircleVsCircle;

    DistanceFunctionMap[Shape::Type::capsule][Shape::Type::circle] = &ComputeDistanceCapsuleVsCircle;
    DistanceFunctionMap[Shape::Type::capsule][Shape::Type::capsule] = &ComputeDistanceConvexVsConvex;

    DistanceFunctionMap[Shape::Type::polygon][Shape::Type::circle] = &ComputeDistancePolygonVsCircle;
    DistanceFunctionMap[Shape::Type::polygon][Shape::Type::capsule] = &ComputeDistanceConvexVsConvex;
    DistanceFunctionMap[Shape::Type::polygon][Shape::Type::polygon] = &ComputeDistanceConvexVsConvex;

    distanceFunctionInitialized = true;
}

} // namespace muli
