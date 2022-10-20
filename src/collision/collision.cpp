#include "muli/collision.h"
#include "muli/capsule.h"
#include "muli/circle.h"
#include "muli/contact_point.h"
#include "muli/edge.h"
#include "muli/polygon.h"
#include "muli/polytope.h"
#include "muli/rigidbody.h"
#include "muli/simplex.h"

namespace muli
{

static constexpr Vec2 origin{ 0.0f };

static bool dectectionFunctionInitialized = false;
DetectionFunction* DetectionFunctionMap[RigidBody::Shape::ShapeCount][RigidBody::Shape::ShapeCount];
void InitializeDetectionFunctionMap();

static bool distanceFunctionInitialized = false;
DistanceFunction* DistanceFunctionMap[RigidBody::Shape::ShapeCount][RigidBody::Shape::ShapeCount];
void InitializeDistanceFunctionMap();

/*
 * Returns support point in 'Minkowski Difference' set
 * Minkowski Sum: A ⊕ B = {Pa + Pb| Pa ∈ A, Pb ∈ B}
 * Minkowski Difference : A ⊖ B = {Pa - Pb| Pa ∈ A, Pb ∈ B}
 * CSO stands for Configuration Space Object
 *
 * 'dir' should be normalized
 */
static Vec2 CSOSupport(RigidBody* a, RigidBody* b, const Vec2& dir)
{
    Vec2 localDirA = MulT(a->GetRotation(), dir);
    Vec2 localDirB = MulT(b->GetRotation(), -dir);

    Vec2 supportA = a->GetTransform() * a->Support(localDirA).position;
    Vec2 supportB = b->GetTransform() * b->Support(localDirB).position;

    return supportA - supportB;
}

struct GJKResult
{
    Simplex simplex;
    float distance;
    bool collide;
};

static GJKResult GJK(RigidBody* a, RigidBody* b, bool earlyReturn)
{
    Vec2 dir{ 1.0f, 0.0f }; // Random initial direction

    Simplex simplex;
    float distance = 0.0f;
    bool collide = false;

    Vec2 supportPoint = CSOSupport(a, b, dir);
    simplex.AddVertex(supportPoint);

    for (uint32 k = 0; k < GJK_MAX_ITERATION; k++)
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
            dir = (simplex.vertices[1] - simplex.vertices[0]).Normalized().Skew();
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

        supportPoint = CSOSupport(a, b, dir);

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

static EPAResult EPA(RigidBody* a, RigidBody* b, Simplex& gjkResult)
{
    Polytope polytope{ gjkResult };

    ClosestEdgeInfo closestEdge{ 0, FLT_MAX, Vec2{ 0.0f } };

    for (uint32 i = 0; i < EPA_MAX_ITERATION; i++)
    {
        closestEdge = polytope.GetClosestEdge();
        const Vec2 supportPoint = CSOSupport(a, b, closestEdge.normal);
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

static void FindContactPoints(const Vec2& n, RigidBody* a, RigidBody* b, ContactManifold* out)
{
    Edge edgeA = a->GetFeaturedEdge(n);
    Edge edgeB = b->GetFeaturedEdge(-n);

    edgeA.Translate(n * a->GetRadius());
    edgeB.Translate(-n * b->GetRadius());

    Edge* ref = &edgeA; // Reference edge
    Edge* inc = &edgeB; // Incident edge
    out->contactNormal = n;
    out->featureFlipped = false;

    float aPerpendicularness = Abs(Dot(edgeA.dir, n));
    float bPerpendicularness = Abs(Dot(edgeB.dir, n));

    if (bPerpendicularness < aPerpendicularness)
    {
        ref = &edgeB;
        inc = &edgeA;
        out->contactNormal = -n;
        out->featureFlipped = true;
    }

    ClipEdge(inc, ref->p1.position, ref->dir);
    ClipEdge(inc, ref->p2.position, -ref->dir);
    ClipEdge(inc, ref->p1.position, -out->contactNormal, true);

    // If two points are closer than threshold, merge them into one point
    if (inc->GetLength() <= CONTACT_MERGE_THRESHOLD)
    {
        out->contactPoints[0] = ContactPoint{ inc->p1.position, inc->p1.id };
        out->numContacts = 1;
    }
    else
    {
        out->contactPoints[0] = ContactPoint{ inc->p1.position, inc->p1.id };
        out->contactPoints[1] = ContactPoint{ inc->p2.position, inc->p2.id };
        out->numContacts = 2;
    }

    out->referencePoint = ref->p1;
}

static bool CircleVsCircle(RigidBody* a, RigidBody* b, ContactManifold* out)
{
    Vec2 pa = a->GetPosition();
    Vec2 pb = b->GetPosition();

    float d = Dist2(pa, pb);
    const float r2 = a->GetRadius() + b->GetRadius();

    if (d > r2 * r2)
    {
        return false;
    }
    else
    {
        if (out == nullptr)
        {
            return true;
        }

        d = Sqrt(d);

        out->contactNormal = (pb - pa).Normalized();
        out->contactTangent = Vec2{ -out->contactNormal.y, out->contactNormal.x };
        out->contactPoints[0] = ContactPoint{ pb + (-out->contactNormal * b->GetRadius()), -1 };
        out->referencePoint = ContactPoint{ pa + (out->contactNormal * a->GetRadius()), -1 };
        out->numContacts = 1;
        out->penetrationDepth = r2 - d;
        out->featureFlipped = false;

        return true;
    }
}

static bool CapsuleVsCircle(RigidBody* a, RigidBody* b, ContactManifold* out)
{
    const Vec2& pa = a->GetPosition();
    const Vec2& pb = b->GetPosition();

    Vec2 a2b = (pb - pa).Normalized();
    const Vec2 localDir = MulT(a->GetRotation(), a2b);
    const Transform& t = a->GetTransform();

    Edge e = Edge{ t * static_cast<Capsule*>(a)->GetVertexA(), t * static_cast<Capsule*>(a)->GetVertexB(), 0, 1 };

    UV w = ComputeWeights(e.p1.position, e.p2.position, b->GetPosition());

    // Find closest point depending on the Voronoi region
    Vec2 normal;
    float distance;
    ContactPoint rp;
    if (w.v <= 0) // Region A: vertex collision
    {
        normal = pb - e.p1.position;
        distance = normal.Normalize();
        rp = e.p1;
    }
    else if (w.v >= 1) // Region B: vertex collision
    {
        normal = pb - e.p2.position;
        distance = normal.Normalize();
        rp = e.p2;
    }
    else // Region AB: Edge vs. vertex collision
    {
        normal = e.normal;
        distance = Dot(pb - e.p1.position, normal);
        if (distance < 0.0f)
        {
            normal *= -1;
            distance *= -1;
        }
        rp = e.p1;
    }

    float r2 = a->GetRadius() + b->GetRadius();
    if (distance > r2)
    {
        return false;
    }
    else
    {
        if (out == nullptr)
        {
            return true;
        }

        out->contactNormal = normal;
        out->contactTangent = Vec2{ -out->contactNormal.y, out->contactNormal.x };
        out->penetrationDepth = r2 - distance;
        out->contactPoints[0] = ContactPoint{ pb + normal * -b->GetRadius(), -1 };
        out->referencePoint = ContactPoint{ rp.position + normal * a->GetRadius(), rp.id };
        out->numContacts = 1;
        out->featureFlipped = false;

        return true;
    }
}

static bool PolygonVsCircle(RigidBody* a, RigidBody* b, ContactManifold* out)
{
    const Vec2& pa = a->GetPosition();
    const Vec2& pb = b->GetPosition();

    Vec2 a2b = (pb - pa).Normalized();
    Edge e = GetIntersectingEdge(static_cast<Polygon*>(a), a2b);
    UV w = ComputeWeights(e.p1.position, e.p2.position, b->GetPosition());

    // Find closest point depending on the Voronoi region
    Vec2 closest;
    Vec2 normal;
    if (w.v <= 0) // Region A: vertex collision
    {
        closest = e.p1.position;
        normal = (pb - e.p1.position).Normalized();
    }
    else if (w.v >= 1) // Region B: vertex collision
    {
        closest = e.p2.position;
        normal = (pb - e.p2.position).Normalized();
    }
    else // Region AB: Edge vs. vertex collision
    {
        closest = LerpVector(e.p1.position, e.p2.position, w);
        normal = -e.normal;
    }

    Vec2 c2b = pb - closest;

    // Signed distance along the normal
    float l = Dot(c2b, normal);
    float r2 = a->GetRadius() + b->GetRadius();

    if (l > r2)
    {
        return false;
    }
    else
    {
        if (out == nullptr)
        {
            return true;
        }

        out->contactNormal = normal;
        out->contactTangent = Vec2{ -out->contactNormal.y, out->contactNormal.x };
        out->penetrationDepth = r2 - l;
        out->contactPoints[0] = ContactPoint{ pb + normal * -b->GetRadius(), -1 };
        out->referencePoint = e.p1;
        out->numContacts = 1;
        out->featureFlipped = false;

        return true;
    }
}

static bool ConvexVsConvex(RigidBody* a, RigidBody* b, ContactManifold* out)
{
    GJKResult gjkResult = GJK(a, b, false);
    Simplex& simplex = gjkResult.simplex;

    float r2 = a->GetRadius() + b->GetRadius();

    if (gjkResult.collide == false)
    {
        switch (simplex.VertexCount())
        {
        case 1: // vertex vs. vertex collision
            if (gjkResult.distance < r2)
            {
                if (out == nullptr)
                {
                    return true;
                }

                out->contactNormal = (origin - simplex.vertices[0]).Normalized();

                Vec2 localDirA = MulT(a->GetRotation(), out->contactNormal);
                Vec2 localDirB = MulT(b->GetRotation(), -out->contactNormal);

                ContactPoint supportA = a->Support(localDirA);
                ContactPoint supportB = b->Support(localDirB);
                supportA.position = a->GetTransform() * (supportA.position + localDirA * a->GetRadius());
                supportB.position = b->GetTransform() * (supportB.position + localDirB * b->GetRadius());

                out->contactTangent = out->contactNormal.Skew();
                out->contactPoints[0] = supportB;
                out->numContacts = 1;
                out->referencePoint = supportA;
                out->penetrationDepth = r2 - gjkResult.distance;
                out->featureFlipped = false;

                return true;
            }
            else
            {
                return false;
            }
        case 2: // vertex vs. edge collision
        case 3: // Simplex vertices are in the collinear position
            if (gjkResult.distance < r2)
            {
                if (out == nullptr)
                {
                    return true;
                }

                out->contactNormal = (simplex.vertices[1] - simplex.vertices[0]).Normalized().Skew();
                Vec2 k = origin - simplex.vertices[0];
                if (Dot(out->contactNormal, k) < 0)
                {
                    out->contactNormal *= -1;
                }
                out->penetrationDepth = r2 - gjkResult.distance;
            }
            else
            {
                return false;
            }
        }
    }
    else
    {
        if (out == nullptr)
        {
            return true;
        }

        // If the gjk termination simplex has vertices less than 3, expand to full simplex
        // Because EPA needs a full n-simplex to get started (actually it's pretty rare case)
        switch (simplex.VertexCount())
        {
        case 1:
        {
            const Vec2& v = simplex.vertices[0];
            Vec2 randomSupport = CSOSupport(a, b, Vec2{ 1.0f, 0.0f });

            if (randomSupport == v)
            {
                randomSupport = CSOSupport(a, b, Vec2{ -1.0f, 0.0f });
            }

            simplex.AddVertex(randomSupport);
        }

            [[fallthrough]];

        case 2:
        {
            Vec2 n = (simplex.vertices[1] - simplex.vertices[0]).Normalized().Skew();
            Vec2 normalSupport = CSOSupport(a, b, n);

            if (simplex.ContainsVertex(normalSupport))
            {
                simplex.AddVertex(CSOSupport(a, b, -n));
            }
            else
            {
                simplex.AddVertex(normalSupport);
            }
        }
        }

        EPAResult epaResult = EPA(a, b, simplex);
        out->contactNormal = epaResult.contactNormal;
        out->penetrationDepth = epaResult.penetrationDepth;
    }

    FindContactPoints(out->contactNormal, a, b, out);
    out->contactTangent = Vec2{ -out->contactNormal.y, out->contactNormal.x };

    return true;
}

bool DetectCollision(RigidBody* a, RigidBody* b, ContactManifold* out)
{
    if (dectectionFunctionInitialized == false)
    {
        InitializeDetectionFunctionMap();
    }

    RigidBody::Shape shapeA = a->GetShape();
    RigidBody::Shape shapeB = b->GetShape();

    if (shapeB > shapeA)
    {
        muliAssert(DetectionFunctionMap[shapeB][shapeA] != nullptr);

        bool collide = DetectionFunctionMap[shapeB][shapeA](b, a, out);
        if (out)
        {
            out->featureFlipped = true;
        }
        return collide;
    }
    else
    {
        muliAssert(DetectionFunctionMap[shapeA][shapeB] != nullptr);

        return DetectionFunctionMap[shapeA][shapeB](a, b, out);
    }
}

static float ComputeDistanceCircleVsCircle(RigidBody* a, RigidBody* b)
{
    float d2 = Dist2(a->GetPosition(), b->GetPosition());
    float r2 = a->GetRadius() + b->GetRadius();

    if (d2 > r2 * r2)
    {
        return 0.0f;
    }
    else
    {
        return Sqrt(d2);
    }
}

static float ComputeDistanceCapsuleVsCircle(RigidBody* a, RigidBody* b)
{
    Vec2 localP = MulT(a->GetTransform(), b->GetPosition());

    Capsule* c = static_cast<Capsule*>(a);
    const Vec2& va = c->GetVertexA();
    const Vec2& vb = c->GetVertexB();

    UV w = ComputeWeights(va, vb, localP);

    const Vec2& closest = w.v <= 0.0f ? va : (w.v >= 1.0f ? vb : LerpVector(va, vb, w));

    float d2 = Dist2(closest, localP);
    float r2 = a->GetRadius() + b->GetRadius();

    if (d2 > r2 * r2)
    {
        return 0.0f;
    }
    else
    {
        return Sqrt(d2);
    }
}

static float ComputeDistancePolygonVsCircle(RigidBody* a, RigidBody* b)
{
    Vec2 pa = a->GetPosition();
    Vec2 pb = b->GetPosition();

    Vec2 a2b = (pb - pa).Normalized();

    Edge e = GetIntersectingEdge(static_cast<Polygon*>(a), a2b);
    UV w = ComputeWeights(e.p1.position, e.p2.position, pb);

    const Vec2& closest =
        w.v <= 0.0f ? e.p1.position : (w.v >= 1.0f ? e.p2.position : LerpVector(e.p1.position, e.p2.position, w));

    Vec2 n = pb - closest;
    float d2 = n.Length2();
    float r2 = a->GetRadius() + b->GetRadius();

    if (d2 > r2 * r2)
    {
        return 0.0f;
    }
    else
    {
        return Sqrt(d2);
    }
}

static float ComputeDistanceConvexVsConvex(RigidBody* a, RigidBody* b)
{
    GJKResult gr = GJK(a, b, false);

    if (gr.collide == true)
    {
        return 0.0f;
    }
    else
    {
        ClosestPoint cp = gr.simplex.GetClosestPoint(origin);

        float d = cp.position.Length();

        if (d <= a->GetRadius() + b->GetRadius())
        {
            return 0.0f;
        }
        else
        {
            return d;
        }
    }
}

float ComputeDistance(RigidBody* a, RigidBody* b)
{
    if (distanceFunctionInitialized == false)
    {
        InitializeDistanceFunctionMap();
    }

    RigidBody::Shape shapeA = a->GetShape();
    RigidBody::Shape shapeB = b->GetShape();

    if (shapeB > shapeA)
    {
        muliAssert(DistanceFunctionMap[shapeB][shapeA] != nullptr);

        return DistanceFunctionMap[shapeB][shapeA](b, a);
    }
    else
    {
        muliAssert(DistanceFunctionMap[shapeA][shapeB] != nullptr);

        return DistanceFunctionMap[shapeA][shapeB](a, b);
    }
}

float ComputeDistance(RigidBody* b, Vec2 q)
{
    return Dist(GetClosestPoint(b, q), q);
}

bool TestPointInside(Circle* c, Vec2 q)
{
    return Dist2(c->GetPosition(), q) < c->GetRadius() * c->GetRadius();
}

bool TestPointInside(Capsule* c, Vec2 q)
{
    Vec2 localQ = MulT(c->GetTransform(), q);

    float l = c->GetLength();
    float r = c->GetRadius();

    Vec2 va = c->GetVertexA();
    Vec2 vb = c->GetVertexB();

    UV w = ComputeWeights(va, vb, localQ);
    float r2 = r * r;

    if (w.v <= 0.0f) // Region A
    {
        return Dist2(va, localQ) < r2;
    }
    else if (w.v >= 1.0f) // Region B
    {
        return Dist2(vb, localQ) < r2;
    }
    else // Region AB
    {
        Vec2 p = LerpVector(va, vb, w);
        return Dist2(p, localQ) < r2;
    }
}

bool TestPointInside(Polygon* p, Vec2 q)
{
    const std::vector<Vec2>& vertices = p->GetVertices();

    Vec2 localQ = MulT(p->GetTransform(), q);

    float dir = Cross(vertices[0] - localQ, vertices[1] - localQ);
    for (uint32 i = 1; i < vertices.size(); i++)
    {
        float nDir = Cross(vertices[i] - localQ, vertices[(i + 1) % vertices.size()] - localQ);

        if (dir * nDir < 0)
        {
            return false;
        }
    }

    return true;
}

bool TestPointInside(RigidBody* b, Vec2 q)
{
    RigidBody::Shape shape = b->GetShape();
    switch (shape)
    {
    case RigidBody::Shape::ShapeCircle:
    {
        return TestPointInside(static_cast<Circle*>(b), q);
    }
    case RigidBody::Shape::ShapeCapsule:
    {
        return TestPointInside(static_cast<Capsule*>(b), q);
    }
    case RigidBody::Shape::ShapePolygon:
    {
        return TestPointInside(static_cast<Polygon*>(b), q);
    }
    default:
        throw std::runtime_error("Not a supported shape");
    }
}

Vec2 GetClosestPoint(Circle* b, Vec2 q)
{
    float radius = b->GetRadius();

    Vec2 dir = (q - b->GetPosition());
    float distance = dir.Normalize();

    if (distance <= radius)
    {
        return q;
    }
    else
    {
        return b->GetPosition() + dir * radius;
    }
}

Vec2 GetClosestPoint(Capsule* c, Vec2 q)
{
    float radius = c->GetRadius();

    const Vec2& va = c->GetVertexA();
    const Vec2& vb = c->GetVertexB();

    Vec2 localQ = MulT(c->GetTransform(), q);
    UV w = ComputeWeights(va, vb, localQ);

    Vec2 closest;
    Vec2 normal;
    float distance;
    if (w.v <= 0) // Region A
    {
        closest = va;
        normal = localQ - va;
        distance = normal.Normalize();
    }
    else if (w.v >= 1) // Region B
    {
        closest = vb;
        normal = localQ - vb;
        distance = normal.Normalize();
    }
    else // Region AB
    {
        normal.Set(0.0f, 1.0f);
        distance = Dot(localQ - va, normal);

        if (Dot(normal, localQ - va) < 0.0f)
        {
            normal *= -1;
            distance *= -1;
        }

        closest = localQ + normal * -distance;
    }

    if (distance <= radius)
    {
        return q;
    }
    else
    {
        closest += normal * radius;
        return c->GetTransform() * closest;
    }
}

Vec2 GetClosestPoint(Polygon* p, Vec2 q)
{
    float radius = p->GetRadius();
    Vec2 localQ = MulT(p->GetTransform(), q);
    const std::vector<Vec2>& vertices = p->GetVertices();
    int32 vertexCount = static_cast<int32>(vertices.size());

    UV w;
    Vec2 normal;
    int32 dir = 0;
    int32 i0 = 0;
    int32 insideCheck = 0;

    while (insideCheck < vertexCount)
    {
        int32 i1 = (i0 + 1) % vertexCount;

        const Vec2& v0 = vertices[i0];
        const Vec2& v1 = vertices[i1];

        w = ComputeWeights(v0, v1, localQ);
        if (w.v <= 0) // Region v0
        {
            if (dir > 0)
            {
                normal = (localQ - v0).Normalized();
                return p->GetTransform() * (v0 + normal * radius);
            }

            dir = -1;
            i0 = (i0 - 1 + vertexCount) % vertexCount;
        }
        else if (w.v >= 1) // Region v1
        {
            if (dir < 0)
            {
                normal = (localQ - v1).Normalized();
                return p->GetTransform() * (v1 + normal * radius);
            }

            dir = 1;
            i0 = (i0 + 1) % vertexCount;
        }
        else // Inside the region
        {
            normal = Cross(v1 - v0, 1.0f).Normalized();
            float distance = Dot(localQ - v0, normal);
            if (distance >= 0)
            {
                Vec2 closest = localQ + normal * (radius - distance);
                return p->GetTransform() * closest;
            }

            if (dir != 0)
            {
                return q;
            }

            ++insideCheck;
            dir = 0;
            i0 = (i0 + 1) % vertexCount;
        }
    }

    return q;
}

Vec2 GetClosestPoint(RigidBody* b, Vec2 q)
{
    RigidBody::Shape shape = b->GetShape();
    switch (shape)
    {
    case RigidBody::Shape::ShapeCircle:
    {
        return GetClosestPoint(static_cast<Circle*>(b), q);
    }
    case RigidBody::Shape::ShapeCapsule:
    {
        return GetClosestPoint(static_cast<Capsule*>(b), q);
    }
    case RigidBody::Shape::ShapePolygon:
    {
        return GetClosestPoint(static_cast<Polygon*>(b), q);
    }
    default:
        throw std::runtime_error("Not a supported shape");
    }
}

Edge GetIntersectingEdge(Polygon* p, Vec2 dir)
{
    const Vec2 localDir = MulT(p->GetRotation(), dir);
    const std::vector<Vec2>& vertices = p->GetVertices();

    for (int32 i = 0; i < vertices.size(); i++)
    {
        int32 i2 = (i + 1) % vertices.size();
        const Vec2& v1 = vertices[i];
        const Vec2& v2 = vertices[i2];

        if (Cross(v1, localDir) > 0 && Cross(v2, localDir) < 0)
        {
            const Transform& t = p->GetTransform();
            return Edge{ t * v1, t * v2, i, i2 };
        }
    }

    throw std::runtime_error("Unreachable");
}

void InitializeDetectionFunctionMap()
{
    if (dectectionFunctionInitialized)
    {
        return;
    }

    DetectionFunctionMap[RigidBody::Shape::ShapeCircle][RigidBody::Shape::ShapeCircle] = &CircleVsCircle;

    DetectionFunctionMap[RigidBody::Shape::ShapeCapsule][RigidBody::Shape::ShapeCircle] = &CapsuleVsCircle;
    DetectionFunctionMap[RigidBody::Shape::ShapeCapsule][RigidBody::Shape::ShapeCapsule] = &ConvexVsConvex;

    DetectionFunctionMap[RigidBody::Shape::ShapePolygon][RigidBody::Shape::ShapeCircle] = &PolygonVsCircle;
    DetectionFunctionMap[RigidBody::Shape::ShapePolygon][RigidBody::Shape::ShapeCapsule] = &ConvexVsConvex;
    DetectionFunctionMap[RigidBody::Shape::ShapePolygon][RigidBody::Shape::ShapePolygon] = &ConvexVsConvex;

    dectectionFunctionInitialized = true;
}

void InitializeDistanceFunctionMap()
{
    if (distanceFunctionInitialized)
    {
        return;
    }

    DistanceFunctionMap[RigidBody::Shape::ShapeCircle][RigidBody::Shape::ShapeCircle] = &ComputeDistanceCircleVsCircle;

    DistanceFunctionMap[RigidBody::Shape::ShapeCapsule][RigidBody::Shape::ShapeCircle] = &ComputeDistanceCapsuleVsCircle;
    DistanceFunctionMap[RigidBody::Shape::ShapeCapsule][RigidBody::Shape::ShapeCapsule] = &ComputeDistanceConvexVsConvex;

    DistanceFunctionMap[RigidBody::Shape::ShapePolygon][RigidBody::Shape::ShapeCircle] = &ComputeDistancePolygonVsCircle;
    DistanceFunctionMap[RigidBody::Shape::ShapePolygon][RigidBody::Shape::ShapeCapsule] = &ComputeDistanceConvexVsConvex;
    DistanceFunctionMap[RigidBody::Shape::ShapePolygon][RigidBody::Shape::ShapePolygon] = &ComputeDistanceConvexVsConvex;

    distanceFunctionInitialized = true;
}

// Compute signed distance between polygons
static float ComputeSeparation(const std::vector<Vec2>& va, const std::vector<Vec2>& vb)
{
    float maxSeparation = -FLT_MAX;

    for (uint32 i = 0; i < va.size(); i++)
    {
        const Vec2& va0 = va[i];
        const Vec2& va1 = va[(i + 1) % va.size()];

        // Right-hand coordinate system, CCW vertex winding
        Vec2 normal = Cross((va1 - va0).Normalized(), 1.0f);
        float separation = FLT_MAX;

        for (uint32 j = 0; j < vb.size(); j++)
        {
            const Vec2& vb0 = vb[j];

            separation = Min(separation, Dot(normal, vb0 - va0));
        }

        maxSeparation = Max(separation, maxSeparation);
    }

    return maxSeparation;
}

bool SAT(Polygon* a, Polygon* b)
{
    const std::vector<Vec2>& va = a->GetVertices();
    const std::vector<Vec2>& vb = b->GetVertices();

    std::vector<Vec2> wva(va.size());
    std::vector<Vec2> wvb(vb.size());

    std::transform(va.begin(), va.end(), wva.begin(), [&](const Vec2& v) { return a->GetTransform() * v; });
    std::transform(vb.begin(), vb.end(), wvb.begin(), [&](const Vec2& v) { return b->GetTransform() * v; });

    return ComputeSeparation(wva, wvb) < 0 && ComputeSeparation(wvb, wva) < 0;
}

} // namespace muli
