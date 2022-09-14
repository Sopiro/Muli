#include "spe/collision.h"
#include "spe/box.h"
#include "spe/capsule.h"
#include "spe/circle.h"
#include "spe/edge.h"
#include "spe/polygon.h"
#include "spe/polytope.h"
#include "spe/rigidbody.h"
#include "spe/simplex.h"

#define APPLY_AXIS_WEIGHT 0

namespace spe
{
static constexpr Vec2 weightAxis{ 0.0f, 1.0f };

struct SupportPoint
{
    Vec2 position;
    int32 index;
};

// Returns the fardest vertex in the 'dir' direction
// 'dir' should be normalized and a local vector
static SupportPoint Support(Polygon* p, const Vec2& dir)
{
    const std::vector<Vec2>& vertices = p->GetVertices();

    int32 idx = 0;
    float maxValue = Dot(dir, vertices[idx]);

    for (int32 i = 1; i < vertices.size(); i++)
    {
        float value = Dot(dir, vertices[i]);
        if (value > maxValue)
        {
            idx = i;
            maxValue = value;
        }
    }

    return SupportPoint{ vertices[idx], idx };
}

/*
 * Returns support point in 'Minkowski Difference' set
 * Minkowski Sum: A ⊕ B = {Pa + Pb| Pa ∈ A, Pb ∈ B}
 * Minkowski Difference : A ⊖ B = {Pa - Pb| Pa ∈ A, Pb ∈ B}
 * CSO stands for Configuration Space Object
 */
//'dir' should be normalized
static Vec2 CSOSupport(Polygon* a, Polygon* b, const Vec2& dir)
{
    Vec2 localDirA = MulT(a->GetRotation(), dir);
    Vec2 localDirB = MulT(b->GetRotation(), -dir);

    Vec2 supportA = a->GetTransform() * Support(a, localDirA).position;
    Vec2 supportB = b->GetTransform() * Support(b, localDirB).position;

    return supportA - supportB;
}

struct GJKResult
{
    bool collide;
    Simplex simplex;
};

static GJKResult GJK(Polygon* b1, Polygon* b2, bool earlyReturn = true)
{
    constexpr Vec2 origin{ 0.0f };
    Vec2 dir(1.0f, 0.0f); // Random initial direction

    bool collide = false;
    Simplex simplex;

    Vec2 supportPoint = CSOSupport(b1, b2, dir);
    simplex.AddVertex(supportPoint);

    for (uint32 k = 0; k < GJK_MAX_ITERATION; k++)
    {
        ClosestPoint closestPoint = simplex.GetClosestPoint(origin);

        if (Dist2(closestPoint.position, origin) < GJK_TOLERANCE)
        {
            collide = true;
            break;
        }

        if (simplex.Count() != 1)
        {
            // Rebuild the simplex with vertices that are used(involved) to calculate closest distance
            simplex.Shrink(closestPoint.contributors, closestPoint.count);
        }

        dir = origin - closestPoint.position;
        float dist = dir.Normalize();

#if 0
        // Avoid floating point error
        if (simplex.Count() == 2)
        {
            Vec2 n = (simplex.vertices[0] - simplex.vertices[1]).Skew();
            dir =  Dot(dir, n) > 0 ? n.Normalized() : -n.Normalized();
        }
#endif

        supportPoint = CSOSupport(b1, b2, dir);

        // If the new support point is not further along the search direction than the closest point,
        // two objects are not colliding so you can early return here.
        if (earlyReturn && dist > Dot(dir, supportPoint - closestPoint.position))
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

    return GJKResult{ collide, simplex };
}

struct EPAResult
{
    float penetrationDepth;
    Vec2 contactNormal;
};

static EPAResult EPA(Polygon* b1, Polygon* b2, Simplex& gjkResult)
{
    Polytope polytope{ gjkResult };

    ClosestEdgeInfo closestEdge{ 0, FLT_MAX, Vec2{ 0.0f } };

    for (uint32 i = 0; i < EPA_MAX_ITERATION; i++)
    {
        closestEdge = polytope.GetClosestEdge();
        const Vec2 supportPoint = CSOSupport(b1, b2, closestEdge.normal);
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

    return EPAResult{ closestEdge.distance, closestEdge.normal };
}

static Edge FindFeaturedEdge(Polygon* b, const Vec2& dir)
{
    const Vec2 localDir = MulT(b->GetRotation(), dir);
    const SupportPoint farthest = Support(b, localDir);

    Vec2 curr = farthest.position;
    int32 idx = farthest.index;

    const Transform& t = b->GetTransform();

    const std::vector<Vec2>& vertices = b->GetVertices();
    int32 vertexCount = static_cast<int32>(b->VertexCount());

    const Vec2& prev = vertices[(idx - 1 + vertexCount) % vertexCount];
    const Vec2& next = vertices[(idx + 1) % vertexCount];

    Vec2 e1 = (curr - prev).Normalized();
    Vec2 e2 = (curr - next).Normalized();

    bool w = Dot(e1, localDir) <= Dot(e2, localDir);

    if (w)
    {
        return Edge{ t * prev, t * curr, (idx - 1 + vertexCount) % vertexCount, idx };
    }
    else
    {
        return Edge{ t * curr, t * next, idx, (idx + 1) % vertexCount };
    }
}

static void ClipEdge(Edge* edge, const Vec2& p, const Vec2& dir, bool removeClippedPoint = false)
{
    float d1 = Dot(edge->p1.position - p, dir);
    float d2 = Dot(edge->p2.position - p, dir);

    if (d1 >= 0 && d2 >= 0)
    {
        return;
    }

    float s = Abs(d1) + Abs(d2);

    if (d1 < 0)
    {
        if (removeClippedPoint)
        {
            edge->p1 = edge->p2;
        }
        else
        {
            edge->p1.position = edge->p1.position + (edge->p2.position - edge->p1.position) * (-d1 / s);
        }
    }
    else if (d2 < 0)
    {
        if (removeClippedPoint)
        {
            edge->p2 = edge->p1;
        }
        else
        {
            edge->p2.position = edge->p2.position + (edge->p1.position - edge->p2.position) * (-d2 / s);
        }
    }
}

static void FindContactPoints(const Vec2& n, Polygon* a, Polygon* b, ContactManifold* out)
{
    Edge edgeA = FindFeaturedEdge(a, n);
    Edge edgeB = FindFeaturedEdge(b, -n);

    Edge* ref = &edgeA; // Reference edge
    Edge* inc = &edgeB; // Incidence edge
    out->bodyA = a;
    out->bodyB = b;
    out->contactNormal = n;
    out->featureFlipped = false;

    float aPerpendicularness = Abs(Dot(edgeA.dir, n));
    float bPerpendicularness = Abs(Dot(edgeB.dir, n));

    if (bPerpendicularness < aPerpendicularness)
    {
        ref = &edgeB;
        inc = &edgeA;
        out->bodyA = b;
        out->bodyB = a;
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

    out->referenceEdge = *ref;
}

static bool CircleVsCircle(Circle* a, Circle* b, ContactManifold* out)
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

        out->bodyA = a;
        out->bodyB = b;
        out->contactNormal = (pb - pa).Normalized();
        out->contactPoints[0] = ContactPoint{ pb + (-out->contactNormal * b->GetRadius()), -1 };
        out->referenceEdge = Edge{ pa + (out->contactNormal * a->GetRadius()), pa + (out->contactNormal * a->GetRadius()) };
        out->numContacts = 1;
        out->penetrationDepth = r2 - d;
        out->featureFlipped = false;

        // Apply axis weight to improve coherence
        if (APPLY_AXIS_WEIGHT && Dot(out->contactNormal, weightAxis) < 0.0f)
        {
            RigidBody* tmp = out->bodyA;
            out->bodyA = out->bodyB;
            out->bodyB = tmp;
            out->contactNormal *= -1;
            out->featureFlipped = out->bodyA != a;
        }
        out->contactTangent = Vec2{ -out->contactNormal.y, out->contactNormal.x };

        return true;
    }
}

static bool ConvexVsCircle(Polygon* a, Circle* b, ContactManifold* out)
{
    const Vec2& pa = a->GetPosition();
    const Vec2& pb = b->GetPosition();

    Vec2 a2b = (pb - pa).Normalized();
    out->referenceEdge = GetFarthestEdge(a, a2b);
    UV w = ComputeWeights(out->referenceEdge.p1.position, out->referenceEdge.p2.position, b->GetPosition());

    // Find closest point depending on the Voronoi region
    Vec2 closest;
    if (w.v <= 0) // Region v1
    {
        closest = out->referenceEdge.p1.position;
        out->contactNormal = (pb - out->referenceEdge.p1.position).Normalized();
    }
    else if (w.v >= 1) // Region v2
    {
        closest = out->referenceEdge.p2.position;
        out->contactNormal = (pb - out->referenceEdge.p2.position).Normalized();
    }
    else
    {
        closest = LerpVector(out->referenceEdge.p1.position, out->referenceEdge.p2.position, w);
        out->contactNormal = -out->referenceEdge.normal;
    }

    Vec2 c2b = pb - closest;
    float l = c2b.Length();

    if (l > b->GetRadius())
    {
        return false;
    }
    else
    {
        out->bodyA = a;
        out->bodyB = b;
        out->contactTangent = out->contactNormal.Skew();
        out->penetrationDepth = b->GetRadius() - l;
        out->contactPoints[0] = ContactPoint{ b->GetPosition() + out->contactNormal * -b->GetRadius(), -1 };
        out->numContacts = 1;

        return true;
    }
}

static bool ConvexVsConvex(Polygon* a, Polygon* b, ContactManifold* out)
{
    GJKResult gjkResult = GJK(a, b);

    if (!gjkResult.collide)
    {
        return false;
    }
    else
    {
        if (out == nullptr)
        {
            return true;
        }

        // If the gjk termination simplex has vertices less than 3, expand to full simplex
        // Because EPA needs a full n-simplex to get started
        Simplex& simplex = gjkResult.simplex;
        switch (simplex.Count())
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
        case 2:
        {
            Edge e{ simplex.vertices[0], simplex.vertices[1] };
            Vec2 normalSupport = CSOSupport(a, b, e.normal);

            if (simplex.ContainsVertex(normalSupport))
            {
                simplex.AddVertex(CSOSupport(a, b, -e.normal));
            }
            else
            {
                simplex.AddVertex(normalSupport);
            }
        }
        }

        EPAResult epaResult = EPA(a, b, gjkResult.simplex);

        FindContactPoints(epaResult.contactNormal, a, b, out);

        // Apply axis weight to improve coherence
        if (APPLY_AXIS_WEIGHT && Dot(out->contactNormal, weightAxis) < 0.0f)
        {
            RigidBody* tmp = out->bodyA;
            out->bodyA = out->bodyB;
            out->bodyB = tmp;
            out->contactNormal *= -1;
            out->featureFlipped = out->bodyA != a;
        }
        out->contactTangent = Vec2{ -out->contactNormal.y, out->contactNormal.x };
        out->penetrationDepth = epaResult.penetrationDepth;

        return true;
    }
}

// Public functions

bool DetectCollision(RigidBody* a, RigidBody* b, ContactManifold* out)
{
    out->numContacts = 0;
    out->penetrationDepth = 0.0f;

    RigidBody::Shape shapeA = a->GetShape();
    RigidBody::Shape shapeB = b->GetShape();

    // Circle vs. Circle collision
    if (shapeA == RigidBody::Shape::ShapeCircle && shapeB == RigidBody::Shape::ShapeCircle)
    {
        return CircleVsCircle(static_cast<Circle*>(a), static_cast<Circle*>(b), out);
    }
    // Convex vs. Circle collision
    else if (shapeA == RigidBody::Shape::ShapePolygon && shapeB == RigidBody::Shape::ShapeCircle)
    {
        return ConvexVsCircle(static_cast<Polygon*>(a), static_cast<Circle*>(b), out);
    }
    else if (shapeA == RigidBody::Shape::ShapeCircle && shapeB == RigidBody::Shape::ShapePolygon)
    {
        return ConvexVsCircle(static_cast<Polygon*>(b), static_cast<Circle*>(a), out);
        out->featureFlipped = true;
    }
    else
    {
        return ConvexVsConvex(static_cast<Polygon*>(a), static_cast<Polygon*>(b), out);
    }
}

bool TestPointInside(RigidBody* b, const Vec2& q)
{
    RigidBody::Shape shape = b->GetShape();
    switch (shape)
    {
    case RigidBody::Shape::ShapeCircle:
    {
        Circle* c = static_cast<Circle*>(b);

        return Dist2(c->GetPosition(), q) < c->GetRadius() * c->GetRadius();
    }
    case RigidBody::Shape::ShapePolygon:
    {
        Polygon* p = static_cast<Polygon*>(b);
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
    case RigidBody::Shape::ShapeCapsule:
    {
        Capsule* c = static_cast<Capsule*>(b);

        Vec2 localQ = MulT(c->GetTransform(), q);

        float l = c->GetLength();
        float r = c->GetRadius();

        Vec2 v1 = c->GetV1();
        Vec2 v2 = c->GetV2();

        UV w = ComputeWeights(v1, v2, localQ);
        float r2 = r * r;

        if (w.v <= 0.0f)
        {
            return Dist2(v1, localQ) < r2;
        }
        else if (w.v >= 1.0f)
        {
            return Dist2(v2, localQ) < r2;
        }
        else
        {
            Vec2 p = LerpVector(v1, v2, w);
            return Dist2(p, localQ) < r2;
        }
    }
    default:
        throw std::exception("Not a supported shape");
    }
}

float ComputeDistance(RigidBody* a, RigidBody* b)
{
    RigidBody::Shape shapeA = a->GetShape();
    RigidBody::Shape shapeB = b->GetShape();

    // Circle vs. Circle distance
    if (shapeA == RigidBody::Shape::ShapeCircle && shapeB == RigidBody::Shape::ShapeCircle)
    {
        Circle* c1 = static_cast<Circle*>(a);
        Circle* c2 = static_cast<Circle*>(b);

        float d = Dist2(c1->GetPosition(), c2->GetPosition());
        float r2 = c1->GetRadius() + c2->GetRadius();

        if (d > r2 * r2)
        {
            return 0.0f;
        }
        else
        {
            return Sqrt(d);
        }
    }
    // Convex vs. Circle distance
    else if (shapeA == RigidBody::Shape::ShapePolygon && shapeB == RigidBody::Shape::ShapeCircle)
    {
        Polygon* p = static_cast<Polygon*>(a);
        Circle* c = static_cast<Circle*>(b);

        Vec2 pp = p->GetPosition();
        Vec2 cp = c->GetPosition();

        Vec2 p2c = (cp - pp).Normalized();

        Edge e = GetFarthestEdge(p, p2c);
        UV w = ComputeWeights(e.p1.position, e.p2.position, cp);

        const Vec2& closest = w.v <= 0 ? e.p1.position : (w.v >= 1 ? e.p2.position : LerpVector(e.p1.position, e.p2.position, w));

        Vec2 n = cp - closest;
        float dist2 = n.Length2();

        if (dist2 > c->GetRadius() * c->GetRadius())
        {
            return 0.0f;
        }
        else
        {
            return Sqrt(dist2);
        }
    }
    else if (shapeA == RigidBody::Shape::ShapeCircle && shapeB == RigidBody::Shape::ShapePolygon)
    {
        Polygon* p = static_cast<Polygon*>(b);
        Circle* c = static_cast<Circle*>(a);

        Vec2 pp = p->GetPosition();
        Vec2 cp = c->GetPosition();

        Vec2 p2c = (cp - pp).Normalized();

        Edge e = GetFarthestEdge(p, p2c);
        UV w = ComputeWeights(e.p1.position, e.p2.position, cp);

        const Vec2& closest = w.v <= 0 ? e.p1.position : (w.v >= 1 ? e.p2.position : LerpVector(e.p1.position, e.p2.position, w));

        Vec2 n = cp - closest;
        float dist2 = n.Length2();

        if (dist2 > c->GetRadius() * c->GetRadius())
        {
            return 0.0f;
        }
        else
        {
            return Sqrt(dist2);
        }
    }
    else
    {
        Polygon* p1 = static_cast<Polygon*>(a);
        Polygon* p2 = static_cast<Polygon*>(b);

        GJKResult gr = GJK(p1, p2, false);

        if (gr.collide)
        {
            return 0.0f;
        }
        else
        {
            ClosestPoint cp = gr.simplex.GetClosestPoint(Vec2{ 0.0f, 0.0f });
            return cp.position.Length();
        }
    }
}

float ComputeDistance(RigidBody* b, const Vec2& q)
{
    return Dist(GetClosestPoint(b, q), q);
}

Vec2 GetClosestPoint(RigidBody* b, const Vec2& q)
{
    if (TestPointInside(b, q))
    {
        return q;
    }

    RigidBody::Shape shape = b->GetShape();
    switch (shape)
    {
    case RigidBody::Shape::ShapeCircle:
    {
        Circle* c = static_cast<Circle*>(b);
        Vec2 dir = (q - b->GetPosition()).Normalized();

        return b->GetPosition() + dir * c->GetRadius();
    }
    case RigidBody::Shape::ShapePolygon:
    {
        Polygon* p = static_cast<Polygon*>(b);

        Vec2 p2q = (q - p->GetPosition()).Normalized();
        Edge e = GetFarthestEdge(p, p2q);

        UV w = ComputeWeights(e.p1.position, e.p2.position, b->GetPosition());

        Vec2 closest;
        if (w.v <= 0) // Region v1
        {
            closest = e.p1.position;
        }
        else if (w.v >= 1) // Region v2
        {
            closest = e.p2.position;
        }
        else
        {
            closest = LerpVector(e.p1.position, e.p2.position, w);
        }

        return closest;
    }
    default:
        throw std::exception("Not a supported shape");
    }
}

Edge GetFarthestEdge(Polygon* p, const Vec2& dir)
{
    const Vec2 localDir = MulT(p->GetRotation(), dir);
    const SupportPoint farthest = Support(p, localDir);

    Vec2 curr = farthest.position;
    int32 idx = farthest.index;

    const Transform& t = p->GetTransform();

    const std::vector<Vec2>& vertices = p->GetVertices();
    int32 vertexCount = static_cast<int32>(p->VertexCount());

    const Vec2& prev = vertices[(idx - 1 + vertexCount) % vertexCount];
    const Vec2& next = vertices[(idx + 1) % vertexCount];

    bool w = Cross(localDir, curr) > 0;

    if (w)
    {
        return Edge{ t * prev, t * curr, (idx - 1 + vertexCount) % vertexCount, idx };
    }
    else
    {
        return Edge{ t * curr, t * next, idx, (idx + 1) % vertexCount };
    }
}

} // namespace spe
