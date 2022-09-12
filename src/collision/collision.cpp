#include "spe/collision.h"
#include "spe/box.h"
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

struct SupportResult
{
    Vec2 vertex;
    int32_t index;
};

// Returns the fardest vertex in the 'dir' direction
// 'dir' should be normalized and a local vector
static SupportResult Support(Polygon* p, Vec2 dir)
{
    const std::vector<Vec2>& vertices = p->GetVertices();

    int32_t idx = 0;
    float maxValue = Dot(dir, vertices[idx]);

    for (int32_t i = 1; i < vertices.size(); i++)
    {
        float value = Dot(dir, vertices[i]);
        if (value > maxValue)
        {
            idx = i;
            maxValue = value;
        }
    }

    return SupportResult{ vertices[idx], idx };
}

/*
 * Returns support point in 'Minkowski Difference' set
 * Minkowski Sum: A ⊕ B = {Pa + Pb| Pa ∈ A, Pb ∈ B}
 * Minkowski Difference : A ⊖ B = {Pa - Pb| Pa ∈ A, Pb ∈ B}
 * CSO stands for Configuration Space Object
 */
//'dir' should be normalized
static Vec2 CSOSupport(Polygon* b1, Polygon* b2, Vec2 dir)
{
    const Vec2 localDirP1 = MulT(b1->GetRotation(), dir);
    const Vec2 localDirP2 = MulT(b2->GetRotation(), -dir);

    const Vec2 supportP1 = b1->GetTransform() * Support(b1, localDirP1).vertex;
    const Vec2 supportP2 = b2->GetTransform() * Support(b2, localDirP2).vertex;

    return supportP1 - supportP2;
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

    for (uint32_t k = 0; k < GJK_MAX_ITERATION; k++)
    {
        ClosestResult closest = simplex.GetClosest(origin);

        if (Dist2(closest.point, origin) < GJK_TOLERANCE)
        {
            collide = true;
            break;
        }

        if (simplex.Count() != 1)
        {
            // Rebuild the simplex with vertices that are used(involved) to calculate closest distance
            simplex.Shrink(closest.contributors, closest.count);
        }

        dir = origin - closest.point;
        float dist = dir.Normalize();

        supportPoint = CSOSupport(b1, b2, dir);

        // If the new support point is not further along the search direction than the closest point,
        // two objects are not colliding so you can early return here.
        if (earlyReturn && dist > Dot(dir, supportPoint - closest.point))
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

    for (uint32_t i = 0; i < EPA_MAX_ITERATION; i++)
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
    const SupportResult farthest = Support(b, localDir);

    Vec2 curr = farthest.vertex;
    int32_t idx = farthest.index;

    const Transform& t = b->GetTransform();

    const std::vector<Vec2>& vertices = b->GetVertices();
    int32_t vertexCount = static_cast<int32_t>(b->VertexCount());

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
    float d = Dist2(a->GetPosition(), b->GetPosition());
    const float r2 = a->GetRadius() + b->GetRadius();

    if (d > r2 * r2)
    {
        return false;
    }
    else
    {
        if (out == nullptr) return true;

        d = Sqrt(d);

        out->bodyA = a;
        out->bodyB = b;
        out->contactNormal = (b->GetPosition() - a->GetPosition()).Normalized();
        out->contactPoints[0] = ContactPoint{ b->GetPosition() + (-out->contactNormal * b->GetRadius()), -1 };
        out->referenceEdge = Edge{ a->GetPosition() + (out->contactNormal * a->GetRadius()),
                                   a->GetPosition() + (out->contactNormal * a->GetRadius()) };
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

    const Vec2& closest = w.v <= 0 ? out->referenceEdge.p1.position
                                   : (w.v >= 1 ? out->referenceEdge.p2.position
                                               : LerpVector(out->referenceEdge.p1.position, out->referenceEdge.p2.position, w));

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
        out->contactNormal = -out->referenceEdge.normal;
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
        if (out == nullptr) return true;

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

bool TestPointInside(RigidBody* b, const Vec2& p)
{
    Vec2 localP = MulT(b->GetTransform(), p);

    if (b->GetShape() == RigidBody::Shape::ShapeCircle)
    {
        return Length(localP) <= static_cast<Circle*>(b)->GetRadius();
    }
    else if (b->GetShape() == RigidBody::Shape::ShapePolygon)
    {
        Polygon* p = static_cast<Polygon*>(b);
        const std::vector<Vec2>& vertices = p->GetVertices();

        float dir = Cross(vertices[0] - localP, vertices[1] - localP);

        for (uint32_t i = 1; i < vertices.size(); i++)
        {
            float nDir = Cross(vertices[i] - localP, vertices[(i + 1) % vertices.size()] - localP);

            if (dir * nDir < 0)
            {
                return false;
            }
        }

        return true;
    }
    else
    {
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

        Vec2 p2c = (c->GetPosition() - p->GetPosition()).Normalized();

        Edge e = GetFarthestEdge(p, p2c);
        UV w = ComputeWeights(e.p1.position, e.p2.position, b->GetPosition());

        const Vec2& closest = w.v <= 0 ? e.p1.position : (w.v >= 1 ? e.p2.position : LerpVector(e.p1.position, e.p2.position, w));

        Vec2 d = c->GetPosition() - closest;
        float distance = d.Length();

        if (distance > c->GetRadius())
        {
            return 0.0f;
        }
        else
        {
            return distance;
        }
    }
    else if (shapeA == RigidBody::Shape::ShapeCircle && shapeB == RigidBody::Shape::ShapePolygon)
    {
        Polygon* p = static_cast<Polygon*>(b);
        Circle* c = static_cast<Circle*>(a);

        Vec2 p2c = (c->GetPosition() - p->GetPosition()).Normalized();

        Edge e = GetFarthestEdge(p, p2c);
        UV w = ComputeWeights(e.p1.position, e.p2.position, b->GetPosition());

        const Vec2& closest = w.v <= 0 ? e.p1.position : (w.v >= 1 ? e.p2.position : LerpVector(e.p1.position, e.p2.position, w));

        Vec2 d = c->GetPosition() - closest;
        float distance = d.Length();

        if (distance > c->GetRadius())
        {
            return 0.0f;
        }
        else
        {
            return distance;
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
            ClosestResult cr = gr.simplex.GetClosest({ 0.0f, 0.0f });
            return cr.point.Length();
        }
    }
}

float ComputeDistance(RigidBody* b, const Vec2& p)
{
    return Dist(GetClosestPoint(b, p), p);
}

Vec2 GetClosestPoint(RigidBody* b, const Vec2& p)
{
    Vec2 localP = MulT(b->GetTransform(), p);

    if (TestPointInside(b, localP))
    {
        return p;
    }
    else
    {
        Vec2 dir = (localP - b->GetPosition()).Normalized();

        if (b->GetType() == RigidBody::Shape::ShapeCircle)
        {
            Vec2 localR = b->GetPosition() + dir * static_cast<Circle*>(b)->GetRadius();
            return b->GetTransform() * localR;
        }
        else if (b->GetType() == RigidBody::Shape::ShapePolygon)
        {
            Polygon* poly = static_cast<Polygon*>(b);
            const std::vector<Vec2>& v = poly->GetVertices();

            SupportResult sr = Support(poly, dir);
            Simplex s;

            s.AddVertex(v[sr.index]);
            s.AddVertex(v[(sr.index + 1) % v.size()]);
            s.AddVertex(v[(sr.index + 2) % v.size()]);

            ClosestResult cr = s.GetClosest(localP);
            return b->GetTransform() * cr.point;
        }
        else
        {
            throw std::exception("Not a supported shape");
        }
    }
}

Edge GetFarthestEdge(Polygon* p, const Vec2& dir)
{
    const Vec2 localDir = MulT(p->GetRotation(), dir);
    const SupportResult farthest = Support(p, localDir);

    Vec2 curr = farthest.vertex;
    int32_t idx = farthest.index;

    const Transform& t = p->GetTransform();

    const std::vector<Vec2>& vertices = p->GetVertices();
    int32_t vertexCount = static_cast<int32_t>(p->VertexCount());

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
