#include <algorithm>
#include <random>

#include "muli/geometry.h"

namespace muli
{

void ComputeConvexHull(const Vec2* vertices, int32 vertexCount, Vec2* outVertices, int32* outVertexCount)
{
    if (vertexCount < 3)
    {
        memcpy(outVertices, vertices, vertexCount * sizeof(Vec2));
        *outVertexCount = vertexCount;
        return;
    }

    // Find the lowest vertex
    int32 index = 0;
    for (int32 i = 1; i < vertexCount; ++i)
    {
        if (vertices[i].y < vertices[index].y)
        {
            index = i;
        }
    }
    Vec2 bottom = vertices[index];

    Vec2* sorted = new Vec2[vertexCount];
    memcpy(sorted, vertices, vertexCount * sizeof(Vec2));
    std::swap(sorted[index], sorted[0]);

    // Sort the vertices based on the angle related to the lowest vertex.
    std::sort(sorted + 1, sorted + vertexCount, [&](const Vec2& a, const Vec2& b) -> bool {
        Vec2 ra = a - bottom;
        Vec2 rb = b - bottom;

        float angleA = Atan2(ra.y, ra.x);
        float angleB = Atan2(rb.y, rb.x);

        if (angleA == angleB)
        {
            return Dist2(bottom, a) < Dist2(bottom, b);
        }
        else
        {
            return angleA < angleB;
        }
    });

    // Discard overlapped bottom vertices
    int32 i = 0;
    while (i < vertexCount)
    {
        Vec2& v0 = sorted[i];
        Vec2& v1 = sorted[(i + 1) % vertexCount];

        if (v0 == v1)
        {
            ++i;
        }
        else
        {
            break;
        }
    }

    // Initialize stack
    int32 sp = 0;
    *outVertexCount = vertexCount - i;
    outVertices[sp++] = sorted[i++];
    outVertices[sp++] = sorted[i++];

    while (i < vertexCount)
    {
        Vec2& v = sorted[i];
        int32 l = sp;

        if (v == outVertices[l - 1])
        {
            ++i;
            continue;
        }

        Vec2 d1 = outVertices[l - 1] - outVertices[l - 2];
        Vec2 d2 = v - outVertices[l - 1];

        if (Cross(d1, d2) <= 0)
        {
            --sp;

            if (l < 3)
            {
                outVertices[sp++] = v;
                break;
            }
        }
        else
        {
            outVertices[sp++] = v;
            ++i;
        }
    }

    delete[] sorted;
}

std::vector<Vec2> ComputeConvexHull(std::span<const Vec2> vertices)
{
    if (vertices.size() < 3)
    {
        return std::vector<Vec2>(vertices.begin(), vertices.end());
    }

    size_t index = 0;
    for (size_t i = 1; i < vertices.size(); ++i)
    {
        if (vertices[i].y < vertices[index].y)
        {
            index = i;
        }
    }
    Vec2 bottom = vertices[index];

    std::vector<Vec2> sorted(vertices.begin(), vertices.end());
    std::swap(sorted[index], sorted[0]);

    std::sort(sorted.begin() + 1, sorted.end(), [&](const Vec2& a, const Vec2& b) -> bool {
        Vec2 ra = a - bottom;
        Vec2 rb = b - bottom;

        float angleA = Atan2(ra.y, ra.x);
        float angleB = Atan2(rb.y, rb.x);

        if (angleA == angleB)
        {
            return Dist2(bottom, a) < Dist2(bottom, b);
        }
        else
        {
            return angleA < angleB;
        }
    });

    // Discard overlapped bottom vertices
    size_t i = 0;
    while (i < sorted.size())
    {
        Vec2& v0 = sorted[i];
        Vec2& v1 = sorted[(i + 1) % sorted.size()];

        if (v0 == v1)
        {
            i++;
        }
        else
        {
            break;
        }
    }

    std::vector<Vec2> s;
    s.push_back(sorted[i++]);
    s.push_back(sorted[i++]);

    while (i < sorted.size())
    {
        Vec2& v = sorted[i];
        size_t l = s.size();

        if (v == s[l - 1])
        {
            ++i;
            continue;
        }

        Vec2 d1 = s[l - 1] - s[l - 2];
        Vec2 d2 = v - s[l - 1];

        if (Cross(d1, d2) <= 0)
        {
            s.pop_back();

            if (l < 3)
            {
                s.push_back(v);
                break;
            }
        }
        else
        {
            s.push_back(v);
            ++i;
        }
    }

    return s;
}

static Circle ComputeCircle2(const Vec2& a, const Vec2& b)
{
    Vec2 center{ (a.x + b.x) / 2, (a.y + b.y) / 2 };
    float radius = Dist(a, b) / 2;

    return Circle{ radius, center };
}

static Circle ComputeCircle3(const Vec2& a, const Vec2& b, const Vec2& c)
{
    // Solve linear equations to find circumcircle
    float d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));

    float nx =
        ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) + (c.x * c.x + c.y * c.y) * (a.y - b.y));
    float ny =
        ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) + (c.x * c.x + c.y * c.y) * (b.x - a.x));

    Vec2 center{ nx / d, ny / d };
    float radius = Dist(center, a);

    return Circle{ radius, center };
}

// Welzl algorithm (https://en.wikipedia.org/wiki/Smallest-circle_problem)
// Compute minimum circle containing all points
// P: original all vertices
// R: vertices (n <= 3) to define minimum circle
static Circle Welzl(const std::vector<Vec2>& P, std::vector<Vec2>& R, int32 n)
{
    if (n == 0 || R.size() == 3)
    {
        // Handle trivial cases
        switch (R.size())
        {
        case 0:
            return Circle{ 0, { 0, 0 } };
        case 1:
            return Circle{ 0, R[0] };
        case 2:
            return ComputeCircle2(R[0], R[1]);

        default:
            return ComputeCircle3(R[0], R[1], R[2]);
        }
    }

    // Select random point.
    Vec2 p = P[n - 1];

    // Find minimum circle without current point p.
    Circle D = Welzl(P, R, n - 1);

    // If the returned circle also contains current point,
    // it is the minimum circle for the whole of P and is returned.
    if (D.TestPoint(identity, p))
    {
        return D;
    }
    else
    {
        // Otherwise, current point must lie on the boundary of the resulting circle.
        // It recurses, but with the set R of points known to be on the boundary.

        R.push_back(p);
        Circle newCircle = Welzl(P, R, n - 1);
        R.pop_back();

        return newCircle;
    }
}

Circle ComputeCircle(std::span<Vec2> points)
{
    std::vector<Vec2> R;
    int32 n = points.size();

    std::vector<Vec2> copy(points.begin(), points.end());
    std::shuffle(copy.begin(), copy.end(), std::mt19937{});

    return Welzl(copy, R, n);
}

// Straight implementation of https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm
std::vector<Polygon> ComputeTriangles(std::span<Vec2> points, bool removeOutliers)
{
    struct TriEdge
    {
        Vec2 p0, p1;

        TriEdge() = default;
        TriEdge(const Vec2& p0, const Vec2& p1)
            : p0{ p0 }
            , p1{ p1 }
        {
        }
    };

    struct Tri
    {
        Vec2 p0, p1, p2;

        Tri() = default;
        Tri(const Vec2& p0, const Vec2& p1, const Vec2& p2)
            : p0{ p0 }
            , p1{ p1 }
            , p2{ p2 }
        {
        }

        bool HasEdge(const TriEdge& edge) const
        {
            if (edge.p0 == p0 && edge.p1 == p1) return true;
            if (edge.p0 == p1 && edge.p1 == p0) return true;

            if (edge.p0 == p1 && edge.p1 == p2) return true;
            if (edge.p0 == p2 && edge.p1 == p1) return true;

            if (edge.p0 == p2 && edge.p1 == p0) return true;
            if (edge.p0 == p0 && edge.p1 == p2) return true;

            return false;
        }

        bool HasVertex(const Vec2& p) const
        {
            return p0 == p || p1 == p || p2 == p;
        }

        Vec2 GetCenter() const
        {
            return (p0 + p1 + p2) / 3;
        }

        void GetEdges(TriEdge edges[3]) const
        {
            edges[0] = TriEdge{ p0, p1 };
            edges[1] = TriEdge{ p1, p2 };
            edges[2] = TriEdge{ p2, p0 };
        }
    };

    auto RayCastEdge = [](Vec2 o, Vec2 d, TriEdge edge) {
        Vec2 e = edge.p1 - edge.p0;
        Vec2 o2a = edge.p0 - o;

        float c = Cross(d, e);

        if (c == 0)
        {
            return false;
        }

        float t = Cross(o2a, e) / c;
        float u = Cross(o2a, d) / c;

        return (t >= 0 && u >= 0 && u <= 1);
    };

    auto Contains = [RayCastEdge](std::vector<TriEdge>& edges, const Vec2& p) {
        int32 count = 0;
        for (TriEdge& e : edges)
        {
            if (RayCastEdge(p, Vec2(0, 1), e))
            {
                ++count;
            }
        }

        return count % 2 == 1;
    };

    std::vector<TriEdge> inputEdges;

    size_t i0 = points.size() - 1;
    for (size_t i1 = 0; i1 < points.size(); ++i1)
    {
        inputEdges.push_back(TriEdge{ points[i0], points[i1] });
        i0 = i1;
    }

    AABB bounds(Vec2{ max_value, max_value }, -Vec2{ max_value, max_value });

    for (const Vec2& p : points)
    {
        bounds = AABB::Union(bounds, p);
    }

    Vec2 extents = bounds.GetExtents();

    Vec2 p0{ bounds.min.x - extents.x - epsilon, bounds.min.y };
    Vec2 p1{ bounds.max.x + extents.x + epsilon, bounds.min.y };
    Vec2 p2{ bounds.min.x + extents.x / 2, bounds.max.y + extents.y + epsilon };

    Tri super{ p0, p1, p2 };

    std::list<Tri> tris;
    tris.push_back(super);

    using TriIter = std::list<Tri>::iterator;

    for (const Vec2& p : points)
    {
        std::vector<TriIter> badTris;

        for (TriIter t = tris.begin(); t != tris.end(); ++t)
        {
            Circle c = ComputeCircle3(t->p0, t->p1, t->p2);
            if (c.TestPoint(identity, p))
            {
                badTris.push_back(t);
            }
        }

        std::vector<TriEdge> poly;

        for (const TriIter& t : badTris)
        {
            TriEdge edges[3];
            t->GetEdges(edges);

            for (const TriEdge& e : edges)
            {
                bool hasSharedEdge = false;
                for (const TriIter& other : badTris)
                {
                    if (t == other)
                    {
                        continue;
                    }

                    if (other->HasEdge(e))
                    {
                        hasSharedEdge = true;
                    }
                }

                if (!hasSharedEdge)
                {
                    poly.push_back(e);
                }
            }
        }

        for (const TriIter& iter : badTris)
        {
            tris.erase(iter);
        }

        for (const TriEdge& e : poly)
        {
            tris.push_back(Tri{ e.p0, e.p1, p });
        }
    }

    std::vector<Polygon> res;

    for (const Tri& t : tris)
    {
        if (!t.HasVertex(super.p0) && !t.HasVertex(super.p1) && !t.HasVertex(super.p2))
        {
            if (!removeOutliers || Contains(inputEdges, t.GetCenter()))
            {
                res.push_back(Polygon{ t.p0, t.p1, t.p2 });
            }
        }
    }

    return res;
}

} // namespace muli