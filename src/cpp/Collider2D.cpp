#include "Collider2D.hpp"
#include "Circle.hpp"
#include "Polygone.hpp"
#include "Hitbox.hpp"
#include "Capsule.hpp"

using namespace std;

namespace ToricCollisionSystem
{
    #pragma region Lines / Straightline / Ray

    bool Collider2D::CollideStraightLines(const StraightLine2D& l1, const StraightLine2D& l2)
    {
        return CollideStraightLines(l1.A, l1.B, l2.A, l2.B);
    }

    bool Collider2D::CollideStraightLines(const Vector2& A, const Vector2& B, const Vector2& O, const Vector2& P)
    {
        float dx1 = B.x - A.x;
        float dy1 = B.y - A.y;
        float dx2 = O.x - P.x;
        float dy2 = O.y - P.y;
        float det = dx1 * dy2 - dy1 * dx2;
        return abs(det) > 1e-3f;
    }

    bool Collider2D::CollideStraightLines(const StraightLine2D& l1, const StraightLine2D& l2, Vector2& collisionPoint)
    {
        return Collider2D::CollideStraightLines(l1.A, l1.B, l2.A, l2.B, collisionPoint);
    }

    bool Collider2D::CollideStraightLines(const Vector2& A, const Vector2& B, const Vector2& O, const Vector2& P, Vector2& collisionPoint)
    {
        float dx1 = B.x - A.x;
        float dy1 = B.y - A.y;
        float dx2 = O.x - P.x;
        float dy2 = O.y - P.y;
        float det = dx1 * dy2 - dy1 * dx2;

        if (abs(det) <= 1e-3f)
        {
            collisionPoint = Vector2::zero();
            return false;
        }

        float dx3 = O.x - A.x;
        float dy3 = O.y - A.y;
        float t = (dx3 * dy2 - dy3 * dx2) / det;

        collisionPoint.x = A.x + t * dx1;
        collisionPoint.y = A.y + t * dy1;
        return true;
    }

    bool Collider2D::CollideLines(const Line2D& l1, const Line2D& l2)
    {
        return Collider2D::CollideLines(l1.A, l1.B, l2.A, l2.B);
    }

    bool Collider2D::CollideLines(const Vector2& A, const Vector2& B, const Vector2& O, const Vector2& P)
    {
        return Collider2D::CollideLineStraightLine(A, B, O, P) && Collider2D::CollideLineStraightLine(O, P, A, B);
    }

    bool Collider2D::CollideLines(const Line2D& l1, const Line2D& l2, Vector2& collisionPoint)
    {
        return Collider2D::CollideLines(l1.A, l1.B, l2.A, l2.B, collisionPoint);
    }

    bool Collider2D::CollideLines(const Vector2& A, const Vector2& B, const Vector2& O, const Vector2& P, Vector2& collisionPoint)
    {
        if (!CollideLineStraightLine(A, B, O, P) || !CollideLineStraightLine(O, P, A, B))
        {
            collisionPoint = Vector2::zero();
            return false;
        }
        return Collider2D::CollideStraightLines(A, B, O, P, collisionPoint);
    }

    bool Collider2D::CollideLineStraightLine(const Line2D& line, const StraightLine2D& straightLine)
    {
        return Collider2D::CollideLineStraightLine(line.A, line.B, straightLine.A, straightLine.B);
    }

    bool Collider2D::CollideLineStraightLine(const Vector2& O, const Vector2& P, const Vector2& A, const Vector2& B)
    {
        Vector2 AB = Vector2(B.x - A.x, B.y - A.y);
        Vector2 AP = Vector2(P.x - A.x, P.y - A.y);
        Vector2 AO = Vector2(O.x - A.x, O.y - A.y);

        float cross1 = AB.x * AP.y - AB.y * AP.x;
        float cross2 = AB.x * AO.y - AB.y * AO.x;
        return cross1 * cross2 < 0.0f;
    }

    bool Collider2D::CollideLineStraightLine(const Line2D& line, const StraightLine2D& straightLine, Vector2& collisionPoint)
    {
        return Collider2D::CollideLineStraightLine(line.A, line.B, straightLine.A, straightLine.B, collisionPoint);
    }

    bool Collider2D::CollideLineStraightLine(const Vector2& O, const Vector2& P, const Vector2& A, const Vector2& B, Vector2& collisionPoint)
    {
        if (Collider2D::CollideLineStraightLine(O, P, A, B))
        {
            return Collider2D::CollideStraightLines(O, P, A, B, collisionPoint);
        }

        collisionPoint = Vector2::zero();
        return false;
    }

    bool Collider2D::CollideLineRay(const Line2D& line, const Ray2D& ray)
    {
        return Collider2D::CollideLines(line.A, line.B, ray.start, ray.end);
    }

    bool Collider2D::CollideLineRay(const Line2D& line, const Ray2D& ray, Vector2& collisionPoint)
    {
        return Collider2D::CollideLines(line.A, line.B, ray.start, ray.end, collisionPoint);
    }

    bool Collider2D::CollideStraightLineRay(const StraightLine2D& line, const Ray2D& ray)
    {
        return Collider2D::CollideLineStraightLine(ray.start, ray.end, line.A, line.B);
    }

    bool Collider2D::CollideStraightLineRay(const StraightLine2D& line, const Ray2D& ray, Vector2& collisionPoint)
    {
        return Collider2D::CollideLineStraightLine(ray.start, ray.end, line.A, line.B, collisionPoint);
    }

    bool Collider2D::CollideRays(const Ray2D& ray1, const Ray2D& ray2)
    {
        return Collider2D::CollideLines(ray1.start, ray1.end, ray2.start, ray2.end);
    }

    bool Collider2D::CollideRays(const Vector2& start1, const Vector2& end1, const Vector2& start2, const Vector2& end2)
    {
        return Collider2D::CollideLines(start1, end1, start2, end2);
    }

    bool Collider2D::CollideRays(const Ray2D& ray1, const Ray2D& ray2, Vector2& collisionPoint)
    {
        return Collider2D::CollideLines(ray1.start, ray1.end, ray2.start, ray2.end, collisionPoint);
    }

    bool Collider2D::CollideRays(const Vector2& start1, const Vector2& end1, const Vector2& start2, const Vector2& end2, Vector2& collisionPoint)
    {
        return Collider2D::CollideLines(start1, end1, start2, end2, collisionPoint);
    }

    #pragma endregion

    #pragma region Circle

    bool Collider2D::ComputeCirclesIntersections(const Circle& circle1, const Circle& circle2, Vector2& outIntersection1, Vector2& outIntersection2)
    {
        float sqrDist = Vector2::sqrDistance(circle1.center(), circle2.center());
        float rSum = circle1.radius + circle2.radius;
        float rDiff = abs(circle1.radius - circle2.radius);

        if (sqrDist <= rSum * rSum && sqrDist > rDiff * rDiff)
        {
            if (abs(circle1.center().y - circle2.center().y) < 1e-3f)
            {
                float x = (circle2.radius * circle2.radius - circle1.radius * circle1.radius - circle2.center().x * circle2.center().x + circle1.center().x * circle1.center().x) /
                    (2.0f * (circle1.center().x - circle2.center().x));
                float b = -2.0f * circle2.center().y;
                float c = circle2.center().x * circle2.center().x + x * x - 2.0f * circle2.center().x * x + circle2.center().y * circle2.center().y - circle2.radius * circle2.radius;
                float sqrtDelta = sqrt(b * b - 4.0f * c);
                outIntersection1 = { x, (b + sqrtDelta) * -0.5f };
                outIntersection2 = { x, (sqrtDelta - b) * 0.5f };
                return true;
            }
            else
            {
                float N = (circle2.radius * circle2.radius - circle1.radius * circle1.radius - circle2.center().x * circle2.center().x + circle1.center().x * circle1.center().x -
                    circle2.center().y * circle2.center().y + circle1.center().y * circle1.center().y) / (2.0f * (circle1.center().y - circle2.center().y));
                float t = (circle1.center().x - circle2.center().x) / (circle1.center().y - circle2.center().y);
                float a = t * t + 1.0f;
                float b = 2.0f * (circle1.center().y * t - N * t - circle1.center().x);
                float c = circle1.center().x * circle1.center().x + circle1.center().y * circle1.center().y + N * N - circle1.radius * circle1.radius - 2.0f * circle1.center().y * N;
                float sqrtDelta = sqrt(b * b - 4.0f * a * c);
                float inv2a = 1.0f / (2.0f * a);
                float x1 = -inv2a * (b + sqrtDelta);
                float x2 = inv2a * (sqrtDelta - b);
                outIntersection1 = { x1, N - x1 * t };
                outIntersection2 = { x2, N - x2 * t };
                return true;
            }
        }

        outIntersection1 = outIntersection2 = Vector2::zero();
        return false;
    }

    bool Collider2D::ComputeCircleStraightLineIntersections(const Circle& circle, const Vector2& A, const Vector2& B, Vector2& outIntersection1, Vector2& outIntersection2)
    {
        if (!CollideCircleStraightLine(circle, A, B))
        {
            outIntersection1 = outIntersection2 = Vector2::zero();
            return false;
        }

        if (abs(A.x - B.x) < 1e-2f)
        {
            float avg = 0.5f * (A.x + B.x);
            float dx = circle.center().x - avg;
            float sqrtDelta = sqrt(circle.radius * circle.radius - dx * dx);
            outIntersection1 = Vector2(avg, circle.center().y - sqrtDelta);
            outIntersection2 = Vector2(avg, circle.center().y + sqrtDelta);
        }
        else
        {
            float m = (B.y - A.y) / (B.x - A.x);
            float p = A.y - m * A.x;
            float a = 1.0f + m * m;
            float b = 2.0f * (m * p - circle.center().x - m * circle.center().y);
            float C = circle.center().x * circle.center().x + p * p - 2.0f * p * circle.center().y + circle.center().y * circle.center().y - circle.radius * circle.radius;
            float sqrtDelta = sqrt(b * b - 4.0f * a * C);
            float inv2a = 1.0f / (2.0f * a);
            m *= inv2a;
            outIntersection1 = Vector2(-inv2a * (b + sqrtDelta), -m * (b + sqrtDelta) + p);
            outIntersection2 = Vector2(inv2a * (sqrtDelta - b), m * (sqrtDelta - b) + p);
        }

        return true;
    }

    pair<Vector2, Vector2> Collider2D::ComputeCircleStraightLineIntersectionsUnchecked(const Circle& circle, const Vector2& A, const Vector2& B)
    {
        if (abs(A.x - B.x) < 1e-2f)
        {
            float avg = 0.5f * (A.x + B.x);
            float dx = circle.center().x - avg;
            float sqrtDelta = sqrt(circle.radius * circle.radius - dx * dx);
            return { Vector2(avg, circle.center().y - sqrtDelta), Vector2(avg, circle.center().y + sqrtDelta) };
        }
        else
        {
            float m = (B.y - A.y) / (B.x - A.x);
            float p = A.y - m * A.x;
            float a = 1.0f + m * m;
            float b = 2.0f * (m * p - circle.center().x - m * circle.center().y);
            float C = circle.center().x * circle.center().x + p * p - 2.0f * p * circle.center().y + circle.center().y * circle.center().y - circle.radius * circle.radius;
            float sqrtDelta = sqrt(b * b - 4.0f * a * C);
            float inv2a = 1.0f / (2.0f * a);
            m *= inv2a;
            return 
            {
                Vector2(-inv2a * (b + sqrtDelta), -m * (b + sqrtDelta) + p),
                Vector2(inv2a * (sqrtDelta - b), m * (sqrtDelta - b) + p)
            };
        }
    }

    vector<Vector2> Collider2D::ComputeCircleLineIntersections(const Circle& circle, const Vector2& A, const Vector2& B)
    {
        if (!CollideCircleStraightLine(circle, A, B))
            return {};

        Vector2 i1, i2;
        if (abs(A.x - B.x) < 1e-2f)
        {
            float avg = 0.5f * (A.x + B.x);
            float dx = circle.center().x - avg;
            float sqrtDelta = sqrt(circle.radius * circle.radius - dx * dx);
            i1 = Vector2(avg, circle.center().y - sqrtDelta);
            i2 = Vector2(avg, circle.center().y + sqrtDelta);

            float minY = min(A.y, B.y) - 1e-4f;
            float maxY = max(A.y, B.y) + 1e-4f;

            vector<Vector2> result;
            if (minY <= i1.y && i1.y <= maxY) result.push_back(i1);
            if (minY <= i2.y && i2.y <= maxY) result.push_back(i2);
            return result;
        }
        else
        {
            float m = (B.y - A.y) / (B.x - A.x);
            float p = A.y - m * A.x;
            float a = 1.0f + m * m;
            float b = 2.0f * (m * p - circle.center().x - m * circle.center().y);
            float C = circle.center().x * circle.center().x + p * p - 2.0f * p * circle.center().y + circle.center().y * circle.center().y - circle.radius * circle.radius;
            float sqrtDelta = sqrt(b * b - 4.0f * a * C);
            float inv2a = 1.0f / (2.0f * a);
            m *= inv2a;
            i1 = Vector2(-inv2a * (b + sqrtDelta), -m * (b + sqrtDelta) + p);
            i2 = Vector2(inv2a * (sqrtDelta - b), m * (sqrtDelta - b) + p);

            float minX = min(A.x, B.x) - 1e-4f;
            float maxX = max(A.x, B.x) + 1e-4f;
            float minY = min(A.y, B.y) - 1e-4f;
            float maxY = max(A.y, B.y) + 1e-4f;

            vector<Vector2> result;
            if (minX <= i1.x && i1.x <= maxX && minY <= i1.y && i1.y <= maxY) result.push_back(i1);
            if (minX <= i2.x && i2.x <= maxX && minY <= i2.y && i2.y <= maxY) result.push_back(i2);
            return result;
        }
    }

    bool Collider2D::CollideCircles(const Circle& circle1, const Circle& circle2)
    {
        float rr = circle1.radius + circle2.radius;
        return Vector2::sqrDistance(circle1.center(), circle2.center()) <= rr * rr;
    }

    bool Collider2D::CollideCircles(const Circle& circle1, const Circle& circle2, Vector2& collisionPoint)
    {
        float sqrDist = Vector2::sqrDistance(circle1.center(), circle2.center());
        float rr = circle1.radius + circle2.radius;
        if (sqrDist <= rr * rr)
        {
            float dist = sqrt(sqrDist);
            float d = 0.5f * (rr - dist);
            collisionPoint = circle1.center() + ((circle2.center() - circle1.center()) * ((circle1.radius - d) / dist));
            return true;
        }
        collisionPoint = Vector2::zero();
        return false;
    }

    bool Collider2D::CollideCircles(const Circle& circle1, const Circle& circle2, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2)
    {
        float sqrDist = Vector2::sqrDistance(circle1.center(), circle2.center());
        float rr = (circle1.radius + circle2.radius);
        if (sqrDist <= rr * rr)
        {
            float dist = sqrt(sqrDist);
            float d = 0.5f * (rr - dist);
            collisionPoint = circle1.center() + ((circle2.center() - circle1.center()) * ((circle1.radius - d) / dist));
            normal1 = (collisionPoint - circle1.center()) * (1.0f / (circle1.radius - d));
            normal2 = (collisionPoint - circle2.center()) * (1.0f / (circle2.radius - d));
            return true;
        }
        collisionPoint = normal1 = normal2 = Vector2::zero();
        return false;
    }

    bool Collider2D::CollideCirclePolygone(const Circle& circle, const Polygone& polygone)
    {
        uint32_t vertexCount = polygone.vertices().size();
        for (uint32_t i = 0; i < vertexCount; i++)
        {
            const Vector2& A = polygone.vertices()[i];
            const Vector2& B = polygone.vertices()[(i + 1) % vertexCount];
            if (CollideCircleLine(circle, A, B))
                return true;
        }

        return circle.Contains(polygone.center()) || (polygone.inclusiveCircle().radius > circle.radius && polygone.Contains(circle.center()));
    }

    bool Collider2D::CollideCirclePolygone(const Circle& circle, const Polygone& polygone, Vector2& collisionPoint)
    {
        collisionPoint = Vector2::zero();
        Vector2 i1, i2;
        Vector2 A, B;

        uint32_t vertexCount = polygone.vertices().size();
        vector<Vector2> cache;
        for (uint32_t i = 0; i < vertexCount; i++)
        {
            A = polygone.vertices()[i];
            B = polygone.vertices()[(i + 1) % vertexCount];

            if (ComputeCircleStraightLineIntersections(circle, A, B, i1, i2))
            {
                bool containI1 = Line2D::Contain(A, B, i1);
                bool containI2 = Line2D::Contain(A, B, i2);

                if (containI1 || containI2)
                {
                    if (containI1 && containI2)
                    {
                        Vector2 tmp = (i1 + i2) * 0.5f;
                        float dist = circle.radius - ((circle.radius - Vector2::distance(circle.center(), tmp)) * 0.5f);
                        Vector2 dir = tmp - circle.center();
                        dir.Normalize();
                        cache.push_back(circle.center() + dir * dist);
                    }
                    else if (containI1)
                    {
                        cache.push_back(i1);
                    }
                    else
                    {
                        cache.push_back(i2);
                    }
                }
            }
        }

        if (!cache.empty())
        {
            for (const Vector2& pos : cache)
            {
                collisionPoint = collisionPoint + pos;
            }
            collisionPoint = collisionPoint * (1.0f / static_cast<float>(cache.size()));
            cache.clear();
            return true;
        }

        if (polygone.Contains(circle.center()))
        {
            collisionPoint = (circle.center() + polygone.center()) * 0.5f;
            return true;
        }
    }

    bool Collider2D::CollideCirclePolygone(const Circle& circle, const Polygone& polygone, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2)
    {
        if (CollideCirclePolygone(circle, polygone, collisionPoint))
        {
            normal1 = collisionPoint - circle.center();
            normal1.Normalize();
            normal2 = normal1 * -1.0f;
            return true;
        }

        normal1 = Vector2::zero();
        normal2 = Vector2::zero();
        return false;
    }

    bool Collider2D::CollideCircleHitbox(const Circle& circle, const Hitbox& hitbox)
    {
        return CollideCirclePolygone(circle, hitbox.ToPolygone());
    }

    bool Collider2D::CollideCircleHitbox(const Circle& circle, const Hitbox& hitbox, Vector2& collisionPoint)
    {
        return CollideCirclePolygone(circle, hitbox.ToPolygone(), collisionPoint);
    }

    bool Collider2D::CollideCircleHitbox(const Circle& circle, const Hitbox& hitbox, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2)
    {
        return CollideCirclePolygone(circle, hitbox.ToPolygone(), collisionPoint, normal1, normal2);
    }

    bool Collider2D::CollideCircleLine(const Circle& circle, const Vector2& A, const Vector2& B)
    {
        return Line2D::SqrDistance(A, B, circle.center()) <= circle.radius * circle.radius;
    }

    bool Collider2D::CollideCircleLine(const Circle& circle, const Line2D& line)
    {
        return CollideCircleLine(circle, line.A, line.B);
    }

    bool Collider2D::CollideCircleLine(const Circle& circle, const Vector2& A, const Vector2& B, Vector2& collisionPoint)
    {
        float rr = circle.radius * circle.radius;
        bool containA = Vector2::sqrDistance(circle.center(), A) <= rr;
        bool containB = Vector2::sqrDistance(circle.center(), B) <= rr;

        if ((containA && containB) || (!containA && !containB))
        {
            Vector2 closestPoint = !containA ? Line2D::ClosestPoint(A, B, circle.center()) : StraightLine2D::ClosestPoint(A, B, circle.center());
            if (!containA && Vector2::sqrDistance(closestPoint, circle.center()) > rr)
            {
                collisionPoint = Vector2::zero();
                return false;
            }

            Vector2 dir = closestPoint - circle.center();
            dir.Normalize();
            collisionPoint = circle.center() + dir * circle.radius;
            return true;
        }

        pair<Vector2, Vector2> intersections = ComputeCircleStraightLineIntersectionsUnchecked(circle, A, B);
        Vector2 i1 = intersections.first;
        Vector2 i2 = intersections.second;
        collisionPoint = (min(A.x, B.x) <= i1.x && i1.x <= max(A.x, B.x) && min(A.y, B.y) <= i1.y && i1.y <= max(A.y, B.y)) ? i1 : i2;
        return true;
    }

    bool Collider2D::CollideCircleLine(const Circle& circle, const Line2D& line, Vector2& collisionPoint)
    {
        return CollideCircleLine(circle, line.A, line.B, collisionPoint);
    }

    bool Collider2D::CollideCircleLine(const Circle& circle, const Vector2& A, const Vector2& B, Vector2& collisionPoint, Vector2& normal)
    {
        if (CollideCircleLine(circle, A, B, collisionPoint))
        {
            normal = collisionPoint - circle.center();
            normal.Normalize();
            return true;
        }
        normal = Vector2::zero();
        return false;
    }

    bool Collider2D::CollideCircleLine(const Circle& circle, const Line2D& line, Vector2& collisionPoint, Vector2& normal)
    {
        return CollideCircleLine(circle, line.A, line.B, collisionPoint, normal);
    }

    bool Collider2D::CollideCircleStraightLine(const Circle& circle, const StraightLine2D& straightLine) 
    {
        return Collider2D::CollideCircleStraightLine(circle, straightLine.A, straightLine.B);
    }

    bool Collider2D::CollideCircleStraightLine(const Circle& circle, const Vector2& A, const Vector2& B) 
    {
        Vector2 u = B - A;
        Vector2 AC = circle.center() - A;
        float numerator = abs(u.x * AC.y - u.y * AC.x);
        float magnitude = sqrt(u.x * u.x + u.y * u.y);
        return numerator < circle.radius * magnitude;
    }

    bool Collider2D::CollideCircleStraightLine(const Circle& circle, const StraightLine2D& straightLine, Vector2& collisionPoint) 
    {
        return CollideCircleStraightLine(circle, straightLine.A, straightLine.B, collisionPoint);
    }

    bool Collider2D::CollideCircleStraightLine(const Circle& circle, const Vector2& A, const Vector2& B, Vector2& collisionPoint) 
    {
        if (!CollideCircleStraightLine(circle, A, B)) 
        {
            collisionPoint = Vector2::zero();
            return false;
        }

        Vector2 u = B - A;
        Vector2 AC = circle.center() - A;
        float ti = (u.x * AC.x + u.y * AC.y) / (u.x * u.x + u.y * u.y);
        collisionPoint.x = A.x + ti * u.x;
        collisionPoint.x = A.y + ti * u.y;

        ti = Vector2::sqrDistance(collisionPoint, circle.center());

        if (ti > circle.radius * circle.radius)
            return true;

        Vector2 endLine = collisionPoint + (circle.radius / sqrtf(ti)) * (collisionPoint - circle.center());
        return Collider2D::CollideCircleLine(circle, collisionPoint, endLine, collisionPoint);
    }

    bool Collider2D::CollideCircleStraightLine(const Circle& circle, const Vector2& A, const Vector2& B, Vector2& collisionPoint, Vector2& normal) 
    {
        if (CollideCircleStraightLine(circle, A, B, collisionPoint)) 
        {
            normal = collisionPoint - circle.center();
            normal.Normalize();
            return true;
        }
        normal = Vector2::zero();
        return false;
    }

    bool Collider2D::CollideCircleStraightLine(const Circle& circle, const StraightLine2D& straightLine, Vector2& collisionPoint, Vector2& normal) 
    {
        return CollideCircleStraightLine(circle, straightLine.A, straightLine.B, collisionPoint, normal);
    }

    bool Collider2D::CollideCircleRay(const Circle& circle, const Ray2D& ray) 
    {
        return CollideCircleRay(circle, ray.start, ray.end);
    }

    bool Collider2D::CollideCircleRay(const Circle& circle, const Vector2& start, const Vector2& end) 
    {
        return Collider2D::CollideCircleLine(circle, start, end);
    }

    bool Collider2D::CollideCircleRay(const Circle& circle, const Ray2D& ray, Vector2& collisionPoint) 
    {
        return CollideCircleRay(circle, ray.start, ray.end, collisionPoint);
    }

    bool Collider2D::CollideCircleRay(const Circle& circle, const Vector2& start, const Vector2& end, Vector2& collisionPoint) 
    {
        if (circle.Contains(start)) 
        {
            collisionPoint = start;
            return true;
        }

        Vector2 i1, i2;
        if (ComputeCircleStraightLineIntersections(circle, start, end, i1, i2)) 
        {
            float minX = min(start.x, end.x);
            float maxX = max(start.x, end.x);
            float minY = min(start.y, end.y);
            float maxY = max(start.y, end.y);

            bool containI1 = i1.x >= minX && i1.x <= maxX && i1.y >= minY && i1.y <= maxY;
            bool containI2 = i2.x >= minX && i2.x <= maxX && i2.y >= minY && i2.y <= maxY;

            if (containI1 && containI2) 
            {
                collisionPoint = Vector2::sqrDistance(start, i1) <= Vector2::sqrDistance(start, i2) ? i1 : i2;
                return true;
            }
            if (containI1) 
            {
                collisionPoint = i1;
                return true;
            }
            if (containI2) 
            {
                collisionPoint = i2;
                return true;
            }
        }

        collisionPoint = Vector2::zero();
        return false;
    }

    bool Collider2D::CollideCircleRay(const Circle& circle, const Ray2D& ray, Vector2& collisionPoint, Vector2& normal) 
    {
        return CollideCircleRay(circle, ray.start, ray.end, collisionPoint, normal);
    }

    bool Collider2D::CollideCircleRay(const Circle& circle, const Vector2& start, const Vector2& end, Vector2& collisionPoint, Vector2& normal) 
    {
        if (circle.Contains(start)) 
        {
            collisionPoint = start;
            normal = start - circle.center();
            normal.Normalize();
            return true;
        }

        if (CollideCircleRay(circle, start, end, collisionPoint)) 
        {
            normal = collisionPoint - circle.center();
            normal.Normalize();
            return true;
        }

        collisionPoint = normal = Vector2::zero();
        return false;
    }

    bool Collider2D::CollideCircleCapsule(const Circle& circle, const Capsule& capsule)
    {
        float rr = (capsule.circle1().radius + circle.radius);
        rr *= rr;

        if (Vector2::sqrDistance(capsule.circle1().center(), circle.center()) <= rr || Vector2::sqrDistance(capsule.circle2().center(), circle.center()) <= rr)
            return true;

        float distance = Vector2::distance(capsule.circle1().center(), capsule.circle2().center());
        if (distance >= 1e-5f)
        {
            Vector2 dirHori = (capsule.circle1().center() - capsule.circle2().center()) / distance;
            Vector2 dirVerti = dirHori.NormalVector();
            distance *= 0.5f;

            Line2D line1(
                capsule.hitbox().center() + (dirVerti * capsule.circle1().radius) - (dirHori * distance),
                capsule.hitbox().center() + (dirVerti * capsule.circle1().radius) + (dirHori * distance)
            );

            if (CollideCircleLine(circle, line1))
                return true;

            Line2D line2(
                capsule.hitbox().center() - (dirVerti * capsule.circle1().radius) - (dirHori * distance),
                capsule.hitbox().center() - (dirVerti * capsule.circle1().radius) + (dirHori * distance)
            );

            return CollideCircleLine(circle, line2);
        }

        return false;
    }

    bool Collider2D::CollideCircleCapsule(const Circle& circle, const Capsule& capsule, Vector2& collisionPoint)
    {
        float distance = Vector2::distance(capsule.circle1().center(), capsule.circle2().center());
        Vector2 i1, i2;

        vector<Vector2> cache;
        if (distance >= 1e-3f)
        {
            Vector2 dirHori = (capsule.circle1().center() - capsule.circle2().center()) * (1.0f / distance);
            Vector2 dirVerti = dirHori.NormalVector();
            distance *= 0.5f;

            Line2D line1(
                capsule.hitbox().center() + (dirVerti * capsule.circle1().radius) - (dirHori * distance),
                capsule.hitbox().center() + (dirVerti * capsule.circle1().radius) + (dirHori * distance)
            );
            Line2D line2(
                capsule.hitbox().center() - (dirVerti * capsule.circle1().radius) - (dirHori * distance),
                capsule.hitbox().center() - (dirVerti * capsule.circle1().radius) + (dirHori * distance)
            );

            auto PerformLine = [&](const Line2D& line)
                {
                    if (ComputeCircleStraightLineIntersections(circle, line.A, line.B, i1, i2))
                    {
                        bool containI1 = Line2D::Contain(line.A, line.B, i1);
                        bool containI2 = Line2D::Contain(line.A, line.B, i2);

                        if (containI1 || containI2)
                        {
                            if (containI1 && containI2)
                            {
                                Vector2 tmp = (i1 + i2) * 0.5f;
                                float dist = circle.radius - ((circle.radius - Vector2::distance(circle.center(), tmp)) * 0.5f);
                                Vector2 norm = tmp - circle.center();
                                norm.Normalize();
                                cache.emplace_back(circle.center() + norm * dist);
                            }
                            else if (containI1)
                                cache.emplace_back(i1);
                            else
                                cache.emplace_back(i2);
                        }
                    }
                };

            PerformLine(line1);
            PerformLine(line2);
        }

        if (ComputeCirclesIntersections(circle, capsule.circle1(), i1, i2))
        {
            if (Vector2::dot(capsule.circle1().center() - capsule.hitbox().center(), i1 - capsule.circle1().center()) >= 0.0f)
                cache.emplace_back(i1);
            if (Vector2::dot(capsule.circle1().center() - capsule.hitbox().center(), i2 - capsule.circle1().center()) >= 0.0f)
                cache.emplace_back(i2);
        }

        if (ComputeCirclesIntersections(circle, capsule.circle2(), i1, i2))
        {
            if (Vector2::dot(capsule.circle2().center() - capsule.hitbox().center(), i1 - capsule.circle2().center()) >= 0.0f)
                cache.emplace_back(i1);
            if (Vector2::dot(capsule.circle2().center() - capsule.hitbox().center(), i2 - capsule.circle2().center()) >= 0.0f)
                cache.emplace_back(i2);
        }

        collisionPoint = Vector2::zero();
        if (!cache.empty())
        {
            for (const Vector2& pos : cache)
                collisionPoint = collisionPoint + pos;

            collisionPoint = collisionPoint * (1.0f / static_cast<float>(cache.size()));
            cache.clear();
            return true;
        }

        if (circle.Contains(capsule.center()))
        {
            collisionPoint = (circle.center() + capsule.center()) * 0.5f;
            return true;
        }

        if (capsule.Contains(circle.center()))
        {
            collisionPoint = (circle.center() + capsule.center()) * 0.5f;
            return true;
        }

        return false;
    }

    bool Collider2D::CollideCircleCapsule(const Circle& circle, const Capsule& capsule, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2)
    {
        if (CollideCircleCapsule(circle, capsule, collisionPoint))
        {
            normal1 = collisionPoint - circle.center();
            normal1.Normalize();
            normal2 = normal1 * -1.0f;
            return true;
        }

        normal1 = Vector2::zero();
        normal2 = Vector2::zero();
        return false;
    }

    #pragma endregion

    #pragma region Polygone

    bool Collider2D::CollidePolygones(const Polygone& poly1, const Polygone& poly2)
    {
        const vector<Vector2>& v1 = poly1.vertices();
        const vector<Vector2>& v2 = poly2.vertices();

        for (uint32_t i = 0u; i < v1.size(); i++)
        {
            Vector2 a1 = v1[i];
            Vector2 a2 = v1[(i + 1u) % v1.size()];
            for (uint32_t j = 0u; j < v2.size(); ++j)
            {
                Vector2 b1 = v2[j];
                Vector2 b2 = v2[(j + 1u) % v2.size()];
                if (CollideLines(a1, a2, b1, b2))
                    return true;
            }
        }

        const Circle& c1 = poly1.inclusiveCircle();
        const Circle& c2 = poly2.inclusiveCircle();
        return (c1.radius >= c2.radius) ? poly1.Contains(poly2.center()) : poly2.Contains(poly1.center());
    }

    bool Collider2D::CollidePolygones(const Polygone& poly1, const Polygone& poly2, Vector2& outCollisionPoint)
    {
        const vector<Vector2>& v1 = poly1.vertices();
        const vector<Vector2>& v2 = poly2.vertices();

        vector<Vector2> cache;
        uint32_t count1 = v1.size();
        uint32_t count2 = v2.size();

        for (uint32_t i = 0u; i < count1; i++)
        {
            uint32_t ip1 = (i + 1u) % count1;
            for (uint32_t j = 0u; j < count2; ++j)
            {
                uint32_t jp1 = (j + 1) % count2;
                Vector2 intersec;
                if (CollideLines(v1[i], v1[ip1], v2[j], v2[jp1], intersec))
                {
                    cache.push_back(intersec);
                }
            }
        }

        if (!cache.empty())
        {
            outCollisionPoint = Vector2::zero();
            for (const Vector2& pt : cache)
                outCollisionPoint = outCollisionPoint + pt;
            outCollisionPoint = outCollisionPoint * (1.0f / static_cast<float>(cache.size()));
            cache.clear();
            return true;
        }

        bool contains = (poly1.inclusiveCircle().radius >= poly2.inclusiveCircle().radius) ? poly1.Contains(poly2.center()) : poly2.Contains(poly1.center());

        if (contains)
        {
            outCollisionPoint = (poly1.center() + poly2.center()) * 0.5f;
            return true;
        }

        outCollisionPoint = Vector2::zero();
        return false;
    }

    bool Collider2D::CollidePolygones(const Polygone& poly1, const Polygone& poly2, Vector2& outCollisionPoint, Vector2& outNormal1, Vector2& outNormal2)
    {
        if (poly1.Contains(poly2.center()) && poly2.Contains(poly1.center()))
        {
            outNormal1 = Vector2::normalize(poly2.center() - poly1.center());
            outNormal2 = outNormal1 * -1.0f;
            outCollisionPoint = (poly1.center() + poly2.center()) * 0.5f;
            return true;
        }

        const vector<Vector2>& v1 = poly1.vertices();
        const vector<Vector2>& v2 = poly2.vertices();
        uint32_t count1 = v1.size();
        uint32_t count2 = v2.size();
        vector<Vector2> cache, cache1;

        for (uint32_t i = 0; i < count1; i++)
        {
            uint32_t ip1 = (i + 1) % count1;
            Vector2 side1 = v1[ip1];
            for (uint32_t j = 0; j < count2; ++j)
            {
                uint32_t jp1 = (j + 1) % count2;
                Vector2 intersec;
                if (CollideLines(v1[i], side1, v2[j], v2[jp1], intersec))
                {
                    cache.push_back(intersec);
                    Vector2 n = (side1 - v1[i]).NormalVector();
                    if (!poly1.IsNormalOnRightDirection(intersec, n, static_cast<int>(i)))
                        n = n * -1.0f;
                    cache.push_back(n);

                    n = (v2[jp1] - v2[j]).NormalVector();
                    if (!poly2.IsNormalOnRightDirection(intersec, n, static_cast<int>(j)))
                        n = n * -1.0f;
                    cache1.push_back(n);
                }
            }
        }

        if (cache.empty())
        {
            bool contains = (poly1.inclusiveCircle().radius >= poly2.inclusiveCircle().radius) ? poly1.Contains(poly2.center()) : poly2.Contains(poly1.center());
            if (contains)
            {
                outCollisionPoint = (poly1.center() + poly2.center()) * 0.5f;
                outNormal1 = Vector2::normalize(poly2.center() - poly1.center());
                outNormal2 = outNormal1 * -1.0f;
                return true;
            }
            outCollisionPoint = outNormal1 = outNormal2 = Vector2::zero();
            return false;
        }

        outCollisionPoint = outNormal1 = Vector2::zero();
        for (const Vector2& pt : cache)
        {
            outCollisionPoint = outCollisionPoint + pt;
        }
        outCollisionPoint = outCollisionPoint * (1.0f / static_cast<float>(cache.size()));
        cache.clear();

        for (const Vector2& n : cache1)
        {
            outNormal1 = outNormal1 + n;
        }
        outNormal1.Normalize();

        if (Vector2::dot(outNormal1, outCollisionPoint - poly1.center()) < 0.0f)
        {
            outNormal1 = outNormal1 * -1.0f;
        }
        outNormal2 = outNormal1 * -1.0f;
        cache1.clear();
        return true;
    }

    bool Collider2D::CollidePolygoneHitbox(const Polygone& poly, const Hitbox& hitbox)
    {
        return CollidePolygones(hitbox.ToPolygone(), poly);
    }

    bool Collider2D::CollidePolygoneHitbox(const Polygone& poly, const Hitbox& hitbox, Vector2& outCollisionPoint)
    {
        return CollidePolygones(poly, hitbox.ToPolygone(), outCollisionPoint);
    }

    bool Collider2D::CollidePolygoneHitbox(const Polygone& poly, const Hitbox& hitbox, Vector2& outCollisionPoint, Vector2& normal1, Vector2& normal2)
    {
        return CollidePolygones(poly, hitbox.ToPolygone(), outCollisionPoint, normal1, normal2);
    }

    bool Collider2D::CollidePolygoneLine(const Polygone& poly, const Vector2& A, const Vector2& B)
    {
        const vector<Vector2>& vertices = poly.vertices();
        uint32_t count = vertices.size();

        for (uint32_t i = 0; i < count; i++)
        {
            uint32_t ip1 = (i + 1) % count;
            if (CollideLines(A, B, vertices[i], vertices[ip1]))
                return true;
        }
        return poly.Contains(A);
    }

    bool Collider2D::CollidePolygoneLine(const Polygone& poly, const Line2D& line)
    {
        return poly.CollideLine(line);
    }

    bool Collider2D::CollidePolygoneLine(const Polygone& poly, const Vector2& A, const Vector2& B, Vector2& outCollisionPoint)
    {
        Line2D line(A, B);
        return CollidePolygoneLine(poly, line, outCollisionPoint);
    }

    bool Collider2D::CollidePolygoneLine(const Polygone& poly, const Line2D& line, Vector2& outCollisionPoint)
    {
        return CollidePolygoneLine(poly, line.A, line.B, outCollisionPoint);
    }

    bool Collider2D::CollidePolygoneLine(const Polygone& poly, const Vector2& A, const Vector2& B, Vector2& outCollisionPoint, Vector2& outNormal)
    {
        Line2D line(A, B);
        return CollidePolygoneLine(poly, line, outCollisionPoint, outNormal);
    }

    bool Collider2D::CollidePolygoneLine(const Polygone& poly, const Line2D& line, Vector2& outCollisionPoint, Vector2& outNormal)
    {
        return CollidePolygoneLine(poly, line.A, line.B, outCollisionPoint, outNormal);
    }

    bool Collider2D::CollidePolygoneStraightLine(const Polygone& polygone, const Vector2& A, const Vector2& B)
    {
        const vector<Vector2>& vertices = polygone.vertices();
        const uint32_t count = vertices.size();

        for (uint32_t i = 0; i < count; i++)
        {
            const Vector2& p1 = vertices[i];
            const Vector2& p2 = vertices[(i + 1) % count];
            if (CollideLineStraightLine(p1, p2, A, B))
                return true;
        }
        return false;
    }

    bool Collider2D::CollidePolygoneStraightLine(const Polygone& polygone, const StraightLine2D& straightLine)
    {
        return polygone.CollideStraightLine(straightLine);
    }

    bool Collider2D::CollidePolygoneStraightLine(const Polygone& polygone, const Vector2& A, const Vector2& B, Vector2& collisionPoint)
    {
        const auto& vertices = polygone.vertices();
        const uint32_t count = vertices.size();

        vector<Vector2> cache;
        for (uint32_t i = 0; i < count; i++)
        {
            const Vector2& p1 = vertices[i];
            const Vector2& p2 = vertices[(i + 1) % count];
            Vector2 intersec;

            if (CollideLineStraightLine(p1, p2, A, B, intersec))
                cache.push_back(intersec);
        }

        if (cache.empty())
        {
            collisionPoint = Vector2::zero();
            return false;
        }

        Vector2 avgInter = Vector2::zero();
        for (const Vector2& pt : cache)
            avgInter = avgInter + pt;

        avgInter = avgInter / static_cast<float>(cache.size());

        Vector2 normal = (B - A).NormalVector();
        Vector2 otherPoint = avgInter + normal;

        float minSqrDist = FLT_MAX;
        Vector2 bestPoint = Vector2::zero();

        for (uint32_t i = 0; i < count; i++)
        {
            const Vector2& p1 = vertices[i];
            const Vector2& p2 = vertices[(i + 1) % count];
            Vector2 intersec;

            if (CollideLineStraightLine(p1, p2, avgInter, otherPoint, intersec))
            {
                float d = Vector2::sqrDistance(intersec, avgInter);
                if (d < minSqrDist)
                {
                    minSqrDist = d;
                    bestPoint = intersec;
                }
            }
        }

        collisionPoint = bestPoint;
        return true;
    }

    bool Collider2D::CollidePolygoneStraightLine(const Polygone& polygone, const StraightLine2D& straightLine, Vector2& collisionPoint)
    {
        return CollidePolygoneStraightLine(polygone, straightLine.A, straightLine.B, collisionPoint);
    }

    bool Collider2D::CollidePolygoneStraightLine(const Polygone& polygone, const Vector2& A, const Vector2& B, Vector2& collisionPoint, Vector2& normal)
    {
        const auto& vertices = polygone.vertices();
        const uint32_t count = vertices.size();

        vector<Vector2> cache;

        for (uint32_t i = 0; i < count; i++)
        {
            const Vector2& p1 = vertices[i];
            const Vector2& p2 = vertices[(i + 1) % count];
            Vector2 intersec;

            if (CollideLineStraightLine(p1, p2, A, B, intersec))
                cache.push_back(intersec);
        }

        if (cache.empty())
        {
            collisionPoint = Vector2::zero();
            normal = Vector2::zero();
            return false;
        }

        Vector2 avgInter = Vector2::zero();
        for (const Vector2& pt : cache)
            avgInter = avgInter + pt;

        avgInter = avgInter / static_cast<float>(cache.size());
        Vector2 candidateNormal = (B - A).NormalVector();
        Vector2 otherPoint = avgInter + candidateNormal;

        float minSqrDist = FLT_MAX;
        int minSideIndex = -1;
        Vector2 bestPoint = Vector2::zero();

        for (uint32_t i = 0; i < count; i++)
        {
            const Vector2& p1 = vertices[i];
            const Vector2& p2 = vertices[(i + 1u) % count];
            Vector2 intersec;

            if (CollideLineStraightLine(p1, p2, avgInter, otherPoint, intersec))
            {
                float d = Vector2::sqrDistance(intersec, avgInter);
                if (d < minSqrDist)
                {
                    minSqrDist = d;
                    bestPoint = intersec;
                    minSideIndex = static_cast<int>(i);
                }
            }
        }

        collisionPoint = bestPoint;
        if (minSideIndex >= 0)
        {
            const Vector2& p1 = vertices[minSideIndex];
            const Vector2& p2 = vertices[(minSideIndex + 1) % count];
            normal = (p2 - p1).NormalVector();

            if (!polygone.IsNormalOnRightDirection(collisionPoint, normal, minSideIndex))
                normal = normal * -1.0f;
        }
        else
        {
            normal = Vector2::zero();
        }

        return true;
    }

    bool Collider2D::CollidePolygoneStraightLine(const Polygone& polygone, const StraightLine2D& straightLine, Vector2& collisionPoint, Vector2& normal)
    {
        return CollidePolygoneStraightLine(polygone, straightLine.A, straightLine.B, collisionPoint, normal);
    }

    bool Collider2D::CollidePolygoneRay(const Polygone& polygone, const Ray2D& ray)
    {
        return CollidePolygoneRay(polygone, ray.start, ray.end);
    }

    bool Collider2D::CollidePolygoneRay(const Polygone& polygone, const Vector2& start, const Vector2& end)
    {
        Vector2 _;
        return CollidePolygoneRay(polygone, start, end, _);
    }

    bool Collider2D::CollidePolygoneRay(const Polygone& polygone, const Ray2D& ray, Vector2& collisionPoint)
    {
        return CollidePolygoneRay(polygone, ray.start, ray.end, collisionPoint);
    }

    bool Collider2D::CollidePolygoneRay(const Polygone& polygone, const Vector2& start, const Vector2& end, Vector2& collisionPoint)
    {
        if (polygone.Contains(start))
        {
            collisionPoint = start;
            return true;
        }

        float minX, maxX, minY, maxY;
        if (start.x >= end.x)
        {
            minX = end.x - 1e-4f;
            maxX = start.x + 1e-4f;
        }
        else
        {
            maxX = end.x + 1e-4f;
            minX = start.x - 1e-4f;
        }
        if (start.y >= end.y)
        {
            minY = end.y - 1e-4f;
            maxY = start.y + 1e-4f;
        }
        else
        {
            maxY = end.y + 1e-4f;
            minY = start.y - 1e-4f;
        }

        vector<Vector2> collideVertices;
        const vector<Vector2>& vertices = polygone.vertices();
        uint32_t count = vertices.size();
        for (uint32_t i = 0u; i < count; i++)
        {
            Vector2 inter;
            if (CollideLineStraightLine(vertices[i], vertices[(i + 1u) % count], start, end, inter))
            {
                if (inter.x >= minX && inter.x <= maxX && inter.y >= minY && inter.y <= maxY)
                {
                    collideVertices.push_back(inter);
                }
            }
        }

        if (collideVertices.empty())
        {
            collisionPoint = Vector2::zero();
            return false;
        }

        collisionPoint = collideVertices[0];
        float minSqrDist = Vector2::sqrDistance(start, collisionPoint);

        for (const Vector2& p : collideVertices)
        {
            float dist = Vector2::sqrDistance(start, p);
            if (dist < minSqrDist)
            {
                minSqrDist = dist;
                collisionPoint = p;
            }
        }

        return true;
    }

    bool Collider2D::CollidePolygoneRay(const Polygone& polygone, const Ray2D& ray, Vector2& collisionPoint, Vector2& normal)
    {
        return CollidePolygoneRay(polygone, ray.start, ray.end, collisionPoint, normal);
    }

    bool Collider2D::CollidePolygoneRay(const Polygone& polygone, const Vector2& start, const Vector2& end, Vector2& collisionPoint, Vector2& normal)
    {
        if (polygone.Contains(start))
        {
            collisionPoint = start;
            normal = Vector2::normalize(start - polygone.center());
            return true;
        }

        float minX, maxX, minY, maxY;
        if (start.x >= end.x)
        {
            minX = end.x - 1e-4f;
            maxX = start.x + 1e-4f;
        }
        else
        {
            maxX = end.x + 1e-4f;
            minX = start.x - 1e-4f;
        }
        if (start.y >= end.y)
        {
            minY = end.y - 1e-4f;
            maxY = start.y + 1e-4f;
        }
        else
        {
            maxY = end.y + 1e-4f;
            minY = start.y - 1e-4f;
        }

        const vector<Vector2>& vertices = polygone.vertices();
        uint32_t count = vertices.size();
        vector<int> intersSideIndex;
        vector<Vector2> intersections;

        for (uint32_t i = 0; i < count; i++)
        {
            Vector2 inter;
            if (CollideLineStraightLine(vertices[i], vertices[(i + 1) % count], start, end, inter))
            {
                if (inter.x >= minX && inter.x <= maxX && inter.y >= minY && inter.y <= maxY)
                {
                    intersections.push_back(inter);
                    intersSideIndex.push_back(static_cast<int>(i));
                }
            }
        }

        if (intersections.empty())
        {
            collisionPoint = normal = Vector2::zero();
            return false;
        }

        uint32_t indexMinDist = 0u;
        collisionPoint = intersections[0];
        float minSqrDist = Vector2::sqrDistance(start, collisionPoint);

        for (uint32_t i = 1u; i < intersections.size(); i++)
        {
            float dist = Vector2::sqrDistance(start, intersections[i]);
            if (dist < minSqrDist)
            {
                minSqrDist = dist;
                collisionPoint = intersections[i];
                indexMinDist = i;
            }
        }

        int sideIndex = intersSideIndex[indexMinDist];
        normal = (vertices[(sideIndex + 1u) % count] - vertices[sideIndex]).NormalVector();
        if (!polygone.IsNormalOnRightDirection(collisionPoint, normal, sideIndex))
        {
            normal = normal * -1.0f;
        }

        return true;
    }

    bool Collider2D::CollidePolygoneCapsule(const Polygone& polygone, const Capsule& capsule)
    {
        return CollideCirclePolygone(capsule.circle1(), polygone)
            || CollideCirclePolygone(capsule.circle2(), polygone)
            || CollidePolygoneHitbox(polygone, capsule.hitbox());
    }

    bool Collider2D::CollidePolygoneCapsule(const Polygone& polygone, const Capsule& capsule, Vector2& collisionPoint)
    {
        float distance = Vector2::distance(capsule.circle1().center(), capsule.circle2().center());
        Vector2 dirHori = (capsule.circle1().center() - capsule.circle2().center()) / distance;
        Vector2 dirVerti = dirHori.NormalVector();
        distance *= 0.5f;

        Line2D line1(
            capsule.hitbox().center() + (dirVerti * capsule.circle1().radius) - (dirHori * distance),
            capsule.hitbox().center() + (dirVerti * capsule.circle1().radius) + (dirHori * distance)
        );
        Line2D line2(
            capsule.hitbox().center() - (dirVerti * capsule.circle1().radius) - (dirHori * distance),
            capsule.hitbox().center() - (dirVerti * capsule.circle1().radius) + (dirHori * distance)
        );

        Vector2 inter;
        const vector<Vector2>& vertices = polygone.vertices();
        vector<Vector2> collideVertices;

        for (uint32_t i = 0u; i < vertices.size(); i++)
        {
            Vector2 v1 = vertices[i];
            Vector2 v2 = vertices[(i + 1u) % vertices.size()];

            if (CollideLines(v1, v2, line1.A, line1.B, inter))
                collideVertices.push_back(inter);
            if (CollideLines(v1, v2, line2.A, line2.B, inter))
                collideVertices.push_back(inter);

            vector<Vector2> circleInter1 = ComputeCircleLineIntersections(capsule.circle1(), v1, v2);
            for (const Vector2& pt : circleInter1)
            {
                if (Vector2::dot(pt - capsule.circle1().center(), dirHori) > 0.0f)
                {
                    collideVertices.push_back(pt);
                }
            }


            vector<Vector2> circleInter2 = ComputeCircleLineIntersections(capsule.circle2(), v1, v2);
            for (const Vector2& pt : circleInter2)
            {
                if (Vector2::dot(pt - capsule.circle2().center(), dirHori) < 0.0f)
                {
                    collideVertices.push_back(pt);
                }
            }
        }

        if (!collideVertices.empty())
        {
            collisionPoint = Vector2::zero();
            for (const Vector2& pt : collideVertices)
            {
                collisionPoint = collisionPoint + pt;
            }
            collisionPoint = collisionPoint * (1.0f / static_cast<float>(collideVertices.size()));
            return true;
        }

        bool contains = capsule.inclusiveCircle().radius >= polygone.inclusiveCircle().radius ? capsule.Contains(polygone.center()) : polygone.Contains(capsule.center());

        if (contains)
        {
            collisionPoint = (capsule.center() + polygone.center()) * 0.5f;
            return true;
        }

        collisionPoint = Vector2::zero();
        return false;
    }

    bool Collider2D::CollidePolygoneCapsule(const Polygone& polygone, const Capsule& capsule, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2)
    {
        float distance = Vector2::distance(capsule.circle1().center(), capsule.circle2().center());
        Vector2 dirHori = (capsule.circle1().center() - capsule.circle2().center()) / distance;
        Vector2 dirVerti = dirHori.NormalVector();
        distance *= 0.5f;

        Line2D line1(
            capsule.hitbox().center() + (dirVerti * capsule.circle1().radius) - (dirHori * distance),
            capsule.hitbox().center() + (dirVerti * capsule.circle1().radius) + (dirHori * distance)
        );
        Line2D line2(
            capsule.hitbox().center() - (dirVerti * capsule.circle1().radius) - (dirHori * distance),
            capsule.hitbox().center() - (dirVerti * capsule.circle1().radius) + (dirHori * distance)
        );

        const vector<Vector2>& vertices = polygone.vertices();
        vector<Vector2> intersections;
        vector<Vector2> normals;
        Vector2 inter, n;

        for (uint32_t i = 0u; i < vertices.size(); i++)
        {
            Vector2 v1 = vertices[i];
            Vector2 v2 = vertices[(i + 1) % vertices.size()];
            bool nComputed = false;

            if (CollideLines(v1, v2, line1.A, line1.B, inter))
            {
                intersections.push_back(inter);
                n = (v2 - v1).NormalVector();
                if (!polygone.IsNormalOnRightDirection(inter, n, static_cast<int>(i)))
                {
                    n = n * -1.0f;
                }
                intersections.push_back(n);
                nComputed = true;
            }

            if (CollideLines(v1, v2, line2.A, line2.B, inter))
            {
                intersections.push_back(inter);
                if (!nComputed)
                {
                    n = (v2 - v1).NormalVector();
                    if (!polygone.IsNormalOnRightDirection(inter, n, static_cast<int>(i)))
                        n = n * -1.0f;
                }
                normals.push_back(n);
            }

            vector<Vector2> circleInter1 = ComputeCircleLineIntersections(capsule.circle1(), v1, v2);
            for (const Vector2& pt : circleInter1)
            {
                n = capsule.circle1().center() - pt;
                if (Vector2::dot(n, dirHori) < 0.0f)
                {
                    intersections.push_back(pt);
                    normals.push_back(n * (1.0f / capsule.circle1().radius));
                }
            }

            vector<Vector2> circleInter2 = ComputeCircleLineIntersections(capsule.circle2(), v1, v2);
            for (const auto& pt : circleInter2)
            {
                n = capsule.circle2().center() - pt;
                if (Vector2::dot(n, dirHori) > 0)
                {
                    intersections.push_back(pt);
                    normals.push_back(n * (1.0f / capsule.circle2().radius));
                }
            }
        }

        if (!intersections.empty())
        {
            collisionPoint = Vector2::zero();
            for (const Vector2& pt : intersections)
            {
                collisionPoint = collisionPoint + pt;
            }
            collisionPoint = collisionPoint * (1.0f / static_cast<float>(intersections.size()));

            normal1 = Vector2::zero();
            for (const Vector2& norm : normals)
            {
                normal1 = normal1 + norm;
            }

            normal1.Normalize();
            normal2 = normal1 * -1.0f;
            return true;
        }

        bool contains = capsule.inclusiveCircle().radius >= polygone.inclusiveCircle().radius ? capsule.Contains(polygone.center()) : polygone.Contains(capsule.center());

        if (contains)
        {
            collisionPoint = (capsule.center() + polygone.center()) * 0.5f;
            normal1 = capsule.center() - polygone.center();
            normal1.Normalize();
            normal2 = normal1 * -1.0f;
            return true;
        }

        collisionPoint = normal1 = normal2 = Vector2::zero();
        return false;
    }

    #pragma endregion

    #pragma region Hitbox

    bool Collider2D::CollideHitboxes(const Hitbox& h1, const Hitbox& h2)
    {
        return CollidePolygones(h1.ToPolygone(), h2.ToPolygone());
    }

    bool Collider2D::CollideHitboxes(const Hitbox& h1, const Hitbox& h2, Vector2& collisionPoint)
    {
        return CollidePolygones(h1.ToPolygone(), h2.ToPolygone(), collisionPoint);
    }

    bool Collider2D::CollideHitboxes(const Hitbox& h1, const Hitbox& h2, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2)
    {
        return CollidePolygones(h1.ToPolygone(), h2.ToPolygone(), collisionPoint, normal1, normal2);
    }

    bool Collider2D::CollideHitboxLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B)
    {
        return CollidePolygoneLine(hitbox.ToPolygone(), A, B);
    }

    bool Collider2D::CollideHitboxLine(const Hitbox& hitbox, const Line2D& line)
    {
        return CollideHitboxLine(hitbox, line.A, line.B);
    }

    bool Collider2D::CollideHitboxLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B, Vector2& collisionPoint)
    {
        return CollidePolygoneLine(hitbox.ToPolygone(), A, B, collisionPoint);
    }

    bool Collider2D::CollideHitboxLine(const Hitbox& hitbox, const Line2D& line, Vector2& collisionPoint)
    {
        return CollideHitboxLine(hitbox, line.A, line.B, collisionPoint);
    }

    bool Collider2D::CollideHitboxLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B, Vector2& collisionPoint, Vector2& normal)
    {
        return CollidePolygoneLine(hitbox.ToPolygone(), A, B, collisionPoint, normal);
    }

    bool Collider2D::CollideHitboxLine(const Hitbox& hitbox, const Line2D& line, Vector2& collisionPoint, Vector2& normal)
    {
        return CollideHitboxLine(hitbox, line.A, line.B, collisionPoint, normal);
    }

    bool Collider2D::CollideHitboxStraightLine(const Hitbox& hitbox, const StraightLine2D& line)
    {
        return CollidePolygoneStraightLine(hitbox.ToPolygone(), line.A, line.B);
    }

    bool Collider2D::CollideHitboxStraightLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B)
    {
        return CollidePolygoneStraightLine(hitbox.ToPolygone(), A, B);
    }

    bool Collider2D::CollideHitboxStraightLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B, Vector2& collisionPoint)
    {
        return CollidePolygoneStraightLine(hitbox.ToPolygone(), A, B, collisionPoint);
    }

    bool Collider2D::CollideHitboxStraightLine(const Hitbox& hitbox, const StraightLine2D& line, Vector2& collisionPoint)
    {
        return CollideHitboxStraightLine(hitbox, line.A, line.B, collisionPoint);
    }

    bool Collider2D::CollideHitboxStraightLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B, Vector2& collisionPoint, Vector2& normal)
    {
        return CollidePolygoneStraightLine(hitbox.ToPolygone(), A, B, collisionPoint, normal);
    }

    bool Collider2D::CollideHitboxStraightLine(const Hitbox& hitbox, const StraightLine2D& line, Vector2& collisionPoint, Vector2& normal)
    {
        return CollideHitboxStraightLine(hitbox, line.A, line.B, collisionPoint, normal);
    }

    bool Collider2D::CollideHitboxRay(const Hitbox& hitbox, const Ray2D& ray)
    {
        return CollidePolygoneRay(hitbox.ToPolygone(), ray);
    }

    bool Collider2D::CollideHitboxRay(const Hitbox& hitbox, const Vector2& start, const Vector2& end)
    {
        return CollidePolygoneRay(hitbox.ToPolygone(), start, end);
    }

    bool Collider2D::CollideHitboxRay(const Hitbox& hitbox, const Ray2D& ray, Vector2& collisionPoint)
    {
        return CollidePolygoneRay(hitbox.ToPolygone(), ray, collisionPoint);
    }

    bool Collider2D::CollideHitboxRay(const Hitbox& hitbox, const Vector2& start, const Vector2& end, Vector2& collisionPoint)
    {
        return CollidePolygoneRay(hitbox.ToPolygone(), start, end, collisionPoint);
    }

    bool Collider2D::CollideHitboxRay(const Hitbox& hitbox, const Ray2D& ray, Vector2& collisionPoint, Vector2& normal)
    {
        return CollidePolygoneRay(hitbox.ToPolygone(), ray, collisionPoint, normal);
    }

    bool Collider2D::CollideHitboxRay(const Hitbox& hitbox, const Vector2& start, const Vector2& end, Vector2& collisionPoint, Vector2& normal)
    {
        return CollidePolygoneRay(hitbox.ToPolygone(), start, end, collisionPoint, normal);
    }

    bool Collider2D::CollideHitboxCapsule(const Hitbox& hitbox, const Capsule& capsule)
    {
        return CollideCircleHitbox(capsule.circle1(), hitbox) ||
            CollideCircleHitbox(capsule.circle2(), hitbox) ||
            CollideHitboxes(hitbox, capsule.hitbox());
    }

    bool Collider2D::CollideHitboxCapsule(const Hitbox& hitbox, const Capsule& capsule, Vector2& collisionPoint)
    {
        return CollidePolygoneCapsule(hitbox.ToPolygone(), capsule, collisionPoint);
    }

    bool Collider2D::CollideHitboxCapsule(const Hitbox& hitbox, const Capsule& capsule, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2)
    {
        return CollidePolygoneCapsule(hitbox.ToPolygone(), capsule, collisionPoint, normal1, normal2);
    }

    #pragma endregion

    #pragma region Capsule

    bool Collider2D::CollideCapsules(const Capsule& capsule1, const Capsule& capsule2)
    {
        return CollideCircleCapsule(capsule1.circle1(), capsule2)
            || CollideCircleCapsule(capsule1.circle2(), capsule2)
            || CollideHitboxCapsule(capsule1.hitbox(), capsule2);
    }

    bool Collider2D::CollideCapsules(const Capsule& capsule1, const Capsule& capsule2, Vector2& collisionPoint)
    {
        vector<Vector2> intersections;
        if (Collider2D::CollideHitboxes(capsule1.hitbox(), capsule2.hitbox(), collisionPoint))
        {
            intersections.push_back(collisionPoint);
        }

        if (CollideCircleHitbox(capsule2.circle1(), capsule1.hitbox(), collisionPoint))
        {
            intersections.push_back(collisionPoint);
        }

        if (CollideCircleHitbox(capsule2.circle2(), capsule1.hitbox(), collisionPoint))
        {
            intersections.push_back(collisionPoint);
        }

        if (CollideCircleHitbox(capsule1.circle1(), capsule2.hitbox(), collisionPoint))
        {
            intersections.push_back(collisionPoint);
        }

        if (CollideCircles(capsule1.circle1(), capsule2.circle1(), collisionPoint))
        {
            intersections.push_back(collisionPoint);
        }

        if (CollideCircles(capsule1.circle1(), capsule2.circle2(), collisionPoint))
        {
            intersections.push_back(collisionPoint);
        }

        if (CollideCircleHitbox(capsule1.circle2(), capsule2.hitbox(), collisionPoint))
        {
            intersections.push_back(collisionPoint);
        }

        if (CollideCircles(capsule1.circle2(), capsule2.circle1(), collisionPoint))
        {
            intersections.push_back(collisionPoint);
        }

        if (CollideCircles(capsule1.circle2(), capsule2.circle2(), collisionPoint))
        {
            intersections.push_back(collisionPoint);
        }

        collisionPoint.x = 0.0f;
        collisionPoint.y = 0.0f;
        if (!intersections.empty())
        {
            for (const Vector2& pt : intersections)
            {
                collisionPoint = collisionPoint + pt;
            }

            collisionPoint = collisionPoint / static_cast<float>(intersections.size());
            return true;
        }

        return false;
    }

    bool Collider2D::CollideCapsules(const Capsule& capsule1, const Capsule& capsule2, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2)
    {
        if (CollideCapsules(capsule1, capsule2, collisionPoint))
        {
            normal1 = Vector2::normalize(collisionPoint - capsule1.center());
            normal2 = Vector2::normalize(collisionPoint - capsule2.center());
            return true;
        }

        normal1 = normal2 = Vector2::zero();
        return false;
    }

    bool Collider2D::CollideCapsuleLine(const Capsule& capsule, const Line2D& line)
    {
        return CollideCapsuleLine(capsule, line.A, line.B);
    }

    bool Collider2D::CollideCapsuleLine(const Capsule& capsule, const Vector2& A, const Vector2& B)
    {
        return CollideCircleLine(capsule.circle1(), A, B) || CollideCircleLine(capsule.circle2(), A, B) || CollideHitboxLine(capsule.hitbox(), A, B);
    }

    bool Collider2D::CollideCapsuleLine(const Capsule& capsule, const Line2D& line, Vector2& collisionPoint)
    {
        return CollideCapsuleLine(capsule, line.A, line.B, collisionPoint);
    }

    bool Collider2D::CollideCapsuleLine(const Capsule& capsule, const Vector2& A, const Vector2& B, Vector2& collisionPoint)
    {
        float distance = Vector2::distance(capsule.circle1().center(), capsule.circle2().center());
        Vector2 dirHori = (capsule.circle1().center() - capsule.circle2().center()) / distance;
        Vector2 dirVerti = dirHori.NormalVector();
        float halfDistance = distance * 0.5f;

        Vector2 center = capsule.hitbox().center();
        float radius = capsule.circle1().radius;

        Line2D line1(center + dirVerti * radius - dirHori * halfDistance, center + dirVerti * radius + dirHori * halfDistance);
        Line2D line2(center - dirVerti * radius - dirHori * halfDistance, center - dirVerti * radius + dirHori * halfDistance);

        Vector2 inter, inter2;
        vector<Vector2> intersections;

        if (CollideLines(line1.A, line1.B, A, B, inter)) 
        {
            intersections.push_back(inter);
        }
        if (CollideLines(line2.A, line2.B, A, B, inter)) 
        {
            intersections.push_back(inter);
        }

        vector<Vector2> circleInters = ComputeCircleLineIntersections(capsule.circle1(), A, B);
        for (const Vector2& pt : circleInters) 
        {
            if (Vector2::dot(dirHori, pt - capsule.circle1().center()) > 0.0f) 
            {
                intersections.push_back(pt);
            }
        }

        circleInters = ComputeCircleLineIntersections(capsule.circle2(), A, B);
        for (const Vector2& pt : circleInters)
        {
            if (Vector2::dot(dirHori, pt - capsule.circle2().center()) < 0.0f) 
            {
                intersections.push_back(pt);
            }
        }

        if (intersections.size() <= 1)
        {
            if (intersections.size() == 1u)
            {
                collisionPoint = intersections[0u];
                return true;
            }

            if (capsule.Contains(A)) 
            {
                if (CollideLineStraightLine(line1.A, line1.B, A, B, inter)) 
                {
                    intersections.push_back(inter);
                }
                if (CollideLineStraightLine(line2.A, line2.B, A, B, inter)) 
                {
                    intersections.push_back(inter);
                }

                if (ComputeCircleStraightLineIntersections(capsule.circle1(), A, B, inter, inter2)) 
                {
                    if (Vector2::dot(dirHori, inter - capsule.circle1().center()) > 0.0f)
                    {
                        intersections.push_back(inter);
                    }
                    if (Vector2::dot(dirHori, inter2 - capsule.circle1().center()) > 0.0f)
                    {
                        intersections.push_back(inter2);
                    }
                }

                if (ComputeCircleStraightLineIntersections(capsule.circle2(), A, B, inter, inter2)) 
                {
                    if (Vector2::dot(dirHori, inter - capsule.circle2().center()) < 0.0f)
                    {
                        intersections.push_back(inter);
                    }
                    if (Vector2::dot(dirHori, inter2 - capsule.circle2().center()) < 0.0f)
                    {
                        intersections.push_back(inter2);
                    }
                }
            }
            else 
            {
                collisionPoint = Vector2::zero();
                return false;
            }
        }

        collisionPoint = intersections[0];
        for (uint32_t i = 1; i < intersections.size(); i++)
        {
            collisionPoint = collisionPoint + intersections[i];
        }
        collisionPoint = collisionPoint / static_cast<float>(intersections.size());

        intersections.clear();
        Vector2 otherPoint = collisionPoint + (B - A).NormalVector();

        if (CollideLineStraightLine(line1.A, line1.B, collisionPoint, otherPoint, inter))
        {
            intersections.push_back(inter);
        }
        if (CollideLineStraightLine(line2.A, line2.B, collisionPoint, otherPoint, inter))
        {
            intersections.push_back(inter);
        }

        if (ComputeCircleStraightLineIntersections(capsule.circle1(), collisionPoint, otherPoint, inter, inter2)) 
        {
            if (Vector2::dot(dirHori, inter - capsule.circle1().center()) > 0.0f)
            {
                intersections.push_back(inter);
            }
            if (Vector2::dot(dirHori, inter2 - capsule.circle1().center()) > 0.0f)
            {
                intersections.push_back(inter2);
            }
        }

        if (ComputeCircleStraightLineIntersections(capsule.circle2(), collisionPoint, otherPoint, inter, inter2)) 
        {
            if (Vector2::dot(dirHori, inter - capsule.circle2().center()) < 0.0f)
            {
                intersections.push_back(inter);
            }
            if (Vector2::dot(dirHori, inter2 - capsule.circle2().center()) < 0.0f)
            {
                intersections.push_back(inter2);
            }
        }

        if (intersections.empty())
        {
            collisionPoint = Vector2::zero();
            return false;
        }

        // Find closest point
        Vector2 closest = intersections[0];
        float minSqr = Vector2::sqrDistance(collisionPoint, closest);

        for (uint32_t i = 1u; i < intersections.size(); i++)
        {
            float sqr = Vector2::sqrDistance(collisionPoint, intersections[i]);
            if (sqr < minSqr) 
            {
                minSqr = sqr;
                closest = intersections[i];
            }
        }

        collisionPoint = closest;
        return true;
    }

    bool Collider2D::CollideCapsuleLine(const Capsule& capsule, const Vector2& A, const Vector2& B, Vector2& collisionPoint, Vector2& normal)
    {
        float dist = Vector2::distance(capsule.circle1().center(), capsule.circle2().center());
        Vector2 dirHori = (capsule.circle1().center() - capsule.circle2().center()) / dist;
        Vector2 dirVerti = dirHori.NormalVector();
        dist *= 0.5f;

        Vector2 mid = capsule.hitbox().center();
        float radius = capsule.circle1().radius;

        Line2D line1(mid + dirVerti * radius - dirHori * dist, mid + dirVerti * radius + dirHori * dist);
        Line2D line2(mid - dirVerti * radius - dirHori * dist, mid - dirVerti * radius + dirHori * dist);

        Vector2 inter, inter2;
        normal = Vector2::zero();
        vector<Vector2> intersections;

        if (CollideLines(line1.A, line1.B, A, B, inter)) 
        {
            intersections.push_back(inter);
            normal = (line1.B - line1.A).NormalVector();
            if (Vector2::dot(normal, inter - capsule.center()) < 0.0f)
            {
                normal = normal * -1.0f;
            }
        }
        if (CollideLines(line2.A, line2.B, A, B, inter))
        {
            intersections.push_back(inter);
            normal = (line2.B - line2.A).NormalVector();
            if (Vector2::dot(normal, inter - capsule.center()) < 0.0f)
            {
                normal = normal * -1.0f;
            }
        }

        vector<Vector2> circleInters = ComputeCircleLineIntersections(capsule.circle1(), A, B);
        for (const Vector2& pt : circleInters) 
        {
            if (Vector2::dot(dirHori, pt - capsule.circle1().center()) > 0.0f) 
            {
                intersections.push_back(pt);
                normal = Vector2::normalize(pt - capsule.circle1().center());
            }
        }

        circleInters = ComputeCircleLineIntersections(capsule.circle2(), A, B);
        for (const Vector2 pt : circleInters) 
        {
            if (Vector2::dot(dirHori, pt - capsule.circle2().center()) < 0.0f) 
            {
                intersections.push_back(pt);
                normal = Vector2::normalize(pt - capsule.circle2().center());
            }
        }

        if (intersections.size() <= 1u) 
        {
            if (intersections.size() == 1u)
            {
                collisionPoint = intersections[0];
                intersections.clear();
                return true;
            }

            if (capsule.Contains(A)) 
            {
                if (CollideLineStraightLine(line1.A, line1.B, A, B, inter)) 
                {
                    intersections.push_back(inter);
                }

                if (CollideLineStraightLine(line2.A, line2.B, A, B, inter)) 
                {
                    intersections.push_back(inter);
                }

                if (ComputeCircleStraightLineIntersections(capsule.circle1(), A, B, inter, inter2)) 
                {
                    if (Vector2::dot(dirHori, inter - capsule.circle1().center()) > 0.0f) 
                    {
                        intersections.push_back(inter);
                    }

                    if (Vector2::dot(dirHori, inter2 - capsule.circle1().center()) > 0.0f)
                    {
                        intersections.push_back(inter2);
                    }
                }

                if (ComputeCircleStraightLineIntersections(capsule.circle2(), A, B, inter, inter2))
                {
                    if (Vector2::dot(dirHori, inter - capsule.circle2().center()) < 0.0f) 
                    {
                        intersections.push_back(inter);
                    }

                    if (Vector2::dot(dirHori, inter2 - capsule.circle2().center()) < 0.0f) 
                    {
                        intersections.push_back(inter2);
                    }
                }
            }
            else 
            {
                collisionPoint = Vector2::zero();
                return false;
            }
        }

        if (intersections.empty())
        {
            collisionPoint = Vector2::zero();
            return false;
        }

        collisionPoint = Vector2::zero();
        for (const Vector2& pt : intersections)
        {
            collisionPoint = collisionPoint + pt;
        }
        collisionPoint = collisionPoint / static_cast<float>(intersections.size());

        Vector2 otherPoint = collisionPoint + (B - A).NormalVector();
        if (CollideLineStraightLine(line1.A, line1.B, collisionPoint, otherPoint, inter)) 
        {
            intersections.push_back(inter);
            Vector2 n = (line1.B - line1.A).NormalVector();
            if (Vector2::dot(n, inter - capsule.center()) < 0.0f) 
            {
                n = n * -1.0f;
            }
            intersections.push_back(n);
        }

        if (CollideLineStraightLine(line2.A, line2.B, collisionPoint, otherPoint, inter)) 
        {
            intersections.push_back(inter);
            Vector2 n = (line2.B - line2.A).NormalVector();
            if (Vector2::dot(n, inter - capsule.center()) < 0.0f) 
            {
                n = n * -1.0f;
            }
            intersections.push_back(n);
        }

        vector<Vector2> normals;
        if (ComputeCircleStraightLineIntersections(capsule.circle1(), collisionPoint, otherPoint, inter, inter2)) 
        {
            if (Vector2::dot(dirHori, inter - capsule.circle1().center()) > 0.0f) 
            {
                intersections.push_back(inter);
                normals.push_back(Vector2::normalize(inter - capsule.circle1().center()));
            }
            if (Vector2::dot(dirHori, inter2 - capsule.circle1().center()) > 0.0f) 
            {
                intersections.push_back(inter2);
                normals.push_back(Vector2::normalize(inter2 - capsule.circle1().center()));
            }
        }
        if (ComputeCircleStraightLineIntersections(capsule.circle2(), collisionPoint, otherPoint, inter, inter2)) 
        {
            if (Vector2::dot(dirHori, inter - capsule.circle2().center()) < 0.0f) 
            {
                intersections.push_back(inter);
                normals.push_back(Vector2::normalize(inter - capsule.circle2().center()));
            }
            if (Vector2::dot(dirHori, inter2 - capsule.circle2().center()) < 0.0f) 
            {
                intersections.push_back(inter2);
                normals.push_back(Vector2::normalize(inter2 - capsule.circle2().center()));
            }
        }

        inter = intersections[0];
        float minSqr = Vector2::sqrDistance(collisionPoint, inter);
        uint32_t minIdx = 0u;
        for (uint32_t i = 1u; i < intersections.size(); ++i)
        {
            float d = Vector2::sqrDistance(collisionPoint, intersections[i]);
            if (d < minSqr) 
            {
                minSqr = d;
                inter = intersections[i];
                minIdx = i;
            }
        }

        collisionPoint = inter;
        normal = normals[minIdx];
        return true;
    }

    bool Collider2D::CollideCapsuleLine(const Capsule& capsule, const Line2D& line, Vector2& collisionPoint, Vector2& normal)
    {
        return CollideCapsuleLine(capsule, line.A, line.B, collisionPoint, normal);
    }

    bool Collider2D::CollideCapsuleStraightLine(const Capsule& capsule, const StraightLine2D& line)
    {
        return CollideCapsuleStraightLine(capsule, line.A, line.B);
    }

    bool Collider2D::CollideCapsuleStraightLine(const Capsule& capsule, const Vector2& A, const Vector2& B)
    {
        return CollideCircleStraightLine(*capsule._circle1, A, B) || CollideCircleStraightLine(*capsule._circle2, A, B) || CollideHitboxStraightLine(*capsule._hitbox, A, B);
    }

    bool Collider2D::CollideCapsuleStraightLine(const Capsule& capsule, const Vector2& A, const Vector2& B, Vector2& collisionPoint)
    {
        const Vector2& c1 = capsule._circle1->center();
        const Vector2& c2 = capsule._circle2->center();
        float dist = Vector2::distance(c1, c2);

        Vector2 dirHori = (c1 - c2) / dist;
        Vector2 dirVerti = dirHori.NormalVector();
        dist *= 0.5f;

        Vector2 center = capsule._hitbox->center();
        float radius = capsule._circle1->radius;

        Line2D line1(
            center + dirVerti * radius - dirHori * dist,
            center + dirVerti * radius + dirHori * dist
        );
        Line2D line2(
            center - dirVerti * radius - dirHori * dist,
            center - dirVerti * radius + dirHori * dist
        );

        Vector2 inter, inter2;
        vector<Vector2> intersections;
        if (CollideLineStraightLine(line1.A, line1.B, A, B, inter))
        {
            intersections.push_back(inter);
        }

        if (CollideLineStraightLine(line2.A, line2.B, A, B, inter))
        {
            intersections.push_back(inter);
        }

        if (ComputeCircleStraightLineIntersections(*capsule._circle1, A, B, inter, inter2))
        {
            if (Vector2::dot(dirHori, inter - c1) > 0.0f) 
            {
                intersections.push_back(inter);
            }

            if (Vector2::dot(dirHori, inter2 - c1) > 0.0f) 
            {
                intersections.push_back(inter2);
            }
        }

        if (ComputeCircleStraightLineIntersections(*capsule._circle2, A, B, inter, inter2))
        {
            if (Vector2::dot(dirHori, inter - c2) < 0.0f) 
            {
                intersections.push_back(inter);
            }

            if (Vector2::dot(dirHori, inter2 - c2) < 0.0f) 
            {
                intersections.push_back(inter2);
            }
        }

        if (intersections.empty())
        {
            collisionPoint = Vector2::zero();
            return false;
        }

        collisionPoint = intersections[0];
        for (size_t i = 1; i < intersections.size(); ++i)
        {
            collisionPoint = collisionPoint + intersections[i];
        }
        collisionPoint = collisionPoint / static_cast<float>(intersections.size());

        Vector2 otherPoint = collisionPoint + (B - A).NormalVector();
        if (CollideLineStraightLine(line1.A, line1.B, collisionPoint, otherPoint, inter))
        {
            intersections.push_back(inter);
        }

        if (CollideLineStraightLine(line2.A, line2.B, collisionPoint, otherPoint, inter))
        {
            intersections.push_back(inter);
        }

        if (ComputeCircleStraightLineIntersections(*capsule._circle1, collisionPoint, otherPoint, inter, inter2))
        {
            if (Vector2::dot(dirHori, inter - c1) > 0.0f) 
            {
                intersections.push_back(inter);
            }

            if (Vector2::dot(dirHori, inter2 - c1) > 0.0f) 
            {
                intersections.push_back(inter2);
            }
        }

        if (ComputeCircleStraightLineIntersections(*capsule._circle2, collisionPoint, otherPoint, inter, inter2))
        {
            if (Vector2::dot(dirHori, inter - c2) < 0.0f) 
            {
                intersections.push_back(inter);
            }

            if (Vector2::dot(dirHori, inter2 - c2) < 0.0f) 
            {
                intersections.push_back(inter2);
            }
        }

        if (intersections.empty())
        {
            collisionPoint = Vector2::zero();
            return false;
        }

        inter = intersections[0];
        float minSqrDistance = Vector2::sqrDistance(collisionPoint, inter);

        for (size_t i = 1; i < intersections.size(); ++i)
        {
            float d = Vector2::sqrDistance(collisionPoint, intersections[i]);
            if (d < minSqrDistance)
            {
                minSqrDistance = d;
                inter = intersections[i];
            }
        }

        collisionPoint = inter;
        return true;
    }

    bool Collider2D::CollideCapsuleStraightLine(const Capsule& capsule, const StraightLine2D& straightLine, Vector2& collisionPoint)
    {
        return CollideCapsuleStraightLine(capsule, straightLine.A, straightLine.B, collisionPoint);
    }

    bool Collider2D::CollideCapsuleStraightLine(const Capsule& capsule, const StraightLine2D& straightLine, Vector2& collisionPoint, Vector2& normal)
    {
        return CollideCapsuleStraightLine(capsule, straightLine.A, straightLine.B, collisionPoint, normal);
    }

    bool Collider2D::CollideCapsuleStraightLine(const Capsule& capsule, const Vector2& A, const Vector2& B, Vector2& collisionPoint, Vector2& normal)
    {
        float dist = Vector2::distance(capsule._circle1->center(), capsule._circle2->center());
        Vector2 dirHori = Vector2::normalize(capsule._circle1->center() - capsule._circle2->center());
        Vector2 dirVerti = dirHori.NormalVector();
        dist *= 0.5f;

        Vector2 center = capsule._hitbox->center();
        float radius = capsule._circle1->radius;

        Line2D line1(center + dirVerti * radius - dirHori * dist, center + dirVerti * radius + dirHori * dist);
        Line2D line2(center - dirVerti * radius - dirHori * dist, center - dirVerti * radius + dirHori * dist);

        Vector2 inter, inter2;
        vector<Vector2> intersections;

        if (CollideLines(line1.A, line1.B, A, B, inter)) 
        {
            intersections.push_back(inter);
        }

        if (CollideLines(line2.A, line2.B, A, B, inter)) 
        {
            intersections.push_back(inter);
        }

        if (ComputeCircleStraightLineIntersections(capsule.circle1(), A, B, inter, inter2)) 
        {
            if (Vector2::dot(dirHori, inter - capsule._circle1->center()) > 0.f) 
            {
                intersections.push_back(inter);
            }

            if (Vector2::dot(dirHori, inter2 - capsule._circle1->center()) > 0.f) 
            {
                intersections.push_back(inter2);
            }
        }

        if (ComputeCircleStraightLineIntersections(capsule.circle2(), A, B, inter, inter2))
        {
            if (Vector2::dot(dirHori, inter - capsule._circle2->center()) < 0.0f) 
            {
                intersections.push_back(inter);
            }

            if (Vector2::dot(dirHori, inter2 - capsule._circle2->center()) < 0.0f) 
            {
                intersections.push_back(inter2);
            }
        }

        if (intersections.empty())
        {
            collisionPoint = normal = Vector2::zero();
            return false;
        }

        collisionPoint = Vector2::zero();
        for (const Vector2& p : intersections)
        {
            collisionPoint = collisionPoint + p;
        }
        collisionPoint = collisionPoint / static_cast<float>(intersections.size());

        Vector2 otherPoint = collisionPoint + (B - A).NormalVector();
        vector<Vector2> normals;

        if (CollideLineStraightLine(line1.A, line1.B, collisionPoint, otherPoint, inter)) 
        {
            intersections.push_back(inter);
            normal = (line1.B - line1.A).NormalVector();
            if (Vector2::dot(normal, inter - capsule.center()) < 0.f) 
            {
                normal = -1.0f * normal;
            }
            normals.push_back(normal);
        }

        if (CollideLineStraightLine(line2.A, line2.B, collisionPoint, otherPoint, inter)) 
        {
            intersections.push_back(inter);
            normal = (line2.B - line2.A).NormalVector();
            if (Vector2::dot(normal, inter - capsule.center()) < 0.f) 
            {
                normal = -1.0f * normal;
            }
            normals.push_back(normal);
        }

        if (ComputeCircleStraightLineIntersections(capsule.circle1(), collisionPoint, otherPoint, inter, inter2))
        {
            if (Vector2::dot(dirHori, inter - capsule._circle1->center()) > 0.0f)
            {
                intersections.push_back(inter);
                normals.push_back(Vector2::normalize(inter - capsule._circle1->center()));
            }
            if (Vector2::dot(dirHori, inter2 - capsule._circle1->center()) > 0.0f)
            {
                intersections.push_back(inter2);
                normals.push_back(Vector2::normalize(inter2 - capsule._circle1->center()));
            }
        }

        if (ComputeCircleStraightLineIntersections(capsule.circle2(), collisionPoint, otherPoint, inter, inter2))
        {
            if (Vector2::dot(dirHori, inter - capsule._circle2->center()) < 0.0f) 
            {
                intersections.push_back(inter);
                normals.push_back(Vector2::normalize(inter - capsule._circle2->center()));
            }

            if (Vector2::dot(dirHori, inter2 - capsule._circle2->center()) < 0.0f)
            {
                intersections.push_back(inter2);
                normals.push_back(Vector2::normalize(inter2 - capsule._circle2->center()));
            }
        }

        if (intersections.empty())
        {
            collisionPoint = normal = Vector2::zero();
            return false;
        }

        float minSqrDist = Vector2::sqrDistance(collisionPoint, intersections[0]);
        uint32_t minIndex = 0u;

        for (size_t i = 1u; i < intersections.size(); i++)
        {
            float sqrDist = Vector2::sqrDistance(collisionPoint, intersections[i]);
            if (sqrDist < minSqrDist) 
            {
                minSqrDist = sqrDist;
                minIndex = i;
            }
        }

        collisionPoint = intersections[minIndex];
        normal = normals[minIndex];

        return true;
    }

    bool Collider2D::CollideCapsuleRay(const Capsule& capsule, const Ray2D& ray)
    {
        return CollideCapsuleLine(capsule, ray.start, ray.end);
    }

    bool Collider2D::CollideCapsuleRay(const Capsule& capsule, const Vector2& start, const Vector2& end)
    {
        return CollideCapsuleLine(capsule, start, end);
    }

    bool Collider2D::CollideCapsuleRay(const Capsule& capsule, const Ray2D& ray, Vector2& collisionPoint)
    {
        return CollideCapsuleRay(capsule, ray.start, ray.end, collisionPoint);
    }

    bool Collider2D::CollideCapsuleRay(const Capsule& capsule, const Vector2& start, const Vector2& end, Vector2& collisionPoint)
    {
        Vector2 cp0, cp1, cp2;
        bool hit0 = CollideCircleRay(capsule.circle1(), start, end, cp0);
        bool hit1 = CollideCircleRay(capsule.circle2(), start, end, cp1);
        bool hit2 = CollideHitboxRay(capsule.hitbox(), start, end, cp2);
        const Vector2 dir = Vector2::normalize(start - end);
        const Vector2 beg = start + dir * (2.0f * capsule.inclusiveCircle().radius);

        if (hit0 && hit1 && hit2)
        {
            float d0 = Vector2::sqrDistance(beg, cp0);
            float d1 = Vector2::sqrDistance(beg, cp1);
            float d2 = Vector2::sqrDistance(beg, cp2);

            collisionPoint = (d0 <= d1) ? ((d0 <= d2) ? cp0 : cp2) : ((d1 <= d2) ? cp1 : cp2);
            return true;
        }

        if (hit0 && hit1)
        {
            float d0 = Vector2::sqrDistance(beg, cp0);
            float d1 = Vector2::sqrDistance(beg, cp1);
            collisionPoint = (d0 <= d1) ? cp0 : cp1;
            return true;
        }

        if (hit0 && hit2)
        {
            float d0 = Vector2::sqrDistance(beg, cp0);
            float d2 = Vector2::sqrDistance(beg, cp2);
            collisionPoint = (d0 <= d2) ? cp0 : cp2;
            return true;
        }

        if (hit1 && hit2)
        {
            float d1 = Vector2::sqrDistance(beg, cp1);
            float d2 = Vector2::sqrDistance(beg, cp2);
            collisionPoint = (d1 <= d2) ? cp1 : cp2;
            return true;
        }

        if (hit0)
        {
            collisionPoint = cp0;
            return true;
        }

        if (hit1)
        {
            collisionPoint = cp1;
            return true;
        }

        if (hit2)
        {
            collisionPoint = cp2;
            return true;
        }

        collisionPoint = Vector2::zero();
        return false;
    }

    bool Collider2D::CollideCapsuleRay(const Capsule& capsule, const Ray2D& ray, Vector2& collisionPoint, Vector2& normal)
    {
        return CollideCapsuleRay(capsule, ray.start, ray.end, collisionPoint, normal);
    }

    bool Collider2D::CollideCapsuleRay(const Capsule& capsule, const Vector2& start, const Vector2& end, Vector2& collisionPoint, Vector2& normal)
    {
        Vector2 dir = start - end;
        dir.Normalize();
        Vector2 beg = start + dir * (2.0f * capsule.inclusiveCircle().radius);

        Vector2 cp0, cp1, cp2;
        Vector2 n0, n1, n2;

        bool hit0 = CollideCircleRay(capsule.circle1(), start, end, cp0, n0);
        bool hit1 = CollideCircleRay(capsule.circle2(), start, end, cp1, n1);
        bool hit2 = CollideHitboxRay(capsule.hitbox(), start, end, cp2, n2);

        if (hit0 && hit1)
        {
            float d0 = Vector2::sqrDistance(beg, cp0);
            float d1 = Vector2::sqrDistance(beg, cp1);

            if (hit2)
            {
                float d2 = Vector2::sqrDistance(beg, cp2);

                if (d0 <= d1)
                {
                    if (d0 <= d2)
                    {
                        collisionPoint = cp0;
                        normal = n0;
                    }
                    else
                    {
                        collisionPoint = cp2;
                        normal = n2;
                    }
                }
                else
                {
                    if (d1 <= d2)
                    {
                        collisionPoint = cp1;
                        normal = n1;
                    }
                    else
                    {
                        collisionPoint = cp2;
                        normal = n2;
                    }
                }

                return true;
            }

            if (d0 <= d1)
            {
                collisionPoint = cp0;
                normal = n0;
            }
            else
            {
                collisionPoint = cp1;
                normal = n1;
            }

            return true;
        }

        if (hit0)
        {
            float d0 = Vector2::sqrDistance(beg, cp0);

            if (hit2)
            {
                float d2 = Vector2::sqrDistance(beg, cp2);

                if (d0 <= d2)
                {
                    collisionPoint = cp0;
                    normal = n0;
                }
                else
                {
                    collisionPoint = cp2;
                    normal = n2;
                }

                return true;
            }

            collisionPoint = cp0;
            normal = n0;
            return true;
        }

        if (hit1)
        {
            float d1 = Vector2::sqrDistance(beg, cp1);

            if (hit2)
            {
                float d2 = Vector2::sqrDistance(beg, cp2);

                if (d1 <= d2)
                {
                    collisionPoint = cp1;
                    normal = n1;
                }
                else
                {
                    collisionPoint = cp2;
                    normal = n2;
                }

                return true;
            }

            collisionPoint = cp1;
            normal = n1;
            return true;
        }

        if (hit2)
        {
            collisionPoint = cp2;
            normal = n2;
            return true;
        }

        collisionPoint = Vector2::zero();
        normal = Vector2::zero();
        return false;
    }

    #pragma endregion
}

