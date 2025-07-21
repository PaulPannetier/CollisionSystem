#pragma once

#include <string>
#include <vector>
#include "Vector2.hpp"
#include "Line2D.hpp"
#include "Straightline2D.hpp"
#include "Ray2D.hpp"

namespace ToricCollisionSystem
{
    class Circle;
    class Polygone;
    class Hitbox;
    class Capsule;

    class Collider2D
    {
        #pragma region static methods

    private:
        static bool ComputeCirclesIntersections(const Circle& circle1, const Circle& circle2, Vector2& outIntersection1, Vector2& outIntersection2);
        static bool ComputeCircleStraightLineIntersections(const Circle& circle, const Vector2& A, const Vector2& B, Vector2& outIntersection1, Vector2& outIntersection2);
        static std::pair<Vector2, Vector2> ComputeCircleStraightLineIntersectionsUnchecked(const Circle& circle, const Vector2& A, const Vector2& B);
        static std::vector<Vector2> ComputeCircleLineIntersections(const Circle& circle, const Vector2& A, const Vector2& B);

    public:
        static bool CollideStraightLines(const StraightLine2D& l1, const StraightLine2D& l2);
        static bool CollideStraightLines(const Vector2& A, const Vector2& B, const Vector2& O, const Vector2& P);
        static bool CollideStraightLines(const StraightLine2D& l1, const StraightLine2D& l2, Vector2& outCollisionPoint);
        static bool CollideStraightLines(const Vector2& A, const Vector2& B, const Vector2& O, const Vector2& P, Vector2& outCollisionPoint);
        static bool CollideLines(const Line2D& l1, const Line2D& l2);
        static bool CollideLines(const Vector2& A, const Vector2& B, const Vector2& O, const Vector2& P);
        static bool CollideLines(const Line2D& l1, const Line2D& l2, Vector2& outCollisionPoint);
        static bool CollideLines(const Vector2& A, const Vector2& B, const Vector2& O, const Vector2& P, Vector2& outCollisionPoint);
        static bool CollideLineStraightLine(const Line2D& line, const StraightLine2D& straightLine);
        static bool CollideLineStraightLine(const Vector2& O, const Vector2& P, const Vector2& A, const Vector2& B);
        static bool CollideLineStraightLine(const Line2D& line, const StraightLine2D& straightLine, Vector2& outCollisionPoint);
        static bool CollideLineStraightLine(const Vector2& O, const Vector2& P, const Vector2& A, const Vector2& B, Vector2& outCollisionPoint);
        static bool CollideLineRay(const Line2D& line, const Ray2D& ray);
        static bool CollideLineRay(const Line2D& line, const Ray2D& ray, Vector2& outCollisionPoint);
        static bool CollideStraightLineRay(const StraightLine2D& line, const Ray2D& ray);
        static bool CollideStraightLineRay(const StraightLine2D& line, const Ray2D& ray, Vector2& outCollisionPoint);
        static bool CollideRays(const Ray2D& ray1, const Ray2D& ray2);
        static bool CollideRays(const Vector2& start1, const Vector2& end1, const Vector2& start2, const Vector2& end2);
        static bool CollideRays(const Ray2D& ray1, const Ray2D& ray2, Vector2& outCollisionPoint);
        static bool CollideRays(const Vector2& start1, const Vector2& end1, const Vector2& start2, const Vector2& end2, Vector2& outCollisionPoint);

        static bool CollideCircles(const Circle& circle1, const Circle& circle2);
        static bool CollideCircles(const Circle& circle1, const Circle& circle2, Vector2& outCollisionPoint);
        static bool CollideCircles(const Circle& circle1, const Circle& circle2, Vector2& outCollisionPoint, Vector2& normal1, Vector2& normal2);
        static bool CollideCirclePolygone(const Circle& circle, const Polygone& polygone);
        static bool CollideCirclePolygone(const Circle& circle, const Polygone& polygone, Vector2& collisionPoint);
        static bool CollideCirclePolygone(const Circle& circle, const Polygone& polygone, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2);
        static bool CollideCircleHitbox(const Circle& circle, const Hitbox& hitbox);
        static bool CollideCircleHitbox(const Circle& circle, const Hitbox& hitbox, Vector2& outCollisionPoint);
        static bool CollideCircleHitbox(const Circle& circle, const Hitbox& hitbox, Vector2& outCollisionPoint, Vector2& outNormal1, Vector2& outNormal2);
        static bool CollideCircleLine(const Circle& circle, const Vector2& A, const Vector2& B);
        static bool CollideCircleLine(const Circle& circle, const Line2D& line);
        static bool CollideCircleLine(const Circle& circle, const Vector2& A, const Vector2& B, Vector2& outCollisionPoint);
        static bool CollideCircleLine(const Circle& circle, const Line2D& line, Vector2& outCollisionPoint);
        static bool CollideCircleLine(const Circle& circle, const Vector2& A, const Vector2& B, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollideCircleLine(const Circle& circle, const Line2D& line, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollideCircleStraightLine(const Circle& circle, const StraightLine2D& straightLine);
        static bool CollideCircleStraightLine(const Circle& circle, const Vector2& A, const Vector2& B);
        static bool CollideCircleStraightLine(const Circle& circle, const StraightLine2D& straightLine, Vector2& outCollisionPoint);
        static bool CollideCircleStraightLine(const Circle& circle, const Vector2& A, const Vector2& B, Vector2& outCollisionPoint);
        static bool CollideCircleStraightLine(const Circle& circle, const Vector2& A, const Vector2& B, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollideCircleStraightLine(const Circle& circle, const StraightLine2D& straightLine, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollideCircleRay(const Circle& circle, const Ray2D& ray);
        static bool CollideCircleRay(const Circle& circle, const Vector2& start, const Vector2& end);
        static bool CollideCircleRay(const Circle& circle, const Ray2D& ray, Vector2& outCollisionPoint);
        static bool CollideCircleRay(const Circle& circle, const Vector2& start, const Vector2& end, Vector2& outCollisionPoint);
        static bool CollideCircleRay(const Circle& circle, const Ray2D& ray, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollideCircleRay(const Circle& circle, const Vector2& start, const Vector2& end, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollideCircleCapsule(const Circle& circle, const Capsule& capsule);
        static bool CollideCircleCapsule(const Circle& circle, const Capsule& capsule, Vector2& outCollisionPoint);
        static bool CollideCircleCapsule(const Circle& circle, const Capsule& capsule, Vector2& outCollisionPoint, Vector2& outNormal1, Vector2& outNormal2);

        static bool CollidePolygones(const Polygone& poly1, const Polygone& poly2);
        static bool CollidePolygones(const Polygone& poly1, const Polygone& poly2, Vector2& outCollisionPoint);
        static bool CollidePolygones(const Polygone& poly1, const Polygone& poly2, Vector2& outCollisionPoint, Vector2& outNormal1, Vector2& outNormal2);
        static bool CollidePolygoneHitbox(const Polygone& poly, const Hitbox& hitbox);
        static bool CollidePolygoneHitbox(const Polygone& poly, const Hitbox& hitbox, Vector2& outCollisionPoint);
        static bool CollidePolygoneHitbox(const Polygone& poly, const Hitbox& hitbox, Vector2& outCollisionPoint, Vector2& normal1, Vector2& normal2);
        static bool CollidePolygoneLine(const Polygone& poly, const Vector2& A, const Vector2& B);
        static bool CollidePolygoneLine(const Polygone& poly, const Line2D& line);
        static bool CollidePolygoneLine(const Polygone& poly, const Vector2& A, const Vector2& B, Vector2& outCollisionPoint);
        static bool CollidePolygoneLine(const Polygone& poly, const Line2D& line, Vector2& outCollisionPoint);
        static bool CollidePolygoneLine(const Polygone& poly, const Vector2& A, const Vector2& B, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollidePolygoneLine(const Polygone& poly, const Line2D& line, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollidePolygoneStraightLine(const Polygone& polygone, const Vector2& A, const Vector2& B);
        static bool CollidePolygoneStraightLine(const Polygone& polygone, const StraightLine2D& straightLine);
        static bool CollidePolygoneStraightLine(const Polygone& polygone, const Vector2& A, const Vector2& B, Vector2& outCollisionPoint);
        static bool CollidePolygoneStraightLine(const Polygone& polygone, const StraightLine2D& straightLine, Vector2& outCollisionPoint);
        static bool CollidePolygoneStraightLine(const Polygone& polygone, const Vector2& A, const Vector2& B, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollidePolygoneStraightLine(const Polygone& polygone, const StraightLine2D& straightLine, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollidePolygoneRay(const Polygone& polygone, const Ray2D& ray);
        static bool CollidePolygoneRay(const Polygone& polygone, const Vector2& start, const Vector2& end);
        static bool CollidePolygoneRay(const Polygone& polygone, const Ray2D& ray, Vector2& outCollisionPoint);
        static bool CollidePolygoneRay(const Polygone& polygone, const Vector2& start, const Vector2& end, Vector2& outCollisionPoint);
        static bool CollidePolygoneRay(const Polygone& polygone, const Ray2D& ray, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollidePolygoneRay(const Polygone& polygone, const Vector2& start, const Vector2& end, Vector2& outCollisionPoint, Vector2& outNormal);
        static bool CollidePolygoneCapsule(const Polygone& polygone, const Capsule& capsule);
        static bool CollidePolygoneCapsule(const Polygone& polygone, const Capsule& capsule, Vector2& outCollisionPoint);
        static bool CollidePolygoneCapsule(const Polygone& polygone, const Capsule& capsule, Vector2& outCollisionPoint, Vector2& outNormal1, Vector2& outNormal2);

        static bool CollideHitboxes(const Hitbox& h1, const Hitbox& h2);
        static bool CollideHitboxes(const Hitbox& h1, const Hitbox& h2, Vector2& collisionPoint);
        static bool CollideHitboxes(const Hitbox& h1, const Hitbox& h2, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2);
        static bool CollideHitboxLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B);
        static bool CollideHitboxLine(const Hitbox& hitbox, const Line2D& line);
        static bool CollideHitboxLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B, Vector2& collisionPoint);
        static bool CollideHitboxLine(const Hitbox& hitbox, const Line2D& line, Vector2& collisionPoint);
        static bool CollideHitboxLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B, Vector2& collisionPoint, Vector2& normal);
        static bool CollideHitboxLine(const Hitbox& hitbox, const Line2D& line, Vector2& collisionPoint, Vector2& normal);
        static bool CollideHitboxStraightLine(const Hitbox& hitbox, const StraightLine2D& line);
        static bool CollideHitboxStraightLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B);
        static bool CollideHitboxStraightLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B, Vector2& collisionPoint);
        static bool CollideHitboxStraightLine(const Hitbox& hitbox, const StraightLine2D& line, Vector2& collisionPoint);
        static bool CollideHitboxStraightLine(const Hitbox& hitbox, const Vector2& A, const Vector2& B, Vector2& collisionPoint, Vector2& normal);
        static bool CollideHitboxStraightLine(const Hitbox& hitbox, const StraightLine2D& line, Vector2& collisionPoint, Vector2& normal);
        static bool CollideHitboxRay(const Hitbox& hitbox, const Ray2D& ray);
        static bool CollideHitboxRay(const Hitbox& hitbox, const Vector2& start, const Vector2& end);
        static bool CollideHitboxRay(const Hitbox& hitbox, const Ray2D& ray, Vector2& collisionPoint);
        static bool CollideHitboxRay(const Hitbox& hitbox, const Vector2& start, const Vector2& end, Vector2& collisionPoint);
        static bool CollideHitboxRay(const Hitbox& hitbox, const Ray2D& ray, Vector2& collisionPoint, Vector2& normal);
        static bool CollideHitboxRay(const Hitbox& hitbox, const Vector2& start, const Vector2& end, Vector2& collisionPoint, Vector2& normal);
        static bool CollideHitboxCapsule(const Hitbox& hitbox, const Capsule& capsule);
        static bool CollideHitboxCapsule(const Hitbox& hitbox, const Capsule& capsule, Vector2& collisionPoint);
        static bool CollideHitboxCapsule(const Hitbox& hitbox, const Capsule& capsule, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2);

        static bool CollideCapsules(const Capsule& capsule1, const Capsule& capsule2);
        static bool CollideCapsules(const Capsule& capsule1, const Capsule& capsule2, Vector2& collisionPoint);
        static bool CollideCapsules(const Capsule& capsule1, const Capsule& capsule2, Vector2& collisionPoint, Vector2& normal1, Vector2& normal2);
        static bool CollideCapsuleLine(const Capsule& capsule, const Line2D& line);
        static bool CollideCapsuleLine(const Capsule& capsule, const Vector2& A, const Vector2& B);
        static bool CollideCapsuleLine(const Capsule& capsule, const Line2D& line, Vector2& collisionPoint);
        static bool CollideCapsuleLine(const Capsule& capsule, const Vector2& A, const Vector2& B, Vector2& collisionPoint);
        static bool CollideCapsuleLine(const Capsule& capsule, const Vector2& A, const Vector2& B, Vector2& collisionPoint, Vector2& normal);
        static bool CollideCapsuleLine(const Capsule& capsule, const Line2D& line, Vector2& collisionPoint, Vector2& normal);
        static bool CollideCapsuleStraightLine(const Capsule& capsule, const StraightLine2D& line);
        static bool CollideCapsuleStraightLine(const Capsule& capsule, const Vector2& A, const Vector2& B);
        static bool CollideCapsuleStraightLine(const Capsule& capsule, const Vector2& A, const Vector2& B, Vector2& collisionPoint);
        static bool CollideCapsuleStraightLine(const Capsule& capsule, const StraightLine2D& straightLine, Vector2& collisionPoint);
        static bool CollideCapsuleStraightLine(const Capsule& capsule, const StraightLine2D& straightLine, Vector2& collisionPoint, Vector2& normal);
        static bool CollideCapsuleStraightLine(const Capsule& capsule, const Vector2& A, const Vector2& B, Vector2& collisionPoint, Vector2& normal);
        static bool CollideCapsuleRay(const Capsule& capsule, const Ray2D& ray);
        static bool CollideCapsuleRay(const Capsule& capsule, const Vector2& start, const Vector2& end);
        static bool CollideCapsuleRay(const Capsule& capsule, const Ray2D& ray, Vector2& collisionPoint);
        static bool CollideCapsuleRay(const Capsule& capsule, const Vector2& start, const Vector2& end, Vector2& collisionPoint);
        static bool CollideCapsuleRay(const Capsule& capsule, const Ray2D& ray, Vector2& collisionPoint, Vector2& normal);
        static bool CollideCapsuleRay(const Capsule& capsule, const Vector2& start, const Vector2& end, Vector2& collisionPoint, Vector2& normal);

        #pragma endregion

    protected:
        virtual const Circle& inclusiveCircle() const = 0;

    public:
        virtual const Vector2& center() const = 0;

        virtual bool Collide(const Collider2D& other) const = 0;
        virtual bool CollideLine(const Line2D& line) const = 0;
        virtual bool CollideStraightLine(const StraightLine2D& line) const = 0;
        virtual bool Contains(const Vector2& point) const = 0;
        virtual float Distance(const Vector2& point) const = 0;
        virtual float SignedDistance(const Vector2& point) const = 0;
        virtual float Area() const = 0;
        virtual Vector2 ClosestPoint(const Vector2& point) const = 0;
        virtual void MoveAt(const Vector2& position) = 0;
        virtual void Rotate(float angle) = 0;
        virtual void Scale(const Vector2& scale) = 0;
        virtual Hitbox* ToHitbox() const = 0;

        virtual bool Normal(const Vector2& point, Vector2& outNormal) const
        {
            outNormal = Vector2::zero();
            return false;
        }
        virtual std::string ToString() const = 0;
        virtual ~Collider2D() = default;
    };
}
