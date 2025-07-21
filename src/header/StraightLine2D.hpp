#pragma once

#include "Vector2.hpp"

namespace ToricCollisionSystem
{
    class StraightLine2D 
    {
    public:
        Vector2 A, B;

        StraightLine2D(const Vector2& A, const Vector2& B);

        Vector2 Symetric(const Vector2& M) const;
        static Vector2 Symetric(const Vector2& M, const Vector2& A, const Vector2& B);

        static Vector2 Reflection(const Vector2& normal, const Vector2& interPointInDroite, const Vector2& initDir);

        Vector2 OrthogonalProjection(const Vector2& M) const;
        static Vector2 OrthogonalProjection(const Vector2& M, const Vector2& A, const Vector2& B);

        bool Contain(const Vector2& point) const;
        static bool Contain(const Vector2& A, const Vector2& B, const Vector2& point);

        float Distance(const Vector2& point) const;
        static float Distance(const Vector2& A, const Vector2& B, const Vector2& point);

        Vector2 Normal() const;
        static Vector2 Normal(const Vector2& A, const Vector2& B);

        Vector2 ClosestPoint(const Vector2& point) const;
        static Vector2 ClosestPoint(const Vector2& A, const Vector2& B, const Vector2& point);
    };
}
