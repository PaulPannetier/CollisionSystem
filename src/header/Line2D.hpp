#pragma once

#include "Vector2.hpp"

namespace ToricCollisionSystem
{
    class Line2D
    {
    public:
        Vector2 A, B;

        Line2D() = default;
        Line2D(const Vector2& A, const Vector2& B);
        Line2D(const Vector2& start, float angle, float length);

        bool Contain(const Vector2& point) const;
        static bool Contain(const Vector2& A, const Vector2& B, const Vector2& point);

        float Distance(const Vector2& point) const;
        static float Distance(const Vector2& A, const Vector2& B, const Vector2& point);

        float SqrDistance(const Vector2& point) const;
        static float SqrDistance(const Vector2& A, const Vector2& B, const Vector2& point);

        Vector2 Normal() const;
        static Vector2 Normal(const Vector2& A, const Vector2& B);

        Vector2 ClosestPoint(const Vector2& point) const;
        static Vector2 ClosestPoint(const Vector2& A, const Vector2& B, const Vector2& point);
    };
}
