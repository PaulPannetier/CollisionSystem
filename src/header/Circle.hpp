#pragma once

#include "Collider2D.hpp"

namespace ToricCollisionSystem
{
    class Circle : public Collider2D
    {
    protected:
        Vector2 _center;

    public:
        float radius;
        Circle();
        Circle(const Vector2& center, float radius);
        Circle(const Circle& circle);

        const Circle& inclusiveCircle() const override;
        const Vector2& center() const override { return _center; }
        void center(const Vector2& center) { _center = center; }

        bool Collide(const Collider2D& other) const override;
        bool CollideLine(const Line2D& line) const override;
        bool CollideStraightLine(const StraightLine2D& straightLine) const override;
        bool Contains(const Vector2& p) const override;
        float Distance(const Vector2& point) const override;
        float SignedDistance(const Vector2& point) const override;
        float Area() const override;
        Vector2 ClosestPoint(const Vector2& point) const override;
        Hitbox* ToHitbox() const override;
        void MoveAt(const Vector2& position) override;
        void Rotate(float angle) override;
        void Scale(const Vector2& scale) override;
        bool Normal(const Vector2& point, Vector2& normal) const override;
        std::string ToString() const override;
    };
}