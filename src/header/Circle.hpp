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

        bool contains(const Vector2& p) const override;
        float distance(const Vector2& point) const override;
        float signedDistance(const Vector2& point) const override;
        float area() const override;
        Vector2 closestPoint(const Vector2& point) const override;
        void moveAt(const Vector2& position) override;
        void rotate(float angle) override;
        void scale(const Vector2& scale) override;
        bool normal(const Vector2& point, Vector2& normal) const override;
        std::string toString() const override;
    };
}