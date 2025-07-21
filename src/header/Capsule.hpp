#pragma once

#include "Collider2D.hpp"

namespace ToricCollisionSystem
{
    class Capsule : public Collider2D
    {
        friend class Collider2D;

    public:
        enum class CapsuleDirection2D { Vertical, Horizontal };

    private:
        Circle* _circle1;
        Circle* _circle2;
        Hitbox* _hitbox;
        Circle* _inclusiveCircle;

        void builder(const Vector2& center, const Vector2& size, CapsuleDirection2D direction, float rotation);
        const Circle& inclusiveCircle() const override;
        const Circle& circle1() const;
        const Circle& circle2() const;
        const Hitbox& hitbox() const;

    public:
        Capsule() = default;
        Capsule(const Vector2& center, const Vector2& size);
        Capsule(const Vector2& center, const Vector2& size, float angle);
        Capsule(const Vector2& center, const Vector2& size, CapsuleDirection2D direction);
        Capsule(const Vector2& center, const Vector2& size, CapsuleDirection2D direction, float angle);
        Capsule(const Capsule& capsule);

        const Vector2& center() const override;
        bool collide(const Collider2D& c) const override;
        bool collideLine(const Line2D& line) const override;
        bool collideStraightLine(const StraightLine2D& line) const override;
        bool contains(const Vector2& point) const override;
        float distance(const Vector2& point) const override;
        float signedDistance(const Vector2& point) const override;
        float area() const override;
        Vector2 closestPoint(const Vector2& point) const override;
        void moveAt(const Vector2& pos) override;
        void rotate(float angle) override;
        void scale(const Vector2& scale) override;
        float angleHori() const;
        bool normal(const Vector2& point, Vector2& outNormal) const override;
        std::string toString() const override;
        ~Capsule() override;
    };
}