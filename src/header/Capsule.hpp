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

        void Builder(const Vector2& center, const Vector2& size, CapsuleDirection2D direction, float rotation);
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
        bool Collide(const Collider2D& c) const override;
        bool CollideLine(const Line2D& line) const override;
        bool CollideStraightLine(const StraightLine2D& line) const override;
        bool Contains(const Vector2& point) const override;
        float Distance(const Vector2& point) const override;
        float SignedDistance(const Vector2& point) const override;
        float Area() const override;
        Vector2 ClosestPoint(const Vector2& point) const override;
        void MoveAt(const Vector2& pos) override;
        void Rotate(float angle) override;
        void Scale(const Vector2& scale) override;
        float AngleHori() const;
        Hitbox* ToHitbox() const override;
        bool Normal(const Vector2& point, Vector2& outNormal) const override;
        std::string ToString() const override;
        ~Capsule() override;
    };
}