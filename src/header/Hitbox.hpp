#pragma once

#include "Collider2D.hpp"

namespace ToricCollisionSystem
{
    class Hitbox : public Collider2D 
    {
    protected:
        Polygone* _rec;
        Vector2 _size;
        
    public:
        Hitbox() = default;
        Hitbox(const Vector2& center, const Vector2& size);
        Hitbox(const Hitbox& hitbox);

        const std::vector<Vector2>& vertices() const;
        const Vector2& size() const;
        const Vector2& center() const override;
        const Circle& inclusiveCircle() const override;
        Polygone* toPolygone() const;

        bool Collide(const Collider2D& other) const override;
        bool CollideLine(const Line2D& line) const override;
        bool CollideStraightLine(const StraightLine2D& line) const override;
        bool Contains(const Vector2& point) const override;
        float Distance(const Vector2& point) const override;
        float SignedDistance(const Vector2& point) const override;
        float Area() const override;
        Vector2 ClosestPoint(const Vector2& point) const override;
        void MoveAt(const Vector2& position) override;
        void Scale(const Vector2& scale) override;
        void Rotate(float angle) override;
        float AngleHori() const;
        Hitbox* ToHitbox() const override;
        const Polygone& ToPolygone() const;
        bool Normal(const Vector2& point, Vector2& outNormal) const override;
        std::string ToString() const override;
        ~Hitbox() override;
    };
}
