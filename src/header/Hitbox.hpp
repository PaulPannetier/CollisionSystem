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

        bool collide(const Collider2D& other) const override;
        bool collideLine(const Line2D& line) const override;
        bool collideStraightLine(const StraightLine2D& line) const override;
        bool contains(const Vector2& point) const override;
        float distance(const Vector2& point) const override;
        float signedDistance(const Vector2& point) const override;
        float area() const override;
        Vector2 closestPoint(const Vector2& point) const override;
        void moveAt(const Vector2& position) override;
        void scale(const Vector2& scale) override;
        void rotate(float angle) override;
        float angleHori() const;
        const Polygone& toPolygone() const;
        bool normal(const Vector2& point, Vector2& outNormal) const override;
        std::string toString() const override;
        ~Hitbox() override;
    };
}
