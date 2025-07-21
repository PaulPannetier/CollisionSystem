#pragma once

#include <vector>
#include "Collider2D.hpp"

namespace ToricCollisionSystem
{
    class Polygone : public Collider2D 
    {
        friend class Collider2D;

    private:
        std::vector<Vector2> _vertices;
        Vector2 _center;
        Circle* _inclusiveCircle;

    protected:
        void ComputeInclusiveCircle();
        bool IsNormalOnRightDirection(const Vector2& point, const Vector2& n, int indexSide) const;

    public:
        Polygone() = default;
        Polygone(const std::vector<Vector2>& vertices);
        Polygone(const Polygone& polygone);
        const Circle& inclusiveCircle() const override { return *_inclusiveCircle; }

        const Vector2& center() const override { return _center; }
        const std::vector<Vector2>& vertices() const { return _vertices; }
        bool Collide(const Collider2D& other) const override;
        bool CollideLine(const Line2D& line) const override;
        bool CollideStraightLine(const StraightLine2D& line) const override;
        bool Contains(const Vector2& point) const override;
        float Distance(const Vector2& point) const override;
        float SignedDistance(const Vector2& point) const override;
        float Area() const override;
        Vector2 ClosestPoint(const Vector2& point) const override;
        void MoveAt(const Vector2& position) override;
        void Rotate(float angle) override;
        void Scale(const Vector2& scale) override;
        Hitbox* ToHitbox() const override;
        bool Normal(const Vector2& point, Vector2& outNormal) const override;
        std::string ToString() const override;
        ~Polygone() override;
    };
}