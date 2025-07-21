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
        bool contains(const Vector2& point) const override;
        float distance(const Vector2& point) const override;
        float signedDistance(const Vector2& point) const override;
        float area() const override;
        Vector2 closestPoint(const Vector2& point) const override;
        void moveAt(const Vector2& position) override;
        void rotate(float angle) override;
        void scale(const Vector2& scale) override;
        bool normal(const Vector2& point, Vector2& outNormal) const override;
        std::string toString() const override;
        ~Polygone() override;
    };
}