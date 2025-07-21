#include <cmath>
#include <sstream>
#include "Collider2D.hpp"
#include "Hitbox.hpp"
#include "Circle.hpp"
#include "Polygone.hpp"
#include "Useful.hpp"

namespace ToricCollisionSystem
{
    Circle::Circle() : _center(), radius(0.0f)
    {

    }

    Circle::Circle(const Vector2& center, float radius) : _center(center), radius(radius)
    {

    }

    Circle::Circle(const Circle& circle) : _center(circle._center), radius(circle.radius)
    {

    }

    const Circle& Circle::inclusiveCircle() const
    {
        return *this;
    }

    bool Circle::Collide(const Collider2D& other) const
    {
        //return Collider2D::Collide(collider, this);
        return false;
    }

    bool Circle::CollideLine(const Line2D& line) const
    {
        return Collider2D::CollideCircleLine(*this, line.A, line.B);
    }

    bool Circle::CollideStraightLine(const StraightLine2D& straightLine) const
    {
        return Collider2D::CollideCircleStraightLine(*this, straightLine.A, straightLine.B);
    }

    bool Circle::Contains(const Vector2& p) const
    {
        return Vector2::sqrDistance(_center, p) <= radius * radius;
    }

    float Circle::Distance(const Vector2& point) const
    {
        return std::max(0.0f, Vector2::distance(point, _center) - radius);
    }

    float Circle::SignedDistance(const Vector2& point) const
    {
        return Vector2::distance(point, _center) - radius;
    }

    float Circle::Area() const
    {
        return M_PI * radius * radius;
    }

    Vector2 Circle::ClosestPoint(const Vector2& point) const
    {
        Vector2 direction = point - _center;
        direction.Normalize();
        return _center + direction * radius;
    }

    void Circle::MoveAt(const Vector2& position)
    {
        _center = position;
    }

    void Circle::Rotate(float angle)
    {
        
    }

    void Circle::Scale(const Vector2& scale)
    {
        radius *= std::max(scale.x, scale.y);
    }

    Hitbox* Circle::ToHitbox() const
    {
        return new Hitbox(_center, { radius, radius });
    }

    bool Circle::Normal(const Vector2& point, Vector2& normal) const
    {
        if (Useful::approximately(Vector2::sqrDistance(_center, point), radius * radius))
        {
            normal = point - _center;
            normal.Normalize();
            return true;
        }
        return Collider2D::Normal(point, normal);
    }

    std::string Circle::ToString() const
    {
        std::ostringstream oss;
        oss << "center: (" << _center.x << ", " << _center.y << ") Radius: " << radius;
        return oss.str();
    }
}