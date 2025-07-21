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

    bool Circle::contains(const Vector2& p) const
    {
        return Vector2::sqrDistance(_center, p) <= radius * radius;
    }

    float Circle::distance(const Vector2& point) const
    {
        return std::max(0.0f, Vector2::distance(point, _center) - radius);
    }

    float Circle::signedDistance(const Vector2& point) const
    {
        return Vector2::distance(point, _center) - radius;
    }

    float Circle::area() const
    {
        return M_PI * radius * radius;
    }

    Vector2 Circle::closestPoint(const Vector2& point) const
    {
        Vector2 direction = point - _center;
        direction.normalize();
        return _center + direction * radius;
    }

    void Circle::moveAt(const Vector2& position)
    {
        _center = position;
    }

    void Circle::rotate(float angle)
    {
        
    }

    void Circle::scale(const Vector2& scale)
    {
        radius *= std::max(scale.x, scale.y);
    }

    bool Circle::normal(const Vector2& point, Vector2& normal) const
    {
        if (Useful::approximately(Vector2::sqrDistance(_center, point), radius * radius))
        {
            normal = point - _center;
            normal.normalize();
            return true;
        }
        return Collider2D::normal(point, normal);
    }

    std::string Circle::toString() const
    {
        std::ostringstream oss;
        oss << "center: (" << _center.x << ", " << _center.y << ") Radius: " << radius;
        return oss.str();
    }
}