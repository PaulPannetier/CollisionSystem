#include <sstream>
#include "Capsule.hpp"
#include "Hitbox.hpp"
#include "Circle.hpp"
#include "Useful.hpp"
#include "Line2D.hpp"

using namespace std;

namespace ToricCollisionSystem
{
    Capsule::Capsule(const Vector2& center, const Vector2& size) 
    {
        CapsuleDirection2D direction = (size.x >= size.y) ? CapsuleDirection2D::Horizontal : CapsuleDirection2D::Vertical;
        builder(center, size, direction, 0.0f);
    }

    Capsule::Capsule(const Vector2& center, const Vector2& size, float angle) 
    {
        CapsuleDirection2D direction = (size.x >= size.y) ? CapsuleDirection2D::Horizontal : CapsuleDirection2D::Vertical;
        builder(center, size, direction, angle);
    }

    Capsule::Capsule(const Vector2& center, const Vector2& size, CapsuleDirection2D direction) 
    {
        builder(center, size, direction, 0.0f);
    }

    Capsule::Capsule(const Vector2& center, const Vector2& size, CapsuleDirection2D direction, float angle) 
    {
        builder(center, size, direction, angle);
    }

    void Capsule::builder(const Vector2& center, const Vector2& size, CapsuleDirection2D direction, float rotation) 
    {
        if (direction == CapsuleDirection2D::Horizontal) 
        {
            _hitbox = new Hitbox(center, Vector2(max(size.x - size.y, 0.0f), size.y));
            _circle1 = new Circle(Vector2(center.x - _hitbox->size().x * 0.5f, center.y), size.y * 0.5f);
            _circle2 = new Circle(Vector2(center.x + _hitbox->size().x * 0.5f, center.y), size.y * 0.5f);
        }
        else 
        {
            _hitbox = new Hitbox(center, Vector2(size.x, max(size.y - size.x, 0.0f)));
            _circle1 = new Circle(Vector2(center.x, center.y - _hitbox->size().y * 0.5f), size.x * 0.5f);
            _circle2 = new Circle(Vector2(center.x, center.y + _hitbox->size().y * 0.5f), size.x * 0.5f);
        }

        float radius = (Vector2::distance(_circle1->center(), _circle2->center()) + _circle1->radius + _circle2->radius) * 0.5f;
        _inclusiveCircle = new Circle(_hitbox->center(), radius);
        rotate(rotation);
    }

    Capsule::Capsule(const Capsule& capsule)
    {
        _circle1 = new Circle(*capsule._circle1);
        _circle2 = new Circle(*capsule._circle2);
        _inclusiveCircle = new Circle(*capsule._inclusiveCircle);
        _hitbox = new Hitbox(*capsule._hitbox);
    }

    const Circle& Capsule::inclusiveCircle() const
    {
        return *_inclusiveCircle;
    }

    const Vector2& Capsule::center() const
    {
        return _hitbox->center();
    }

    const Circle& Capsule::circle1() const
    {
        return *_circle1;
    }

    const Circle& Capsule::circle2() const
    {
        return *_circle2;
    }

    const Hitbox& Capsule::hitbox() const
    {
        return *_hitbox;
    }

    float Capsule::angleHori() const 
    {
        return _hitbox->angleHori();
    }

    bool Capsule::collideLine(const Line2D& line) const 
    {
        //return CollideCapsuleLine(*this, line.A(), line.B());
        return false;
    }

    bool Capsule::collideStraightLine(const StraightLine2D& line) const 
    {
        //return CollideCapsuleStraightLine(*this, line.A(), line.B());
        return false;
    }

    bool Capsule::collide(const Collider2D& collider) const 
    {
        //return Collider2D::Collide(collider, *this);
        return false;
    }

    Vector2 Capsule::closestPoint(const Vector2& point) const 
    {
        float distance = Vector2::distance(_circle1->center(), _circle2->center());
        Vector2 dirHori = (_circle1->center() - _circle2->center()) / distance;
        Vector2 dirVerti = dirHori.normalVector();
        distance *= 0.5f;

        Vector2 A = _hitbox->center() + dirVerti * _circle1->radius - dirHori * distance;
        Vector2 B = _hitbox->center() + dirVerti * _circle1->radius + dirHori * distance;
        Vector2 A2 = _hitbox->center() - dirVerti * _circle1->radius - dirHori * distance;
        Vector2 B2 = _hitbox->center() - dirVerti * _circle1->radius + dirHori * distance;

        Vector2 cp1 = Line2D::ClosestPoint(A, B, point);
        Vector2 cp2 = Line2D::ClosestPoint(A2, B2, point);

        float cp1Dist = Vector2::sqrDistance(cp1, point);
        float cp2Dist = Vector2::sqrDistance(cp2, point);

        Vector2 res = (cp1Dist <= cp2Dist) ? cp1 : cp2;
        float currentSqrDist = min(cp1Dist, cp2Dist);

        Vector2 diff1 = point - _circle1->center();
        Vector2 diff2 = point - _circle2->center();

        if (Vector2::dot(dirHori, diff1) > 0.0f) 
        {
            cp1 = _circle1->center() + Vector2::normalize(diff1) * _circle1->radius;
            cp1Dist = Vector2::sqrDistance(cp1, point);
            if (cp1Dist < currentSqrDist) 
            {
                res = cp1;
                currentSqrDist = cp1Dist;
            }
        }

        if (Vector2::dot(dirHori, diff2) > 0.0f)
        {
            cp2 = _circle2->center() + Vector2::normalize(diff2) * _circle2->radius;
            cp2Dist = Vector2::sqrDistance(cp2, point);
            if (cp2Dist < currentSqrDist) 
            {
                res = cp2;
            }
        }

        return res;
    }

    float Capsule::distance(const Vector2& point) const 
    {
        return contains(point) ? 0.0f : Vector2::distance(closestPoint(point), point);
    }

    float Capsule::signedDistance(const Vector2& point) const 
    {
        float d = distance(point);
        return contains(point) ? -d : d;
    }

    float Capsule::area() const 
    {
        return _circle1->area() + _hitbox->area();
    }

    bool Capsule::contains(const Vector2& point) const 
    {
        return _inclusiveCircle->contains(point) && (_circle1->contains(point) || _circle2->contains(point) || _hitbox->contains(point));
    }

    void Capsule::moveAt(const Vector2& pos) 
    {
        Vector2 d1 = _circle1->center() - center();
        Vector2 d2 = _circle2->center() - center();
        _hitbox->moveAt(pos);
        _circle1->moveAt(pos + d1);
        _circle2->moveAt(pos + d2);
        _inclusiveCircle->moveAt(pos);
    }

    void Capsule::rotate(float angle) 
    {
        Vector2 c = center();

        Vector2 offset1 = _circle1->center() - c;
        float len1 = offset1.length();
        float angle1 = Useful::angle(Vector2::zero(), offset1) + angle;
        _circle1->moveAt(c + Vector2(len1 * cos(angle1), len1 * sin(angle1)));

        Vector2 offset2 = _circle2->center() - c;
        float len2 = offset2.length();
        float angle2 = Useful::angle(Vector2::zero(), offset2) + angle;
        _circle2->moveAt(c + Vector2(len2 * cos(angle2), len2 * sin(angle2)));

        _hitbox->rotate(angle);
    }

    void Capsule::scale(const Vector2& scale)
    {
        _circle1->scale(scale);
        _circle2->scale(scale);
        _hitbox->scale(scale);

        _circle1->moveAt(center() + (_circle1->center() - center()) * scale);
        _circle2->moveAt(center() + (_circle2->center() - center()) * scale);

        _inclusiveCircle->scale(scale);
    }

    bool Capsule::normal(const Vector2& point, Vector2& outNormal) const 
    {
        if (_circle1->normal(point, outNormal))
            return true;
        if (_circle2->normal(point, outNormal))
            return true;
        if (_hitbox->normal(point, outNormal))
            return true;
        return Collider2D::normal(point, outNormal);
    }

    string Capsule::toString() const 
    {
        ostringstream oss;
        oss << "Center:" << center().to_string() << ", size:" << _hitbox->size().to_string() << ", rotation:" << angleHori();
        return oss.str();
    }

    Capsule::~Capsule()
    {
        delete _circle1;
        delete _circle2;
        delete _hitbox;
        delete _inclusiveCircle;
    }
}