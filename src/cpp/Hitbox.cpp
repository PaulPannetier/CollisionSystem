#include <cmath>
#include <sstream>
#include "Collider2D.hpp"
#include "Hitbox.hpp"
#include "Circle.hpp"
#include "Polygone.hpp"
#include "Useful.hpp"

using namespace std;

namespace ToricCollisionSystem
{
    Hitbox::Hitbox(const Vector2& center, const Vector2& size) : _size(size)
    {
        vector<Vector2> vertices = 
        {
            { center.x - size.x * 0.5f, center.y - size.y * 0.5f },
            { center.x + size.x * 0.5f, center.y - size.y * 0.5f },
            { center.x + size.x * 0.5f, center.y + size.y * 0.5f },
            { center.x - size.x * 0.5f, center.y + size.y * 0.5f }
        };
        _rec = new Polygone(vertices);
    }

    Hitbox::Hitbox(const Hitbox& hitbox) : _size(hitbox._size)
    {
        _rec = new Polygone(*hitbox._rec);
    }

    const Vector2& Hitbox::center() const 
    {
        return _rec->center();
    }

    const Vector2& Hitbox::size() const
    {
        return _size;
    }

    const Circle& Hitbox::inclusiveCircle() const
    {
        return _rec->inclusiveCircle();
    }

    Polygone* Hitbox::toPolygone() const
    {
        return new Polygone(*_rec);
    }

    bool Hitbox::Collide(const Collider2D& other) const 
    {
        //return Collider2D::Collide(other, *this);
        return false;
    }

    bool Hitbox::CollideLine(const Line2D& line) const 
    {
        //return Collider2D::CollideHitboxLine(*this, line);
        return false;
    }

    bool Hitbox::CollideStraightLine(const StraightLine2D& line) const
    {
        //return Collider2D::CollideHitboxStraightLine(*this, line);
        return false;
    }

    bool Hitbox::Contains(const Vector2& point) const 
    {
        return _rec->Contains(point);
    }

    float Hitbox::Distance(const Vector2& point) const 
    {
        return _rec->Distance(point);
    }

    float Hitbox::SignedDistance(const Vector2& point) const 
    {
        return _rec->SignedDistance(point);
    }

    float Hitbox::Area() const 
    {
        return _size.x * _size.y;
    }

    Vector2 Hitbox::ClosestPoint(const Vector2& point) const 
    {
        return _rec->ClosestPoint(point);
    }

    void Hitbox::MoveAt(const Vector2& position) 
    {
        _rec->MoveAt(position);
    }

    void Hitbox::Scale(const Vector2& scale) 
    {
        _rec->Scale(scale);
        _size = { _size.x * scale.x, _size.y * scale.y };
    }

    void Hitbox::Rotate(float angle) 
    {
        if (abs(angle) > FLT_EPSILON)
        {
            _rec->Rotate(angle);
        }
    }

    float Hitbox::AngleHori() const 
    {
        const vector<Vector2>& verts = _rec->vertices();
        return Useful::angleHori(_rec->center(), (verts[2] + verts[1]) * 0.5f);
    }

    Hitbox* Hitbox::ToHitbox() const
    {
        return new Hitbox(*this);
    }

    const Polygone& Hitbox::ToPolygone() const
    {
        return *_rec;
    }

    bool Hitbox::Normal(const Vector2& point, Vector2& outNormal) const 
    {
        return _rec->Normal(point, outNormal);
    }

    string Hitbox::ToString() const 
    {
        ostringstream oss;
        oss << "Center:" << center().x << "," << center().y << " Size:" << _size.x << "," << _size.y << " rotation:" << AngleHori();
        return oss.str();
    }

    Hitbox::~Hitbox()
    {
        delete _rec;
    }
}