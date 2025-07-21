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

    bool Hitbox::contains(const Vector2& point) const 
    {
        return _rec->contains(point);
    }

    float Hitbox::distance(const Vector2& point) const 
    {
        return _rec->distance(point);
    }

    float Hitbox::signedDistance(const Vector2& point) const 
    {
        return _rec->signedDistance(point);
    }

    float Hitbox::area() const 
    {
        return _size.x * _size.y;
    }

    Vector2 Hitbox::closestPoint(const Vector2& point) const 
    {
        return _rec->closestPoint(point);
    }

    void Hitbox::moveAt(const Vector2& position) 
    {
        _rec->moveAt(position);
    }

    void Hitbox::scale(const Vector2& scale) 
    {
        _rec->scale(scale);
        _size = { _size.x * scale.x, _size.y * scale.y };
    }

    void Hitbox::rotate(float angle) 
    {
        if (abs(angle) > FLT_EPSILON)
        {
            _rec->rotate(angle);
        }
    }

    float Hitbox::angleHori() const 
    {
        const vector<Vector2>& verts = _rec->vertices();
        return Useful::angleHori(_rec->center(), (verts[2] + verts[1]) * 0.5f);
    }

    const Polygone& Hitbox::toPolygone() const
    {
        return *_rec;
    }

    bool Hitbox::normal(const Vector2& point, Vector2& outNormal) const 
    {
        return _rec->normal(point, outNormal);
    }

    string Hitbox::toString() const 
    {
        ostringstream oss;
        oss << "Center:" << center().x << "," << center().y << " Size:" << _size.x << "," << _size.y << " rotation:" << angleHori();
        return oss.str();
    }

    Hitbox::~Hitbox()
    {
        delete _rec;
    }
}