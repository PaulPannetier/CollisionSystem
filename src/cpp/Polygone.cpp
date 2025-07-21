#include <cmath>
#include <sstream>
#include "Polygone.hpp"
#include "Hitbox.hpp"
#include "Circle.hpp"
#include "Useful.hpp"
#include "Random.hpp"

using namespace std;

namespace ToricCollisionSystem
{
    Polygone::Polygone(const vector<Vector2>& vertices) : _vertices(vertices)
    {
        for (int i = static_cast<int>(_vertices.size()) - 1; i >= 0; i--)
        {
            if (_vertices[i] == _vertices[(i + 1) % vertices.size()])
            {
                _vertices.erase(vertices.begin() + i);
            }
        }

        _vertices.shrink_to_fit();

        _center = Vector2::zero();
        for (const Vector2& v : vertices) 
        {
            _center = _center + v;
        }

        if (!vertices.empty())
        {
            _center.x /= static_cast<float>(vertices.size());
            _center.y /= static_cast<float>(vertices.size());
        }

        ComputeInclusiveCircle();
    }

    Polygone::Polygone(const Polygone& polygone) : _center(polygone._center), _vertices(polygone._vertices)
    {
        _inclusiveCircle = new Circle(polygone._inclusiveCircle->center(), polygone._inclusiveCircle->radius);
    }

    void Polygone::ComputeInclusiveCircle()
    {
        float maxDist = 0.0f;
        for (uint32_t i = 0; i < _vertices.size(); i++)
        {
            maxDist = max(Vector2::distance(_center, _vertices[i]), maxDist);
        }

        if (_inclusiveCircle == nullptr)
        {
            _inclusiveCircle = new Circle(_center, maxDist);
        }
        else
        {
            _inclusiveCircle->center(_center);
            _inclusiveCircle->radius = maxDist;
        }
    }

    bool Polygone::IsNormalOnRightDirection(const Vector2& point, const Vector2& n, int indexSide) const
    {
        Vector2 A = point;
        Vector2 B = point + n * (2.0f * _inclusiveCircle->radius);
        uint32_t count = 0;

        for (uint32_t i = 0; i < _vertices.size(); i++) 
        {
            if (i == indexSide) 
                continue;
            if (Collider2D::CollideLines(_vertices[i], _vertices[(i + 1u) % _vertices.size()], A, B)) 
            {
                count++;
            }
        }

        return Useful::isEven(count);
    }

    bool Polygone::collide(const Collider2D& c) const 
    {
        //return Collider2D::Collide(c, *this);
        return false;
    }

    bool Polygone::collideLine(const Line2D& line) const
    {
        //return Collider2D::CollidePolygoneLine(*this, line.A, line.B);
        return false;
    }

    bool Polygone::collideStraightLine(const StraightLine2D& line) const
    {
        //return Collider2D::CollidePolygoneStaightLine(*this, line.A, line.B);
        return false;
    }

    bool Polygone::contains(const Vector2& point) const
    {
        if (_vertices.size() < 3)
            return false;

        if (!_inclusiveCircle->contains(point))
            return false;

        Vector2 I = _center + Random::Vector2(_inclusiveCircle->radius * 2.0f);
        int intersections = 0;

        for (uint32_t i = 0; i < _vertices.size(); i++)
        {
            const Vector2& A = _vertices[i];
            const Vector2& B = _vertices[(i + 1) % _vertices.size()];

            Vector2 D = B - A;
            Vector2 E = point - I;
            float denom = D.x * E.y - D.y * E.x;

            if (Useful::approximately(denom, 0.0f))
                return contains(point);

            float t = -(A.x * E.y - I.x * E.y - E.x * A.y + E.x * I.y) / denom;
            if (t < 0.0f || t >= 1.0f)
                continue;

            float u = -(-D.x * A.y + D.x * I.y + D.y * A.x - D.y * I.x) / denom;
            if (u < 0.0f || u >= 1.0f)
                continue;

            intersections++;
        }

        return Useful::isOdd(intersections);
    }

    float Polygone::distance(const Vector2& point) const
    {
        return contains(point) ? 0.0f : Vector2::distance(closestPoint(point), point);
    }

    float Polygone::signedDistance(const Vector2& point) const
    {
        Vector2 closest = closestPoint(point);
        float dist = Vector2::distance(closest, point);
        return contains(point) ? -dist : dist;
    }

    float Polygone::area() const
    {
        float sum = 0.0f;
        for (uint32_t i = 0; i < _vertices.size(); i++)
        {
            const Vector2& p1 = _vertices[i];
            const Vector2& p2 = _vertices[(i + 1) % _vertices.size()];
            sum += (p1.x * p2.y) - (p2.x * p1.y);
        }
        return 0.5f * abs(sum);
    }

    Vector2 Polygone::closestPoint(const Vector2& point) const
    {
        vector<Vector2> closest(_vertices.size());
        for (uint32_t i = 0u; i < _vertices.size(); i++)
        {
            closest[i] = Line2D::ClosestPoint(_vertices[i], _vertices[(i + 1) % _vertices.size()], point);
        }

        float minSqr = Vector2::sqrDistance(closest[0], point);
        uint32_t index = 0u;
        for (uint32_t i = 1u; i < closest.size(); i++)
        {
            float sqr = Vector2::sqrDistance(closest[i], point);
            if (sqr < minSqr)
            {
                minSqr = sqr;
                index = i;
            }
        }

        return closest[index];
    }

    void Polygone::moveAt(const Vector2& position) 
    {
        Vector2 oldCenter = _center;
        _center = position;
        for (Vector2& v : _vertices) 
        {
            v = _center + (v - oldCenter);
        }
        _inclusiveCircle->moveAt(position);
    }

    void Polygone::rotate(float angle)
    {
        for (Vector2& v : _vertices)
        {
            float dist = Vector2::distance(v, _center);
            float ang = Useful::angleHori(_center, v) + angle;
            v = Vector2(_center.x + dist * cos(ang), _center.y + dist * sin(ang));
        }
    }

    void Polygone::scale(const Vector2& scale) 
    {
        for (Vector2& v : _vertices) 
        {
            v = _center - (_center - v) * scale;
        }
        ComputeInclusiveCircle();
    }

    bool Polygone::normal(const Vector2& point, Vector2& outNormal) const 
    {
        uint32_t minIndex = 0;
        float minSqr = Line2D::SqrDistance(_vertices[0], _vertices[1], point);

        for (uint32_t i = 1u; i < _vertices.size(); i++)
        {
            float sqr = Line2D::SqrDistance(_vertices[i], _vertices[(i + 1) % _vertices.size()], point);
            if (sqr < minSqr) 
            {
                minSqr = sqr;
                minIndex = i;
            }
        }

        if (minSqr > 1e-2f) 
        {
            outNormal = Vector2::zero();
            return false;
        }

        outNormal = Line2D::Normal(_vertices[minIndex], _vertices[(minIndex + 1) % _vertices.size()]);
        if (!IsNormalOnRightDirection(point, outNormal, minIndex)) 
        {
            outNormal = -1.0f * outNormal;
        }

        return true;
    }

    string Polygone::toString() const 
    {
        ostringstream ss;
        ss << "Vertices number : " << _vertices.size() << "\nVertices :\n";
        for (uint32_t i = 0; i < _vertices.size(); ++i) 
        {
            ss << "    " << _vertices[i].to_string() << (i < _vertices.size() - 1 ? ",\n" : "\n");
        }
        return ss.str();
    }

    Polygone::~Polygone()
    {
        delete _inclusiveCircle;
    }
}