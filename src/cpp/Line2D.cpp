#include "Line2D.hpp"
#include "StraightLine2D.hpp"
#include "Useful.hpp"

using namespace std;

namespace ToricCollisionSystem 
{
    Line2D::Line2D(const Vector2& A, const Vector2& B) : A(A), B(B)
    {
    
    }

    Line2D::Line2D(const Vector2& start, float angle, float length) : A(start), B(start.x + length * cos(angle), start.y + length * sin(angle))
    {

    }

    bool Line2D::Contain(const Vector2& point) const 
    {
        return Contain(A, B, point);
    }

    bool Line2D::Contain(const Vector2& A, const Vector2& B, const Vector2& point) 
    {
        if (fabs(A.x - B.x) < 1e-2f) 
        {
            return Useful::approximately((A.x + B.x) * 0.5f, point.x) && min(A.y, B.y) <= point.y && point.y <= max(A.y, B.y);
        }
        if (min(A.x, B.x) > point.x || max(A.x, B.x) < point.x || min(A.y, B.y) > point.y || max(A.y, B.y) < point.y)
        {
            return false;
        }
        float a = (B.y - A.y) / (B.x - A.x);
        float b = A.y - a * A.x;
        return fabs(a * point.x + b - point.y) < 1e-3f;
    }

    float Line2D::Distance(const Vector2& point) const 
    {
        return Distance(A, B, point);
    }

    float Line2D::Distance(const Vector2& A, const Vector2& B, const Vector2& point) 
    {
        return sqrt(SqrDistance(A, B, point));
    }

    float Line2D::SqrDistance(const Vector2& point) const 
    {
        return SqrDistance(A, B, point);
    }

    float Line2D::SqrDistance(const Vector2& A, const Vector2& B, const Vector2& point) 
    {
        float r = (((point.x - A.x) * (B.x - A.x)) + ((point.y - A.y) * (B.y - A.y))) / Vector2::sqrDistance(A, B);
        Vector2 P = A + r * (B - A);
        if (0.0f <= r && r <= 1.0f)
            return Vector2::sqrDistance(P, point);
        return r < 0.0f ? Vector2::sqrDistance(A, point) : Vector2::sqrDistance(B, point);
    }

    Vector2 Line2D::Normal() const 
    {
        return Normal(A, B);
    }

    Vector2 Line2D::Normal(const Vector2& A, const Vector2& B) 
    {
        if (fabs(A.x - B.x) < 1e-2f)
            return { 1.0f, 0.0f };
        return (B - A).normalVector();
    }

    Vector2 Line2D::ClosestPoint(const Vector2& point) const 
    {
        return ClosestPoint(A, B, point);
    }

    Vector2 Line2D::ClosestPoint(const Vector2& A, const Vector2& B, const Vector2& point) 
    {
        Vector2 orthProj = StraightLine2D::OrthogonalProjection(point, A, B);
        if (orthProj.x >= min(A.x, B.x) && orthProj.x <= max(A.x, B.x) && orthProj.y >= min(A.y, B.y) && orthProj.y <= max(A.y, B.y)) 
        {
            return orthProj;
        }
        return Vector2::sqrDistance(A, point) <= Vector2::sqrDistance(B, point) ? A : B;
    }
}
