#include <cmath>
#include <algorithm>
#include "StraightLine2D.hpp"
#include "Useful.hpp"

using namespace std;

namespace ToricCollisionSystem 
{
    StraightLine2D::StraightLine2D(const Vector2& A, const Vector2& B) : A(A), B(B)
    {
    
    }

    Vector2 StraightLine2D::Symetric(const Vector2& M) const
    {
        return Symetric(M, A, B);
    }

    Vector2 StraightLine2D::Symetric(const Vector2& M, const Vector2& A, const Vector2& B)
    {
        if (abs(A.x - B.x) < 1e-3f)
        {
            float x = M.x >= (A.x - B.x) * 0.5f ? M.x - 2.0f * Distance(A, B, M) : M.x + 2.0f * Distance(A, B, M);
            return { x, M.y };
        }
        return 2.0f * OrthogonalProjection(M, A, B) - M;
    }

    Vector2 StraightLine2D::Reflection(const Vector2& normal, const Vector2& interPointInDroite, const Vector2& initDir)
    {
        return initDir - 2.0f * Vector2::dot(initDir, normal) * normal;
    }

    Vector2 StraightLine2D::OrthogonalProjection(const Vector2& point) const
    {
        return OrthogonalProjection(point, A, B);
    }

    Vector2 StraightLine2D::OrthogonalProjection(const Vector2& point, const Vector2& A, const Vector2& B)
    {
        if (abs(A.x - B.x) < 1e-2f)
        {
            return { (A.x + B.x) * 0.5f, point.y };
        }

        float r = (((point.x - A.x) * (B.x - A.x)) + ((point.y - A.y) * (B.y - A.y))) / Vector2::sqrDistance(A, B);
        return A + r * (B - A);
    }

    bool StraightLine2D::Contain(const Vector2& point) const 
    {
        return Contain(A, B, point);
    }

    bool StraightLine2D::Contain(const Vector2& A, const Vector2& B, const Vector2& point) 
    {
        if (abs(A.x - B.x) < 1e-2f)
        {
            return Useful::approximately((A.x + B.x) * 0.5f, point.x);
        }
        float a = (B.y - A.y) / (B.x - A.x);
        float b = A.y - a * A.x;
        return abs(a * point.x + b - point.y) < 1e-3f;
    }

    float StraightLine2D::Distance(const Vector2& point) const 
    {
        return Distance(A, B, point);
    }

    float StraightLine2D::Distance(const Vector2& A, const Vector2& B, const Vector2& point)
    {
        if (abs(A.x - B.x) < 1e-2f)
        {
            return abs((A.x + B.x) * 0.5f - point.x);
        }
        float a = (B.y - A.y) / (B.x - A.x);
        float b = A.y - a * A.x;
        return abs(a * point.x - point.y + b) / sqrt(a * a + 1.0f);
    }

    Vector2 StraightLine2D::Normal() const 
    {
        return Normal(A, B);
    }

    Vector2 StraightLine2D::Normal(const Vector2& A, const Vector2& B) 
    {
        if (abs(A.x - B.x) < 1e-2f)
            return { 1.0f, 0.0f };
        Vector2 res((B.y - A.y) / (B.x - A.x), -1.0f);
        res.normalize();
        return res;
    }

    Vector2 StraightLine2D::ClosestPoint(const Vector2& point) const
    {
        return OrthogonalProjection(point, A, B);
    }

    Vector2 StraightLine2D::ClosestPoint(const Vector2& A, const Vector2& B, const Vector2& point)
    {
        return OrthogonalProjection(point, A, B);
    }
}
