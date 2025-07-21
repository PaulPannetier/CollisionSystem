#include <sstream>
#include "Vector2.hpp"
#include "Useful.hpp"

using namespace std;

namespace ToricCollisionSystem
{
    float Vector2::distance(const Vector2& v1, const Vector2& v2)
    {
        float tmpX = v1.x - v2.x;
        float tmpY = v1.y - v2.y;
        return sqrt(tmpX * tmpX + (tmpY * tmpY));
    }

    float Vector2::sqrDistance(const Vector2& v1, const Vector2& v2)
    {
        float tmpX = v1.x - v2.x;
        float tmpY = v1.y - v2.y;
        return tmpX * tmpX + (tmpY * tmpY);
    }

    float Vector2::dot(const Vector2& v1, const Vector2& v2)
    {
        return v1.x * v2.x + (v1.y * v2.y);
    }

    Vector2 Vector2::normalize(const Vector2& vector)
    {
        float oneOlength = 1.0f / sqrt(vector.x * vector.x + (vector.y * vector.y));
        return { vector.x * oneOlength, vector.y * oneOlength };
    }

    float Vector2::sqr_length()
    {
        return x * x + (y * y);
    }

    float Vector2::length()
    {
        return sqrt(x * x + (y * y));
    }

    Vector2 Vector2::NormalVector()
    {
        if (!Useful::approximately(x, 0.0f))
        {
            float y = sqrt(1.0f / (((this->y * this->y) / (x * x)) + 1.0f));
            return { -this->y * y / x, y };
        }
        else if (!Useful::approximately(y, 0.0f))
        {
            float x = sqrt(1.0f / (1.0f + (this->x * this->x) / (y * y)));
            return { x, -this->x * x / y };
        }
        else
        {
            return { x, y };
        }
    }

    void Vector2::Normalize()
    {
        float length = sqrt(x * x + (y * y));
        this->x /= length;
        this->y /= length;
    }

    std::string Vector2::to_string() const
    {
        ostringstream ss;
        ss << '('<< x << ", " << y << ')';
        return ss.str();
    }
}

