#pragma once

#include <string>

namespace ToricCollisionSystem
{
    struct Vector2
    {
    public:
        static float distance(const Vector2& v1, const Vector2& v2);
        static float sqrDistance(const Vector2& v1, const Vector2& v2);
        static float dot(const Vector2& v1, const Vector2& v2);
        static Vector2 zero() { return { 0.0f, 0.0f }; }

        float x, y;

        Vector2() : x(0.0f), y(0.0f)
        {

        }

        Vector2(float x, float y) : x(x), y(y)
        {

        }

        Vector2(const Vector2& vector) : x(vector.x), y(vector.y)
        {

        }

        Vector2 normalVector();
        static Vector2 normalize(const Vector2& vector);
        float length();
        float sqr_length();
        void normalize();
        std::string to_string() const;
    };

    static Vector2 operator*(const Vector2& vector, float other)
    {
        return { vector.x * other, vector.y * other };
    }

    static Vector2 operator*(float other, const Vector2& vector)
    {
        return { vector.x * other, vector.y * other };
    }

    static Vector2 operator*(const Vector2& left, const Vector2& right)
    {
        return { left.x * right.x, left.y * right.y };
    }

    static Vector2 operator/(const Vector2& vector, float other)
    {
        float scalar = 1.0f / other;
        return { vector.x * scalar, vector.y * scalar };
    }

    static Vector2 operator/(const Vector2& left, const Vector2& right)
    {
        return { left.x / right.x, left.y / right.y };
    }

    static Vector2 operator+(const Vector2& left, const Vector2& right)
    {
        return { left.x + right.x, left.y + right.y };
    }

    static Vector2 operator-(const Vector2& left, const Vector2& right)
    {
        return { left.x - right.x, left.y - right.y };
    }

    static bool operator==(const Vector2& left, const Vector2& right)
    {
        return left.x == right.x && left.y == right.y;
    }

    static bool operator!=(const Vector2& left, const Vector2& right)
    {
        return left.x != right.x || left.y != right.y;
    }
}
