#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include <math.h>
#include <cfloat>
#include <type_traits>
#include <vector>
#include <algorithm>
#include "Vector2.hpp"

namespace ToricCollisionSystem
{
    class Useful 
    {
    public:

        static bool approximately(float a, float b)
        {
            return std::fabs(b - a) < std::fmax(1e-6f * std::fmax(std::fabs(a), std::fabs(b)), FLT_EPSILON * 8.0f);
        }

        static constexpr bool isOdd(int number) 
        {
            return (number & 1) != 0;
        }

        static constexpr bool isEven(int number)
        {
            return (number & 1) == 0;
        }

        // Returns angle in [0, 2π] between (1, 0) and (b - a)
        static float angleHori(const Vector2& a, const Vector2& b)
        {
            return std::atan2(a.y - b.y, a.x - b.x) + static_cast<float>(M_PI);
        }

        // Returns angle in [-π, π] between vector a and b
        static float angle(const Vector2& a, const Vector2& b)
        {
            return clampModulo(-static_cast<float>(M_PI), static_cast<float>(M_PI), angleHori(Vector2::zero(), a) + angleHori(Vector2::zero(), b));
        }

        // Wraps a value between start (inclusive) and end (exclusive)
        static float clampModulo(float start, float end, float value) 
        {
            if (end < start)
                return clampModulo(end, start, value);
            if (end - start < FLT_EPSILON)
                return start;

            if (value >= start && value < end)
                return value;

            float modulo = end - start;
            float result = std::fmod(value - start, modulo);
            if (result < 0.0f)
                result += modulo;
            return result + start;
        }

        std::vector<Vector2> Merge(const std::vector<Vector2>& vector1, const std::vector<Vector2>& vector2)
        {
            std::vector<Vector2> result;
            result.reserve(vector1.size() + vector2.size());

            for (const Vector2& v : vector1)
                result.push_back(v);

            for (const Vector2& v : vector1)
                result.push_back(v);

            return result;
        }

        std::vector<Vector2> Distinct(const std::vector<Vector2>& input) 
        {
            std::vector<Vector2> result;
            result.reserve(input.size());

            for (const Vector2& item : input) {
                if (std::find(result.begin(), result.end(), item) == result.end())
                    result.push_back(item);
            }

            return result;
        }

        std::vector<Vector2> Clone(const std::vector<Vector2>& input) 
        {
            std::vector<Vector2> result;
            result.reserve(input.size());

            for (const Vector2& v : input)
                result.push_back(v);

            return result;
        }
    };
}
