#pragma once

#include "Vector2.hpp"

namespace ToricCollisionSystem
{
    class Ray2D 
    {
    public:
        Vector2 start, end;

        Ray2D(const Vector2& start, const Vector2& end);
    };
}
