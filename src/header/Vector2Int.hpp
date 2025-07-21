#pragma once

namespace ToricCollisionSystem
{
    struct Vector2Int
    {
        int x, y;

        Vector2Int() : x(0.0f), y(0.0f)
        {

        }

        Vector2Int(int x, int y) : x(x), y(y)
        {

        }

        Vector2Int(const Vector2Int& vector) : x(vector.x), y(vector.y)
        {

        }
    };
}