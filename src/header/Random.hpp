#pragma once

#define _USE_MATH_DEFINES

#include <random>
#include <chrono>
#include <math.h>
#include <utility>  // for std::pair
#include "Vector2.hpp"

namespace ToricCollisionSystem
{
    class Random
    {
    private:
        static std::mt19937& engine() 
        {
            static std::mt19937 rng(static_cast<uint32_t>(
                std::chrono::high_resolution_clock::now().time_since_epoch().count()));
            return rng;
        }
    public:

        static void SetSeed(int seed)
        {
            engine().seed(seed);
        }

        static void SetRandomSeed()
        {
            SetSeed(static_cast<int>(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
        }

        // Random int in [a, b]
        static int Rand(int a, int b)
        {
            std::uniform_int_distribution<int> dist(a, b);
            return dist(engine());
        }

        // Random float in [0.0, 1.0]
        static float Rand()
        {
            return static_cast<float>(engine()()) / static_cast<float>(engine().max());
        }

        // Random float in [a, b]
        static float Rand(float a, float b)
        {
            std::uniform_real_distribution<float> dist(a, b);
            return dist(engine());
        }

        // Random int in [a, b)
        static int RandExclude(int a, int b)
        {
            std::uniform_int_distribution<int> dist(a, b - 1);
            return dist(engine());
        }

        // Random float in [a, b)
        static float RandExclude(float a, float b)
        {
            std::uniform_real_distribution<float> dist(a, b);
            return dist(engine());
        }

        // Random float in [0.0, 1.0)
        static float RandExclude()
        {
            std::uniform_real_distribution<float> dist(0.0f, 1.0f);
            return dist(engine());
        }

        // Random unit vector on the unit circle
        static ToricCollisionSystem::Vector2 Vector2()
        {
            float angle = RandExclude(0.0f, 2.0f * M_PI);
            return { std::cos(angle), std::sin(angle) };
        }

        // Random vector with specified magnitude
        static ToricCollisionSystem::Vector2 Vector2(float magnitude)
        {
            float angle = RandExclude(0.0f, 2.0f * M_PI);
            return { magnitude * std::cos(angle), magnitude * std::sin(angle) };
        }
    };
}