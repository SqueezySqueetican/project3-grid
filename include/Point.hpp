/**
 * @file Point.hpp
 * @brief Point structure with x, y, theta coordinates
 */

#pragma once
#include <list>

struct Point
{
    double x{0.0};
    double y{0.0};
    double theta{0.0};

    bool operator==(Point point) {
        return (x == point.x && y == point.y && theta == point.theta);
    }
};