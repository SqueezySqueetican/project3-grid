/**
 * @file Grid.hpp
 * @brief 2D grid for robot path planning with obstacles
 */

#pragma once

#include <cmath>
#include <list>
#include "Point.hpp"

using namespace std;

/**
 * @class Grid
 * @brief Grid with equally spaced points and obstacle support
 */
class Grid {
public:

    int w, h;            ///< weight and high of grid
    double dist;         ///< Distance between adjacent points
    list<Point> grid_dots;
    list<Point> obstacle_dots;

    Grid(int width, int height, double distance);


    /**
     * @brief Check if point is inside grid and not on obstacle
     * @param point Point to check
     * @return true if valid (inside grid and not obstacle)
     */
    bool isInGrid(Point point);


    /**
     * @brief Add obstacle at specified point
     * @param obstacle_point Point to mark as obstacle
     */
    void obstacle(Point obstacle_point);


    /**
     * @brief Find 4-directional neighbors of a point
     * @param point Center point
     * @return List of valid neighbors (inside grid, not obstacles)
     */
    list<Point> findNeighbors(Point point);
};