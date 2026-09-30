#include "Grid.hpp"

Grid::Grid(int width, int height, double distance) 
    : w(width), h(height), dist(distance) {
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            Point dot;
            dot.x = i * dist;  
            dot.y = j * dist;  
            grid_dots.push_back(dot);
        }
    }
}

bool Grid::isInGrid(Point point) {
    if (point.x < 0 || point.x > (w - 1) * dist ||
        point.y < 0 || point.y > (h - 1) * dist) {
        return false;
    }
    for (Point obs : obstacle_dots) {
        if (point.x == obs.x && point.y == obs.y)
            return false;
    }
    return true;
}

void Grid::obstacle(Point obstacle_point) {
    grid_dots.remove(obstacle_point);
    obstacle_dots.push_back(obstacle_point);
}


list<Point> Grid::findNeighbors(Point point) {
    list<Point> neighbors;

    Point p5; p5.x = point.x + dist; p5.y = point.y;        neighbors.push_back(p5);
    Point p6; p6.x = point.x;        p6.y = point.y + dist; neighbors.push_back(p6);
    Point p7; p7.x = point.x - dist; p7.y = point.y;        neighbors.push_back(p7);
    Point p8; p8.x = point.x;        p8.y = point.y - dist; neighbors.push_back(p8);

    list<Point> valid_neighbors;
    for (Point n : neighbors) {
        if (isInGrid(n)) {
            valid_neighbors.push_back(n);
        }
    }
    return valid_neighbors;
}