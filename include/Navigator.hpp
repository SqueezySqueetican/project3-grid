/**
 * @file Navigator.hpp
 * @brief Path navigation and state management for robot
 * 
 * Tracks robot progress along a path, handles waypoint transitions,
 * and manages rotation states between waypoints.
 */


#include "Grid.hpp"
#include "RobotState.hpp"
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


/**
 * @class Navigator
 * @brief Manages path following with waypoint switching
 * 
 * Updates robot state (MOVING, ROTATING, FINISHED) based on
 * position relative to current and next waypoints.
 * 
 * @note Uses grid for obstacle checking (currently disabled)
 */
class Navigator {
private:
    Grid g;
    vector<Point> path;
    int target_index;
    State state;
    double pos_tolerance;
    double ang_tolerance;

public:

    /**
     * @brief Constructor
     * @param grid    Grid for obstacle checking
     * @param Path    Vector of waypoints
     * @param pos_tol Position tolerance (default: 0.02m)
     * @param ang_tol Angle tolerance (default: 0.01rad)
     */
    Navigator(Grid grid, std::vector<Point> Path, double pos_tol = 0.02, double ang_tol = 0.01);


    /**
     * @brief Update navigation state based on current robot position
     * @param position Current robot pose
     * 
     * @details Checks if reached current waypoint, advances to next,
     *          and switches to ROTATING state if direction change needed.
     */
    void update(Point position);


    /**
     * @brief Find best point to bypass obstacle (uses common neighbors)
     * @param obstacle Obstacle position
     * @param robot    Current robot position
     * @param nextGoal Next waypoint after obstacle
     * @return Best point closest to nextGoal
     */
    Point findBestPoint(Point obstacle, Point robot, Point nextGoal);



    Point stopBeforeObstacle(Point obstacle, Point robot); 

    double normalizeAngle(double angle);
    bool needToRotate(Point current, Point next, Point robot);


    State getState() ;  
    Point getTarget() ;  
    bool isFinished() ; 
    int getIndex() ;  
};
