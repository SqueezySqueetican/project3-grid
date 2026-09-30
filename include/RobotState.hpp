/**
 * @file RobotState.hpp
 * @brief Robot physical parameters and state definitions
 * 
 * Contains structs for robot geometry, velocity limits,
 * and navigation state enumeration.
 */


#pragma once


/**
 * @struct Twist
 * @brief Velocity command with linear and angular components
 */
struct Twist
{
    double linear_x{0.0};
    double angular_z{0.0};
};


/**
 * @struct RobotState
 * @brief Physical parameters and limits of the robot
 * 
 * @details Contains wheel geometry, velocity limits,
 *          and acceleration constraints for simulation.
 */
struct RobotState
{
    // Robot geometry
    double wheel_radius{0.1};               // [m]
    double wheel_separation{0.5};           // [m]

    // Velocity limits
    double max_linear_velocity{1.0};        // [m/s]
    double max_angular_velocity{1.5};       // [rad/s]

    // Acceleration limits
    double max_linear_acceleration{0.5};    // [m/s^2]
    double max_linear_deceleration{1.0};    // [m/s^2]

    double max_angular_acceleration{1.0};   // [rad/s^2]
    double max_angular_deceleration{1.5};   // [rad/s^2]
};


/**
 * @enum State
 * @brief Navigation states for the robot
 * 
 * - MOVING:    Moving toward target
 * - STOPPING:  Stopped (waiting)
 * - ROTATING:  Rotating in place
 * - FINISHED:  Path complete
 */
enum State { MOVING, STOPPING, ROTATING, FINISHED};