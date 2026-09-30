/**
 * @file Simulator.hpp
 * @brief Robot dynamics simulator with velocity and acceleration limits
 * 
 * Simulates differential-drive robot motion with realistic constraints
 * on velocity and acceleration.
 */

#include <cmath>
#include <algorithm>
#include "RobotState.hpp"
#include "Point.hpp"
#include "PID.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @class Simulator
 * @brief Simulates robot motion with physical constraints
 * 
 * Applies velocity and acceleration limits to commanded velocities,
 * updates pose using differential drive kinematics.
 */
class Simulator
{
public:
    /**
     * @brief Constructor
     * @param parameters Robot physical parameters (wheel radius, limits, etc.)
     */
    explicit Simulator(const RobotState& parameters);

    /**
     * @brief Set velocity command (clamped to limits)
     * @param command Desired linear and angular velocity
     */
    void setVelocityCommand(const Twist& command);

    /**
     * @brief Update robot state by one time step
     * @param dt Time step in seconds
     * @details Applies acceleration limits, integrates pose
     */
    void update(double dt);

    // ---- Getters ----
    Point getPose() const;      ///< Get current pose (x, y, theta)
    Twist getVelocity() const;  ///< Get current velocity (linear, angular)

    /**
     * @brief Reset robot to initial state
     * @param pose    Initial pose (default: origin)
     * @param velocity Initial velocity (default: zero)
     */
    void reset(const Point& pose = Point{},
               const Twist& velocity = Twist{});

private:
    RobotState parameters_;     ///< Robot physical parameters

    Point pose_;                ///< Current pose
    Twist velocity_;            ///< Current velocity
    Twist commanded_velocity_;  ///< Desired velocity (before limits)

    /**
     * @brief Apply acceleration/deceleration limits to velocity
     * @param current      Current velocity
     * @param target       Desired velocity
     * @param acceleration Max acceleration (speed up)
     * @param deceleration Max deceleration (slow down)
     * @param dt           Time step
     * @return Limited velocity
     */
    double limitVelocity(double current, double target,
                         double acceleration, double deceleration,
                         double dt) const;

    /**
     * @brief Normalize angle to [-PI, PI]
     */
    static double normalizeAngle(double angle);
};