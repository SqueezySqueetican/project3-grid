/**
 * @file Controller.hpp
 * @brief Main robot controller using two PID controllers
 * 
 * Generates velocity commands (linear and angular) based on robot pose,
 * target position, and current state.
 * 
 * @author mahmoudian
 * @date 2026-08-19     */

#include "grid.hpp"
#include "PID.hpp"
#include "RobotState.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * @class Controller
 * @brief Generates twist commands using two PID controllers
 * 
 * - linear_pid: controls linear velocity based on distance error
 * - angular_pid: controls angular velocity based on angle error
 * 
 */

class Controller {
private:
    PID linear_pid;
    PID angular_pid;

public:
    /*
     * @param kp_lin  Proportional gain for linear PID
     * @param ki_lin  Integral gain for linear PID
     * @param kd_lin  Derivative gain for linear PID
     * @param kp_ang  Proportional gain for angular PID
     * @param ki_ang  Integral gain for angular PID
     * @param kd_ang  Derivative gain for angular PID
     * @param dt      Time step in seconds
     */
    Controller(double kp_lin, double ki_lin, double kd_lin,
               double kp_ang, double ki_ang, double kd_ang,
               double dt);

    Twist computeCommand(Point robot, Point target, State state);

private:
    static double normalizeAngle(double angle);
};