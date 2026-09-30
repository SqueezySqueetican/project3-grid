#include "Point.hpp"
#include "Path.hpp"
#include "Grid.hpp"
#include "RobotState.hpp"
#include "PID.hpp"
#include "Simulator.hpp"
#include "Navigator.hpp"
#include "Controller.hpp"

#include <fstream>
#include <iostream>

int main() {
    Grid grid(10, 10, 1); 


    // Define paths
    
    // PATH NO.1 (0,0) → (1,0) → (2,0) → (2,1)
    vector<Point> path1;
    Point p1; p1.x = 0; p1.y = 0; path1.push_back(p1);
    Point p2; p2.x = 1; p2.y = 0; path1.push_back(p2);
    Point p3; p3.x = 2; p3.y = 0; path1.push_back(p3);
    Point p4; p4.x = 2; p4.y = 1; path1.push_back(p4);

    // PATH NO.2 (0,0) → (3,0) → (3,3) → (0,3)
    // (0,0) → (0,3) → (3,3) → (3,0)

    vector<Point> path2;
    Point p5; p5.x = 0; p5.y = 0; path2.push_back(p5);
    Point p6; p6.x = 0; p6.y = 3; path2.push_back(p6);
    Point p7; p7.x = 3; p7.y = 3; path2.push_back(p7);
    Point p8; p8.x = 3; p8.y = 0; path2.push_back(p8);
    // Point p9; p9.x = 0; p9.y = 6; path2.push_back(p9);
    // Point p10; p10.x = 1; p10.y = 6; path2.push_back(p10);
    // Point p11; p11.x = 1; p11.y = 2; path2.push_back(p11);
    // Point p12; p12.x = 4; p12.y = 2; path2.push_back(p12); 
    // Point p18; p18.x = 2; p18.y = 2; path2.push_back(p18); 
 


    // PATH NO.3 (0,0) → (3,0) → (3,2) → (1,2) → (1,4)
    vector<Point> path3;
    Point p13; p13.x = 0; p13.y = 0; path3.push_back(p13);
    Point p14; p14.x = 3; p14.y = 0; path3.push_back(p14);
    Point p15; p15.x = 3; p15.y = 2; path3.push_back(p15);
    Point p16; p16.x = 1; p16.y = 2; path3.push_back(p16);
    Point p17; p17.x = 1; p17.y = 4; path3.push_back(p17);

    
    // set path
    vector<Point> path = path2;

    // add obstacle
    // Point obs1; obs1.x = 3; obs1.y = 3; grid.obstacle(obs1);
   

    // Initialize robot and controllers
    RobotState parameters;
    Simulator robot(parameters);
    double dt = 0.01;

    Controller controller(1.0, 0.0, 0.0,    // linear PID
                          2.5, 0.0, 0.0,    // angular PID
                          dt);
    
    Navigator navigator(grid, path, 0.002, 0.02);  


    // save output in csv file
    ofstream file("58robot(P1-P2.5)path22.csv");
    file << "Time,x,y,theta,target_x,target_y,error_dist,error_angle,cmd_v,cmd_omega,current_v,current_omega,state\n";


    // simulation loop
    for (int i = 0; i < 7000; i++) {
        Point position = robot.getPose();

        navigator.update(position);
        
        if (navigator.isFinished()) {
            cout << "Mission Complete!\n";
            break;
        }

        Point target = navigator.getTarget();
        State state = navigator.getState();

        double dx = target.x - position.x;
        double dy = target.y - position.y;
        double error_distance = sqrt(dx*dx + dy*dy);
        double target_angle = atan2(dy, dx);
        double error_angle = target_angle - position.theta;

        // Compute and apply command
        Twist cmd = controller.computeCommand(position, target, state);
        robot.setVelocityCommand(cmd);
        robot.update(dt);

        // Get new state
        Point new_pose = robot.getPose();
        Twist new_vel = robot.getVelocity();

        // saving data
        double time = i * dt;
        file << time << ","
             << new_pose.x << ","
             << new_pose.y << ","
             << new_pose.theta << ","
             << target.x << ","
             << target.y << ","
             << error_distance << ","
             << error_angle << ","
             << cmd.linear_x << ","
             << cmd.angular_z << ","
             << new_vel.linear_x << ","
             << new_vel.angular_z << ","
             << state << "\n";


        //print data each 10 step
        if (i % 100 == 0) {
            cout << "Time: " << time*10 << "s "
                 << "| Pos: (" << new_pose.x << ", " << new_pose.y << ")\t"
                 << "| Target: (" << target.x << "," << target.y<< ")\t"
                 << "| angl err: " << error_angle << "\t"
                 << "| State: " << state << endl;
        }
    }

    file.close();
    return 0;
}