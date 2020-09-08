// Test node to move the robot to a pose commanded via service or topic


#include <chrono>
#include <iostream>
#include <thread>

#include <ros/ros.h>

#include <franka/robot.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include <Eigen/Core>

#include "examples_common.h"

int main(int argc, char *argv[])
{

    const std::string ROBOT_IP = "178.16.0.2";

    // Initialize ROS and declare node
    ros::init(argc, argv, "panda_control_simple");
    ros::NodeHandle nh;

    try {

        franka::Robot robot(ROBOT_IP);

        // Set robot collision behavior
        setDefaultBehavior(robot);

        robot.setCollisionBehavior(
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

        // Simply move the robot to a fixed joint config
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);

        ROS_WARN_STREAM( "WARNING: This example will move the robot! " << std::endl
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..."
        );
        std::cin.ignore();

        robot.control(motion_generator);

        std::cout << "Finished moving to initial joint configuration." << std::endl << std::endl;

    }
    catch (const franka::Exception& e)
    {
        ROS_INFO_STREAM(e.what() << std::endl);
        return -1;
    }





    ROS_INFO_STREAM("Panda control node successfully created!");

    ROS_INFO_STREAM("Time to die.");

    return 0;
}



