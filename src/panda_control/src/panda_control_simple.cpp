// Test node to move the robot to a pose commanded via service or topic


#include <chrono>
#include <iostream>
#include <thread>
#include <signal.h>

#include <ros/ros.h>

#include <franka/robot.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "examples_common.h"

#include <panda_control/GoToCartPose.h>


class CartesianController {

    public: 

    CartesianController(franka::Robot& robot);

    bool moveEECallback(panda_control::GoToCartPose::Request &req,
                        panda_control::GoToCartPose::Response &resp);

    private:

    franka::Robot* robot_ptr;

};

CartesianController::CartesianController(franka::Robot& robot)
{
    robot_ptr = &robot;
}

//  Implement the callback
bool CartesianController::moveEECallback(panda_control::GoToCartPose::Request &req,
                        panda_control::GoToCartPose::Response &resp)
{

    Eigen::Vector3d pos_f(req.target_pose.position.x, req.target_pose.position.y, req.target_pose.position.z);
    //  Quaternion in req is expressed in xyzw form
    Eigen::Quaternion<double> quat_f(req.target_pose.orientation.w, req.target_pose.orientation.x, req.target_pose.orientation.y, req.target_pose.orientation.z);
    quat_f.normalize();

    ROS_INFO_STREAM("EEF motion requested");
    ROS_INFO_STREAM("Target position: xyz " << pos_f(0) << " "
                                            << pos_f(1) << " "
                                            << pos_f(2)
                                            );
    ROS_INFO_STREAM("Target orientation: xyzw " << quat_f.x() << " "
                                                << quat_f.y() << " "
                                                << quat_f.z() << " "
                                                << quat_f.w()
                                                );
    ROS_WARN_STREAM("Starting movement towards target pose");

    Eigen::Affine3d pose_f = Eigen::Affine3d::Identity();
    pose_f.translation() = pos_f;
    pose_f.linear() = quat_f.toRotationMatrix();  

    // franka::CartesianPose is expressed as a 4x4 matrix (from ee frame to base frame), column major
    std::array<double, 16> pose_i;

    // Get duration from request
    double time = 0.0;
    double total_duration = std::abs(req.duration);

    //  Activate controller with a lambda function. For now, this just changes the orientation
    robot_ptr->control([&time, &pose_i, &pos_f, &total_duration](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {

        if (time == 0.0) {
            pose_i = robot_state.O_T_EE_c;
        }

        time += period.toSec();

        // Trajectory is a sin in 3 coordinates
        // The angle is smoothed over a cosine arc
        // This is not ensured to respect acceleration and velocity requirements!
        double angle = M_PI / 4 * (1 - std::cos(M_PI * time / total_duration));
        double new_x = pose_i[12] + std::sin(angle) * (pos_f(0) - pose_i[12]);
        double new_y = pose_i[13] + std::sin(angle) * (pos_f(1) - pose_i[13]);
        double new_z = pose_i[14] + std::sin(angle) * (pos_f(2) - pose_i[14]);

        std::array<double, 16> new_pose = pose_i;
        new_pose[12] = new_x;
        new_pose[13] = new_y;
        new_pose[14] = new_z;

      if (time >= total_duration) {
        ROS_INFO_STREAM("Motion is complete");
        return franka::MotionFinished(new_pose);
      }

      return new_pose;
    });

    // Assume movement was successful
    // TODO: how can we ascertain this?
    resp.success = true;

    return true;

}

// bool uselessCallback(panda_control::GoToCartPose::Request &req,
//                         panda_control::GoToCartPose::Response &resp)
// {
//     resp.success=true;
//     return true;   
// }

void sigIntHandler(int sig)
{
    ROS_INFO_STREAM("Time to die.");
    ros::shutdown();
}

int main(int argc, char *argv[])
{

    // Temporary, we might want to fetch this from a config file or rosparam server
    // const std::string ROBOT_IP = "172.16.0.2";
    std::string robot_ip;

    // Initialize ROS and declare node
    ros::init(argc, argv, "panda_cartesian_controller");
    ros::NodeHandle nh("~");

    // Parse parameters
    nh.param<std::string>("robot_ip", robot_ip, "172.16.0.2");

    ROS_INFO_STREAM("Robot ip: " << robot_ip);
    ROS_INFO_STREAM("Panda control node successfully created!");

    // fake server
    // ros::ServiceServer server = nh.advertiseService("cartesian_target_pose", 
    //                                             &uselessCallback);

    try {

        // Open network connection with the robot
        franka::Robot robot(robot_ip);

        ROS_INFO_STREAM("Network connection with the robot established.");

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

        ROS_WARN_STREAM("Robot is being homed.");

        robot.control(motion_generator);

        ROS_WARN_STREAM("Finished moving to initial joint configuration.");

        CartesianController controller(robot);

        ros::ServiceServer server = nh.advertiseService("cartesian_target_pose", 
                                                        &CartesianController::moveEECallback, 
                                                        &controller);

        signal(SIGINT, sigIntHandler);
        ros::spin();

    }
    catch (const franka::Exception& e)
    {
        ROS_INFO_STREAM(e.what());
        return -1;
    }

    return 0;
}



