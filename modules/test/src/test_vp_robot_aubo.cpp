#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoseVector.h>
#include <iostream>
#include "vp_robot_aubo_robots.h"

int main()
{
    vpRobotAuboRobots robot("192.168.123.96", true);
    // robot.setPosition(vpRobot::JOINT_STATE, vpColVector{vpMath::rad(90), vpMath::rad(0), vpMath::rad(-90), vpMath::rad(0), vpMath::rad(-90), vpMath::rad(0)});
    // vpColVector pose;
    // robot.getPosition(vpRobot::END_EFFECTOR_FRAME, pose);
    // std::cout << "pose: " << pose.t() << std::endl;
    // pose[0] += 0.05;
    // std::cout << "pose: " << pose.t() << std::endl;
    // robot.setPosition(vpRobot::END_EFFECTOR_FRAME, pose);
    char key = 'a';
    while (std::cin >> key)
    {
        if (key == 'q')
        {
            break;
        }
    }
    return 0;
}