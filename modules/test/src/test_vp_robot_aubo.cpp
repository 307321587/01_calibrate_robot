#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/robot/vpRobot.h>
#include <iostream>
#include "vp_robot_aubo_robots.h"
#include <chrono>
#include <ostream>

using namespace std;
int main()
{
    vpRobotAuboRobots robot("192.168.123.96");
    robot.setPosition(vpRobot::JOINT_STATE, vpColVector{vpMath::rad(90), vpMath::rad(0), vpMath::rad(-90), vpMath::rad(0), vpMath::rad(-90), vpMath::rad(0)});
    // vpColVector pose;
    // robot.getPosition(vpRobot::END_EFFECTOR_FRAME, pose);
    // std::cout << "pose: " << pose.t() << std::endl;
    // pose[0] += 0.05;
    // std::cout << "pose: " << pose.t() << std::endl;
    // robot.setPosition(vpRobot::END_EFFECTOR_FRAME, pose);
    // char key = 'a';
    // while (std::cin >> key)
    // {
    //     if (key == 'q')
    //     {
    //         break;
    //     }
    // }
    robot.rtInit("192.168.123.96");
    int acc_sum_count = 40;
    vpColVector target_vel;
    target_vel << -0.01, 0, 0, -0.01, -0.01, 0.01;
    vpColVector max_acc = target_vel / acc_sum_count;
    vpColVector cur_vel;
    cur_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    int count = 0;
    int count_sum = 200;

    bool stop = false;
    auto last = chrono::system_clock::now();
    vpColVector position;
    // 输入不为q
    while (!stop)
    {
        auto now = chrono::system_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(now - last);
        int cost = duration.count();
        // cout << "cost:" << cost << "us" << endl;
        last = now;

        if (count == count_sum + acc_sum_count - 1)
        {
            stop = true;
        }

        if (count % count_sum == 0)
        {
            target_vel = -target_vel;
            max_acc = -max_acc;
        }

        if ((target_vel - cur_vel).frobeniusNorm() > 1e-6)
        {
            cur_vel += max_acc;
        }

        // cout << "cur:" << cur_vel.t() << endl;
        robot.setVelocity(vpRobot::END_EFFECTOR_FRAME, cur_vel);
        count++;
        position = robot.getPositionP(vpRobot::END_EFFECTOR_FRAME);
        cout << position.t() << endl;
        this_thread::sleep_for(chrono::microseconds(5000));
    }
    cout << "end" << endl;
    return 0;
}